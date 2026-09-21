"""Random 2D room generator producing a camera-realistic scene-graph JSON.

World frame: meters, origin = bottom-left inner corner, x right, y up, yaw rad CCW.
Every object footprint is a convex polygon with 3..6 vertices (CCW order).
Output schema: see README.md (scene JSON v2.0).
"""
import argparse
import json
import math
import random
import sys
import time

SCHEMA_VERSION = "2.0"
# Robot shape (user drawing 2026-09-11). Body frame: +x forward (wide flat edge), CCW order.
# Hexagonal chassis, 34 cm long, 24 cm wide at the front, 10 cm at the rear.
ROBOT_CHASSIS_M = [(0.145, -0.122), (0.145, 0.122), (0.051, 0.173),
                   (-0.195, 0.05), (-0.195, -0.05), (0.051, -0.173)]
ROBOT_WHEEL_RADIUS_POS_M = 0.21     # wheel center distance from robot center (L)
ROBOT_WHEEL_ANGLES_DEG = (60, 180, 300)
ROBOT_WHEEL_LEN_M = 0.08            # wheel footprint length (tangential)
ROBOT_WHEEL_WIDTH_M = 0.03          # wheel footprint width (radial)
ROBOT_RADIUS_M = ROBOT_WHEEL_RADIUS_POS_M + ROBOT_WHEEL_WIDTH_M / 2   # bounding circle 0.225
WALL_MARGIN_M = 0.05          # gap kept between an object and a wall
OBJECT_GAP_M = 0.10           # minimum gap between two objects
ROOM_SIZE_RANGE_M = (4.0, 7.0)
DOOR_WIDTH_RANGE_M = (0.8, 1.0)
DOOR_END_MARGIN_M = 0.3       # door never closer than this to a corner
MAX_PLACE_TRIES = 200
MAX_POLYGON_VERTICES = 6
NEAR_COUNT = 2
CONFIDENCE_RANGE = (0.70, 0.99)
ROUND_DIGITS = 3
MAX_RANDOM_SEED = 1_000_000
DISC_VERTICES = 8            # generic disc approximation (unused by the robot now)
BED_WALL_EXTRA_M = 0.02      # bed sits this much further from the wall than WALL_MARGIN_M

# class -> (side_x range, side_y range, vertex count, colors)
FURNITURE_SPEC = {
    "bed":   ((1.8, 2.0), (1.2, 1.6), 4, ["blue", "white", "gray"]),
    "table": ((1.0, 1.6), (0.7, 0.9), 4, ["brown", "red", "black"]),
    "chair": ((0.40, 0.50), (0.40, 0.50), 4, ["black", "brown", "gray"]),
    "fan":   ((0.35, 0.45), (0.35, 0.45), 6, ["white", "gray"]),
}
# class -> (side_x range, side_y range, (min_vertices, max_vertices), colors)
MINI_SPEC = {
    "cup":    ((0.08, 0.12), (0.08, 0.12), (4, 6), ["red", "white", "blue"]),
    "bottle": ((0.06, 0.08), (0.06, 0.08), (5, 6), ["green", "blue"]),
    "ball":   ((0.15, 0.25), (0.15, 0.25), (6, 6), ["orange", "red", "yellow"]),
    "box":    ((0.25, 0.40), (0.20, 0.35), (4, 4), ["brown", "gray"]),
    "plant":  ((0.25, 0.35), (0.25, 0.35), (5, 6), ["green"]),
    "book":   ((0.20, 0.25), (0.14, 0.18), (4, 4), ["red", "blue", "yellow"]),
}
MINI_COUNT_RANGE = (3, 6)
MINI_GAP_M = 0.05
CHAIR_COUNT_RANGE = (1, 3)
CHAIR_OFFSET_M = 0.15         # chair distance from the table edge
CHAIR_GAP_M = 0.05
TABLE_WALL_MARGIN_M = 1.0     # table center stays this far from walls
FAN_CORNER_OFFSET_M = 0.45
FAN_CORNER_JITTER_M = 0.15
FREE_SIDE_CLEARANCE_M = 0.05
ROBOT_DOOR_OFFSET_M = 0.15    # robot spawns radius + this inside the door
DOOR_ZONE_EXTRA_M = 0.4       # free depth kept inside the door beyond the robot
SIDE_NAMES = ("front", "left", "back", "right")   # +x, +y, -x, -y in the object frame
WALL_ORDER = ("south", "east", "north", "west")


# ----------------------------------------------------------------- geometry
def _rotate(x, y, yaw, cx, cy):
    c, s = math.cos(yaw), math.sin(yaw)
    return (cx + c * x - s * y, cy + s * x + c * y)


def regular_polygon(cx, cy, rx, ry, n, yaw):
    """Convex n-gon inscribed in an ellipse (rx, ry), rotated by yaw. CCW order."""
    pts = []
    for i in range(n):
        a = 2 * math.pi * i / n + math.pi / n
        pts.append(_rotate(rx * math.cos(a), ry * math.sin(a), yaw, cx, cy))
    return pts


def rectangle(cx, cy, sx, sy, yaw):
    """Oriented rectangle, CCW order. sx along the object x axis, sy along y."""
    hx, hy = sx / 2, sy / 2
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    return [_rotate(x, y, yaw, cx, cy) for x, y in corners]


def disc_polygon(cx, cy, r, n=DISC_VERTICES):
    return regular_polygon(cx, cy, r, r, n, 0.0)


def convex_hull(points):
    """Andrew monotone chain. Returns CCW hull without duplicate endpoint."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def robot_wheels_body():
    """Wheel footprints (rectangles, body frame, CCW)."""
    wheels = []
    for deg in ROBOT_WHEEL_ANGLES_DEG:
        a = math.radians(deg)
        cx, cy = ROBOT_WHEEL_RADIUS_POS_M * math.cos(a), ROBOT_WHEEL_RADIUS_POS_M * math.sin(a)
        wheels.append(rectangle(cx, cy, ROBOT_WHEEL_WIDTH_M, ROBOT_WHEEL_LEN_M, a))
    return wheels


ROBOT_FOOTPRINT_BODY = convex_hull(ROBOT_CHASSIS_M + [p for w in robot_wheels_body() for p in w])


def robot_footprint(x, y, theta):
    """Robot collision hull (chassis + wheels) placed at a world pose."""
    return [_rotate(px, py, theta, x, y) for px, py in ROBOT_FOOTPRINT_BODY]


def polygons_overlap(a, b, gap=0.0):
    """Separating Axis Theorem for two convex polygons, with a minimum gap."""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            nx, ny = y1 - y2, x2 - x1           # edge normal
            length = math.hypot(nx, ny)
            if length == 0:
                continue
            nx, ny = nx / length, ny / length
            pa = [nx * x + ny * y for x, y in a]
            pb = [nx * x + ny * y for x, y in b]
            if max(pa) + gap <= min(pb) or max(pb) + gap <= min(pa):
                return False
    return True


def polygon_inside_room(poly, room_w, room_l, margin):
    return all(margin <= x <= room_w - margin and margin <= y <= room_l - margin
               for x, y in poly)


def polygon_center(poly):
    n = len(poly)
    return (sum(p[0] for p in poly) / n, sum(p[1] for p in poly) / n)


# ----------------------------------------------------------------- generator
class RoomGenerator:
    """Builds one random room from a seed. Call generate() once."""

    def __init__(self, seed):
        self.seed = seed
        self.rng = random.Random(seed)
        self.room_w = round(self.rng.uniform(*ROOM_SIZE_RANGE_M), 2)
        self.room_l = round(self.rng.uniform(*ROOM_SIZE_RANGE_M), 2)
        self.objects = []          # dicts: class, polygon, sx, sy, yaw, color
        self.protected = []        # objects that must keep >= 1 free side
        self.door = None
        self.door_zone = None      # cached free rectangle inside the door
        self.robot = None

    # -- public
    def generate(self):
        self._make_door()
        self._make_robot()
        bed = self._place_bed()
        table = self._place_table()
        self.protected = [o for o in (bed, table) if o is not None]
        if table is not None:
            self._place_chairs(table)
        self._place_fan()
        self._place_minis()
        return self._export()

    # -- walls / door / robot
    def _wall_segments(self):
        w, l = self.room_w, self.room_l
        return {
            "south": ((0.0, 0.0), (w, 0.0)),
            "east": ((w, 0.0), (w, l)),
            "north": ((w, l), (0.0, l)),
            "west": ((0.0, l), (0.0, 0.0)),
        }

    def _make_door(self):
        wall = self.rng.choice(WALL_ORDER)
        width = round(self.rng.uniform(*DOOR_WIDTH_RANGE_M), 2)
        along = self.room_w if wall in ("south", "north") else self.room_l
        start = round(self.rng.uniform(DOOR_END_MARGIN_M, along - width - DOOR_END_MARGIN_M), 2)
        if wall == "south":
            p1, p2, inward = (start, 0.0), (start + width, 0.0), (0, 1)
        elif wall == "north":
            p1, p2, inward = (start, self.room_l), (start + width, self.room_l), (0, -1)
        elif wall == "west":
            p1, p2, inward = (0.0, start), (0.0, start + width), (1, 0)
        else:
            p1, p2, inward = (self.room_w, start), (self.room_w, start + width), (-1, 0)
        self.door = {"wall": wall, "p1": p1, "p2": p2, "width": width, "inward": inward}
        self.door_zone = self._door_zone()

    def _door_mid(self):
        d = self.door
        return ((d["p1"][0] + d["p2"][0]) / 2, (d["p1"][1] + d["p2"][1]) / 2)

    def _make_robot(self):
        mx, my = self._door_mid()
        ix, iy = self.door["inward"]
        off = ROBOT_RADIUS_M + ROBOT_DOOR_OFFSET_M
        x, y = mx + ix * off, my + iy * off
        self.robot = {"x": x, "y": y, "theta": math.atan2(iy, ix),
                      "polygon": robot_footprint(x, y, math.atan2(iy, ix))}

    def _door_zone(self):
        """Rectangle inside the door that must stay free so the robot can enter."""
        d = self.door
        depth = 2 * ROBOT_RADIUS_M + DOOR_ZONE_EXTRA_M
        mx, my = self._door_mid()
        ix, iy = d["inward"]
        cx, cy = mx + ix * depth / 2, my + iy * depth / 2
        if ix == 0:
            return rectangle(cx, cy, d["width"], depth, 0.0)
        return rectangle(cx, cy, depth, d["width"], 0.0)

    # -- placement helpers
    def _blocked(self, poly, gap=OBJECT_GAP_M):
        if not polygon_inside_room(poly, self.room_w, self.room_l, WALL_MARGIN_M):
            return True
        if polygons_overlap(poly, self.robot["polygon"], gap):
            return True
        if polygons_overlap(poly, self.door_zone, 0.0):
            return True
        return any(polygons_overlap(poly, o["polygon"], gap) for o in self.objects)

    @staticmethod
    def _spec(cls):
        return FURNITURE_SPEC.get(cls) or MINI_SPEC[cls]

    def _sample_size(self, cls):
        spec = self._spec(cls)
        return self.rng.uniform(*spec[0]), self.rng.uniform(*spec[1])

    def _add(self, cls, poly, sx, sy, yaw):
        obj = {"class": cls, "polygon": poly, "sx": sx, "sy": sy, "yaw": yaw,
               "color": self.rng.choice(self._spec(cls)[3])}
        self.objects.append(obj)
        return obj

    @staticmethod
    def _footprint(cx, cy, sx, sy, yaw, n):
        if n == 4:
            return rectangle(cx, cy, sx, sy, yaw)
        return regular_polygon(cx, cy, sx / 2, sy / 2, n, yaw)

    def _try_place(self, cls, n, yaw_choices=None, region=None, gap=OBJECT_GAP_M):
        """Rejection sampling. region = (xmin, xmax, ymin, ymax) for the center."""
        sx, sy = self._sample_size(cls)
        xmin, xmax, ymin, ymax = region or (0.0, self.room_w, 0.0, self.room_l)
        for _ in range(MAX_PLACE_TRIES):
            yaw = self.rng.choice(yaw_choices) if yaw_choices else self.rng.uniform(0, 2 * math.pi)
            cx, cy = self.rng.uniform(xmin, xmax), self.rng.uniform(ymin, ymax)
            poly = self._footprint(cx, cy, sx, sy, yaw, n)
            if self._blocked(poly, gap):
                continue
            obj = self._add(cls, poly, sx, sy, yaw)
            if self._blocks_protected():
                self.objects.pop()
                continue
            return obj
        return None

    def _blocks_protected(self):
        """True when any protected object (bed/table) lost its last free side."""
        return any(not self._free_sides(o) for o in self.protected)

    # -- furniture
    def _place_bed(self):
        """Bed with its long side against a wall other than the door wall."""
        sx, sy = self._sample_size("bed")
        walls = [w for w in WALL_ORDER if w != self.door["wall"]]
        m = WALL_MARGIN_M + BED_WALL_EXTRA_M
        for _ in range(MAX_PLACE_TRIES):
            wall = self.rng.choice(walls)
            if wall in ("south", "north"):
                yaw = 0.0
                cy = m + sy / 2 if wall == "south" else self.room_l - m - sy / 2
                cx = self.rng.uniform(sx / 2 + m, self.room_w - sx / 2 - m)
            else:
                yaw = math.pi / 2
                cx = m + sy / 2 if wall == "west" else self.room_w - m - sy / 2
                cy = self.rng.uniform(sx / 2 + m, self.room_l - sx / 2 - m)
            poly = rectangle(cx, cy, sx, sy, yaw)
            if not self._blocked(poly):
                return self._add("bed", poly, sx, sy, yaw)
        return None

    def _place_table(self):
        m = TABLE_WALL_MARGIN_M
        region = (m, self.room_w - m, m, self.room_l - m)
        yaws = [0.0, math.pi / 2, self.rng.uniform(0, math.pi)]
        return self._try_place("table", 4, yaws, region, gap=2 * ROBOT_RADIUS_M)

    def _place_chairs(self, table):
        """Chairs around the table; keeps at least one table side free for the robot."""
        count = self.rng.randint(*CHAIR_COUNT_RANGE)
        tcx, tcy = polygon_center(table["polygon"])
        sides = list(range(4))
        self.rng.shuffle(sides)
        placed = 0
        for side in sides:
            if placed >= count:
                break
            sx, sy = self._sample_size("chair")
            angle = table["yaw"] + side * math.pi / 2
            half = table["sx"] / 2 if side % 2 == 0 else table["sy"] / 2
            dist = half + CHAIR_OFFSET_M + max(sx, sy) / 2
            cx, cy = tcx + dist * math.cos(angle), tcy + dist * math.sin(angle)
            poly = rectangle(cx, cy, sx, sy, angle)
            if self._blocked(poly, CHAIR_GAP_M):
                continue
            self._add("chair", poly, sx, sy, angle)
            if self._blocks_protected():
                self.objects.pop()          # this chair blocked the last free side
                continue
            placed += 1
        if placed == 0:
            self._try_place("chair", 4, gap=CHAIR_GAP_M)   # fallback: anywhere in the room

    def _place_fan(self):
        c, j = FAN_CORNER_OFFSET_M, FAN_CORNER_JITTER_M
        corners = [(c, c), (self.room_w - c, c), (c, self.room_l - c),
                   (self.room_w - c, self.room_l - c)]
        self.rng.shuffle(corners)
        for cx, cy in corners:
            if self._try_place("fan", 6, None, (cx - j, cx + j, cy - j, cy + j)):
                return
        self._try_place("fan", 6)

    def _place_minis(self):
        for _ in range(self.rng.randint(*MINI_COUNT_RANGE)):
            cls = self.rng.choice(list(MINI_SPEC))
            nmin, nmax = MINI_SPEC[cls][2]
            self._try_place(cls, self.rng.randint(nmin, nmax), gap=MINI_GAP_M)

    # -- derived fields
    def _free_sides(self, obj):
        """Sides (object frame) where a robot disc fits next to the object."""
        cx, cy = polygon_center(obj["polygon"])
        free = []
        for i, name in enumerate(SIDE_NAMES):
            angle = obj["yaw"] + i * math.pi / 2
            half = obj["sx"] / 2 if i % 2 == 0 else obj["sy"] / 2
            dist = half + ROBOT_RADIUS_M + FREE_SIDE_CLEARANCE_M
            foot = robot_footprint(cx + dist * math.cos(angle), cy + dist * math.sin(angle),
                                   angle + math.pi)          # facing the object
            if not polygon_inside_room(foot, self.room_w, self.room_l, 0.0):
                continue
            if any(o is not obj and polygons_overlap(foot, o["polygon"]) for o in self.objects):
                continue
            free.append(name)
        return free

    def _near(self, idx, centers):
        """Ids of the NEAR_COUNT closest landmarks (other objects or the door)."""
        cx, cy = centers[idx]
        landmarks = [(math.hypot(ox - cx, oy - cy), f"obj_{j + 1}")
                     for j, (ox, oy) in enumerate(centers) if j != idx]
        dx, dy = self._door_mid()
        landmarks.append((math.hypot(dx - cx, dy - cy), "door_1"))
        landmarks.sort()
        return [name for _, name in landmarks[:NEAR_COUNT]]

    # -- export
    def _export(self):
        def r3(v):
            return round(v, ROUND_DIGITS)

        def pt(p):
            return {"x": r3(p[0]), "y": r3(p[1])}

        walls = [{"id": f"wall_{i + 1}", "name": name, "p1": pt(a), "p2": pt(b)}
                 for i, (name, (a, b)) in enumerate(self._wall_segments().items())]
        centers = [polygon_center(o["polygon"]) for o in self.objects]
        exported = []
        for i, o in enumerate(self.objects):
            exported.append({
                "id": f"obj_{i + 1}",
                "class": o["class"],
                "confidence": r3(self.rng.uniform(*CONFIDENCE_RANGE)),
                "color": o["color"],
                "center": pt(centers[i]),
                "yaw": r3(o["yaw"] % (2 * math.pi)),
                "polygon": [pt(p) for p in o["polygon"]],
                "free_sides": self._free_sides(o),
                "near": self._near(i, centers),
            })
        d = self.door
        return {
            "version": SCHEMA_VERSION,
            "seed": self.seed,
            "units": "m",
            "frame": "world",
            "source": "simulator",
            "timestamp": time.time(),
            "room": {"width": self.room_w, "length": self.room_l},
            "walls": walls,
            "doors": [{"id": "door_1", "wall": d["wall"], "p1": pt(d["p1"]),
                       "p2": pt(d["p2"]), "width": d["width"]}],
            "robot": {
                "x": r3(self.robot["x"]), "y": r3(self.robot["y"]),
                "theta": r3(self.robot["theta"]),
                "radius": r3(ROBOT_RADIUS_M),
                "chassis": [pt(p) for p in ROBOT_CHASSIS_M],
                "wheels": [{"angle_deg": deg, "polygon": [pt(p) for p in w]}
                           for deg, w in zip(ROBOT_WHEEL_ANGLES_DEG, robot_wheels_body())],
                "footprint": [pt(p) for p in ROBOT_FOOTPRINT_BODY],
                "frame_note": "chassis/wheels/footprint are in the robot body frame (+x forward)",
            },
            "objects": exported,
        }


def generate(seed=None):
    """Generate one scene dict. Random seed when None."""
    if seed is None:
        seed = random.randrange(1, MAX_RANDOM_SEED)
    return RoomGenerator(seed).generate()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Generate a random 2D room scene JSON.")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--out", type=str, default=None, help="write JSON to this file")
    args = ap.parse_args(argv)
    scene = generate(args.seed)
    text = json.dumps(scene, indent=2)
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"seed {scene['seed']} -> {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
