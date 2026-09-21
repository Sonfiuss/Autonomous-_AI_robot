"""L3 candidate geometry: every reachable "gathering point" of a scene.

For each object and each of its free sides the robot hull is placed next to that side at up to
three spots (corner_a, center, corner_b), facing the object. Each spot carries a code-generated
description (compass direction, nearest landmark) so the LLM can match words like
"top right corner of the bed" without doing any geometry itself.

CLI:  python candidates.py --seed 7            print the candidate list for a generated scene
      python candidates.py --file x.json       ... for a saved scene
      python candidates.py --seed 7 --summary  print the LLM-facing text summary
"""
import argparse
import json
import math
import sys

import config

sys.path.insert(0, config.ROOM_DIR)
from room_generator import (  # noqa: E402  (path inserted above)
    SIDE_NAMES, generate, polygon_inside_room, polygons_overlap, robot_footprint,
)

COMPASS_8 = ("east", "north-east", "north", "north-west", "west", "south-west", "south", "south-east")
ZERO_EPS = 1e-9                  # direction shorter than this has no compass word
DOOR_INWARD = {"south": (0.0, 1.0), "north": (0.0, -1.0), "west": (1.0, 0.0), "east": (-1.0, 0.0)}


def compass_word(dx, dy):
    """8-way compass word for a world-frame direction (+y = north = screen top)."""
    if abs(dx) < ZERO_EPS and abs(dy) < ZERO_EPS:
        return "center"
    idx = int(round(math.atan2(dy, dx) / (math.pi / 4))) % 8
    return COMPASS_8[idx]


def side_normal(obj, side):
    """Outward unit normal of an object side in the world frame."""
    angle = obj["yaw"] + SIDE_NAMES.index(side) * math.pi / 2
    return math.cos(angle), math.sin(angle)


def _pts(poly):
    return [(p["x"], p["y"]) for p in poly]


def _round(v):
    return round(v, config.ROUND_DIGITS)


class CandidateBuilder:
    """Builds the candidate spot list for one scene JSON (v2.0)."""

    def __init__(self, scene):
        self.scene = scene
        self.room_w = scene["room"]["width"]
        self.room_l = scene["room"]["length"]
        self.radius = scene["robot"]["radius"]
        self.polys = {o["id"]: _pts(o["polygon"]) for o in scene["objects"]}
        self.by_id = {o["id"]: o for o in scene["objects"]}
        self.landmark_pos = {o["id"]: (o["center"]["x"], o["center"]["y"]) for o in scene["objects"]}
        for d in scene["doors"]:
            self.landmark_pos[d["id"]] = ((d["p1"]["x"] + d["p2"]["x"]) / 2,
                                          (d["p1"]["y"] + d["p2"]["y"]) / 2)

    # -- public
    def build(self):
        spots = []
        for obj in self.scene["objects"]:
            for side in obj["free_sides"]:
                spots.extend(self._side_spots(obj, side))
        for door in self.scene["doors"]:
            spot = self._door_spot(door)
            if spot is not None:
                spots.append(spot)
        return spots

    # -- geometry
    def _side_spots(self, obj, side):
        cx, cy = obj["center"]["x"], obj["center"]["y"]
        ux, uy = side_normal(obj, side)                       # outward normal of the side
        tx, ty = -uy, ux                                      # tangent along the side (CCW)
        poly = self.polys[obj["id"]]
        half = max((px - cx) * ux + (py - cy) * uy for px, py in poly)
        t_vals = [(px - cx) * tx + (py - cy) * ty for px, py in poly]
        t_min, t_max = min(t_vals), max(t_vals)
        dist = half + self.radius + config.SPOT_CLEARANCE_M
        theta = math.atan2(-uy, -ux)                          # face the object
        positions = [("center", (t_min + t_max) / 2)]
        if t_max - t_min >= config.CORNER_MIN_SIDE_M:
            positions = [("corner_a", t_max), ("center", (t_min + t_max) / 2), ("corner_b", t_min)]
        out = []
        for name, t in positions:
            x = cx + dist * ux + t * tx
            y = cy + dist * uy + t * ty
            if not self._free(x, y, theta, exclude=obj["id"]):
                continue
            out.append(self._spot(obj["id"], side, name, x, y, theta, (cx, cy), (ux, uy)))
        return out

    def _door_spot(self, door):
        mid = self.landmark_pos[door["id"]]
        ux, uy = DOOR_INWARD[door["wall"]]
        dist = self.radius + config.DOOR_INSIDE_OFFSET_M
        x, y = mid[0] + dist * ux, mid[1] + dist * uy
        theta = math.atan2(-uy, -ux)                          # face the door
        if not self._free(x, y, theta, exclude=None):
            return None
        return self._spot(door["id"], "inside", "center", x, y, theta, mid, (ux, uy), cls="door")

    def _free(self, x, y, theta, exclude):
        foot = robot_footprint(x, y, theta)
        if not polygon_inside_room(foot, self.room_w, self.room_l, 0.0):
            return False
        return not any(oid != exclude and polygons_overlap(foot, poly)
                       for oid, poly in self.polys.items())

    # -- description
    def _spot(self, target_id, side, name, x, y, theta, center, normal, cls=None):
        cls = cls or self.by_id[target_id]["class"]
        side_compass = compass_word(*normal)
        if cls == "door":
            where = "just inside the door"
        elif name == "center":
            where = f"middle of the {side_compass} side"
        else:
            where = f"{compass_word(x - center[0], y - center[1])} corner"
        near_id, near_txt = self._nearest_landmark(target_id, x, y)
        desc = where if cls == "door" else f"{where} of the {cls}"
        if near_txt:
            desc += f", nearest landmark {near_txt}"
        return {
            "id": f"{target_id}:{side}:{name}",
            "target_id": target_id, "class": cls, "side": side, "spot": name,
            "side_compass": side_compass,
            "x": _round(x), "y": _round(y), "theta": _round(theta),
            "near": near_id, "description": desc,
        }

    def _nearest_landmark(self, target_id, x, y):
        best = None
        for lid, (lx, ly) in self.landmark_pos.items():
            if lid == target_id:
                continue
            d = math.hypot(lx - x, ly - y)
            if best is None or d < best[0]:
                best = (d, lid)
        if best is None:
            return None, ""
        lid = best[1]
        label = self.by_id[lid]["class"] if lid in self.by_id else "door"
        return lid, f"{lid} ({label}, {best[0]:.1f} m away)"


def build_candidates(scene):
    return CandidateBuilder(scene).build()


def scene_summary(scene, spots):
    """Compact text the LLM reads: one line per object, one line per spot."""
    lines = [f"Room {scene['room']['width']} x {scene['room']['length']} m. "
             "World frame: +x = east = screen right, +y = north = screen top."]
    lines.append("OBJECTS (id | class | color | free sides as side=compass | near):")
    for o in scene["objects"]:
        sides = ", ".join(f"{sd}={compass_word(*side_normal(o, sd))}" for sd in o["free_sides"])
        near = ", ".join(o["near"])
        lines.append(f"- {o['id']} | {o['class']} | {o['color']} | {sides or 'none (unreachable)'} | near {near}")
    for d in scene["doors"]:
        lines.append(f"- {d['id']} | door | on the {d['wall']} wall")
    lines.append("SPOTS (candidate id | description):")
    for s in spots:
        lines.append(f"- {s['id']} | {s['description']}")
    return "\n".join(lines)


def _main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", type=int)
    ap.add_argument("--file")
    ap.add_argument("--summary", action="store_true", help="print the LLM-facing summary instead of JSON")
    args = ap.parse_args()
    if args.file:
        with open(args.file, encoding="utf-8") as f:
            scene = json.load(f)
    else:
        scene = generate(args.seed)
    spots = build_candidates(scene)
    print(scene_summary(scene, spots) if args.summary else json.dumps(spots, indent=2))


if __name__ == "__main__":
    _main()
