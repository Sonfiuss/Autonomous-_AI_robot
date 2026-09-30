"""Offline tests of object colors and the scene's robot outline - no camera, no robot. Run:
python3 vision/test_object_color.py

object_color: naming pixels, votes landing on the cell under the object's foot (and none from feet out of
range), a detector box's foot climbing to the box top past the floor-level band, the hue preference and the
unknown cases. scene_export: the real robot's outline (the simulator's turned to wheels 0/120/240, W1 in
front) and the color fields of the objects it exports.
"""
import math
import sys

import cv2
import numpy as np

import object_color
from floor_geometry import CameraMount
from floor_segment import floor_hit
from object_color import COLOR_NAMES, UNKNOWN_COLOR, ColorVotes, ObjectColor, color_names
from object_distance import Intrinsics
from occupancy_map import LOG_MAX, LOG_MIN, PIXEL_OBSTACLE, OccupancyMap, body_to_world
from scene_export import extract_regions, robot_shape, to_scene

H, W = 480, 640
INTR = Intrinsics(fx=570.0, fy=570.0, cx=319.5, cy=239.5)
MOUNT = CameraMount(height_m=0.24, pitch_deg=-0.92, forward_m=0.185, left_m=0.062)
POSE = (0.0, 0.0, 0.0)
MAX_RANGE_M = 3.5
COL0, COL1 = 200, 440            # the synthetic object's columns
FOOT_ROW = 299                   # its lowest obstacle row: ~2.7 m ahead with this mount
BAND_TOP = 260                   # the obstacle labels stop here, as at the floor trapezoid's top
BOX_TOP = 100
FAR_M = 1.0                      # a max range the synthetic foot lies beyond
SHIFT_M = 1.0                    # a mask this far aside holds none of the object's cells

# BGR of a named color, and the name it must get.
SAMPLES = (((0, 0, 230), "red"), ((0, 140, 255), "orange"), ((0, 220, 230), "yellow"), ((40, 200, 40), "green"),
           ((220, 90, 20), "blue"), ((160, 40, 130), "purple"), ((200, 170, 255), "pink"), ((30, 70, 120), "brown"),
           ((15, 15, 15), "black"), ((240, 240, 240), "white"), ((128, 128, 128), "gray"))
GREEN, DARK, GRAY = (40, 200, 40), (20, 20, 20), (128, 128, 128)
RED_BGR, WHITE_BGR = (0, 0, 230), (240, 240, 240)
FULL_SHARE = 0.99                # one color over the whole object
ONE_FOOT_COL = COL0 + 10
BLOCK_CELLS = 4                  # the synthetic scene's object: a square of cells
WHEEL_TOL_M = 1e-3
FRONT_WIDTH_M, REAR_WIDTH_M = 0.10, 0.244     # the simulator's hexagon, narrow end now in front
PAINTED = ObjectColor("red", "#e60000", 0.9, 400)
UNNAMED = ObjectColor(UNKNOWN_COLOR, None, 0.3, 400)     # votes, but no name with a clear share


def _labels(top=BAND_TOP):
    labels = np.zeros((H, W), np.uint8)
    labels[top:FOOT_ROW + 1, COL0:COL1] = PIXEL_OBSTACLE
    return labels


def _contacts():
    return np.array([(u, FOOT_ROW) for u in range(COL0, COL1)], np.int64)


def _image(columns=None, rows=None):
    """Background mid gray; the object's block painted per column share or per row band."""
    image = np.full((H, W, 3), GRAY, np.uint8)
    for (c0, c1), color in (columns or {}).items():
        image[:, c0:c1] = color
    for (r0, r1), color in (rows or {}).items():
        image[r0:r1, COL0:COL1] = color
    return image


def _foot_mask(occ_map):
    """The cells under the synthetic object's feet."""
    forward, left, _ = floor_hit(_contacts()[:, 0], _contacts()[:, 1], INTR, MOUNT)
    xy = body_to_world(np.column_stack((forward, left)), POSE)
    mask = np.zeros((occ_map.n, occ_map.n), bool)
    ix = np.floor((xy[:, 0] - occ_map.origin[0]) / occ_map.res).astype(int)
    iy = np.floor((xy[:, 1] - occ_map.origin[1]) / occ_map.res).astype(int)
    mask[iy, ix] = True
    return mask


def _vote(image, labels=None, boxes=(), max_range_m=MAX_RANGE_M, contacts=None):
    occ_map = OccupancyMap()
    votes = ColorVotes(occ_map)
    added = votes.add_view(_labels() if labels is None else labels, _contacts() if contacts is None else contacts,
                           image, POSE, INTR, MOUNT, max_range_m, boxes)
    return votes, added, _foot_mask(occ_map)


def check_color_names():
    """Each sample BGR gets its basic color term."""
    bgr = np.array([[s[0] for s in SAMPLES]], np.uint8)
    names = color_names(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV).reshape(-1, 3))
    return [f"{color} named {COLOR_NAMES[got]}" for (color, want), got in zip(SAMPLES, names)
            if COLOR_NAMES[got] != want]


def check_votes_on_the_foot():
    """Every foot of a tall object votes its face, capped at FACE_ROWS, onto its own cell; none lands
    aside; a foot beyond the max range votes nothing."""
    errors = []
    votes, added, mask = _vote(_image(columns={(COL0, COL1): RED_BGR}), labels=_labels(top=0))
    feet = len(range(COL0, COL1, object_color.PIXEL_STRIDE))
    rows = (object_color.FACE_ROWS - object_color.FOOT_SKIP_ROWS) // object_color.PIXEL_STRIDE + 1
    if added != feet * rows:
        errors.append(f"{added} pixels voted, want {feet} feet x {rows} rows")
    got = votes.of(mask)
    if got.name != "red" or got.pixels != added or got.share < FULL_SHARE:
        errors.append(f"object on its cells: {got}")
    shift = int(round(SHIFT_M / OccupancyMap().res))
    aside = votes.of(np.roll(mask, shift, axis=1))
    if aside.pixels or aside.name != UNKNOWN_COLOR:
        errors.append(f"cells {SHIFT_M} m aside got {aside}")
    _, far, _ = _vote(_image(columns={(COL0, COL1): RED_BGR}), max_range_m=FAR_M)
    if far:
        errors.append(f"a foot beyond {FAR_M} m voted {far} pixels")
    return errors


def check_box_climbs():
    """A chair: green seat above a dark band of legs and shadow, labels only on the band. Without its
    detector box the band decides (black); with it, the middle feet climb to the box top (green) and the
    box's sides stay at FACE_ROWS."""
    errors = []
    image = _image(rows={(BOX_TOP, BAND_TOP): GREEN, (BAND_TOP, FOOT_ROW + 1): DARK})
    plain, _, mask = _vote(image)
    if plain.of(mask).name != "black":
        errors.append(f"band only: {plain.of(mask)}")
    boxed, added, mask = _vote(image, boxes=[(COL0, BOX_TOP, COL1, FOOT_ROW + 1)])
    if boxed.of(mask).name != "green":
        errors.append(f"with the box: {boxed.of(mask)}")
    margin = 0.5 * (1.0 - object_color.BOX_CORE_SHARE) * (COL1 - COL0)
    feet = np.arange(COL0, COL1, object_color.PIXEL_STRIDE)
    core = int(((feet >= COL0 + margin) & (feet < COL1 - margin)).sum())
    climb = (FOOT_ROW - object_color.FOOT_SKIP_ROWS - BOX_TOP) // object_color.PIXEL_STRIDE + 1
    side = (object_color.FACE_ROWS - object_color.FOOT_SKIP_ROWS) // object_color.PIXEL_STRIDE + 1
    if added != core * climb + (len(feet) - core) * side:
        errors.append(f"{added} pixels with the box, want {core} core feet x {climb} + {len(feet) - core} x {side}")
    return errors


def check_shares():
    """A hue on MIN_HUE_SHARE of the pixels wins over a gray majority, one below it does not; no name with
    a clear share, or too few pixels, is unknown."""
    errors = []
    span = COL1 - COL0

    def split(*parts):
        """Columns of the object in consecutive shares of span, painted with the given colors."""
        columns, at = {}, COL0
        for share, color in parts:
            end = at + int(round(share * span))
            columns[(at, end)] = color
            at = end
        return _image(columns=columns)

    cases = ((split((0.3, GREEN), (0.7, DARK)), "green"), (split((0.2, GREEN), (0.8, DARK)), "black"),
             (split((0.34, DARK), (0.33, WHITE_BGR), (0.33, GRAY)), UNKNOWN_COLOR))
    for image, want in cases:
        votes, _, mask = _vote(image)
        if votes.of(mask).name != want:
            errors.append(f"want {want}, got {votes.of(mask)}")
    one_foot = np.array([(ONE_FOOT_COL, FOOT_ROW)], np.int64)
    votes, added, mask = _vote(_image(columns={(COL0, COL1): GREEN}), contacts=one_foot)
    if added >= object_color.MIN_PIXELS or votes.of(mask).name != UNKNOWN_COLOR:
        errors.append(f"one foot ({added} pixels): {votes.of(mask)}")
    if ColorVotes(OccupancyMap()).of(mask) != ObjectColor(UNKNOWN_COLOR, None, 0.0, 0):
        errors.append("no votes at all is not a plain unknown")
    return errors


def check_robot_shape():
    """Wheels 0/120/240 with W1 straight ahead at L; the hexagon's narrow end in front, wide end behind."""
    errors = []
    shape = robot_shape()
    angles = [w["angle_deg"] for w in shape["wheels"]]
    if angles != [0, 120, 240]:
        errors.append(f"wheel angles {angles}")
    w1 = shape["wheels"][0]["polygon"]
    cx, cy = sum(p["x"] for p in w1) / len(w1), sum(p["y"] for p in w1) / len(w1)
    if math.hypot(cx - 0.21, cy) > WHEEL_TOL_M:
        errors.append(f"W1 centred at ({cx:.3f}, {cy:.3f}), want (0.21, 0)")
    xs = [p["x"] for p in shape["chassis"]]

    def width_at(x):
        return max(p["y"] for p in shape["chassis"] if abs(p["x"] - x) < WHEEL_TOL_M) - \
            min(p["y"] for p in shape["chassis"] if abs(p["x"] - x) < WHEEL_TOL_M)

    if abs(width_at(max(xs)) - FRONT_WIDTH_M) > WHEEL_TOL_M or abs(width_at(min(xs)) - REAR_WIDTH_M) > WHEEL_TOL_M:
        errors.append(f"front width {width_at(max(xs)):.3f}, rear {width_at(min(xs)):.3f}")
    footprint = np.array([(p["x"], p["y"]) for p in shape["footprint"]], np.float32)
    for wheel in shape["wheels"]:
        for p in wheel["polygon"]:
            if cv2.pointPolygonTest(footprint, (p["x"], p["y"]), True) < -WHEEL_TOL_M:
                errors.append(f"wheel {wheel['angle_deg']} corner {p} outside the footprint")
    return errors


class FixedVotes:
    """A ColorVotes stand-in giving every object the same color."""

    def __init__(self, color):
        self.color = color

    def of(self, mask):
        return self.color


def check_scene_fields():
    """to_scene: an object's color, color_rgb and color_share from the votes (unknown / None / None without,
    and for a vote that named nothing), the robot with its chassis, wheels and footprint."""
    occ_map = OccupancyMap()
    occ_map.log_odds[:] = LOG_MIN
    iy, ix = occ_map.world_to_cell(1.0, 0.0)[::-1]
    occ_map.log_odds[iy:iy + BLOCK_CELLS, ix:ix + BLOCK_CELLS] = LOG_MAX
    regions = extract_regions(occ_map)
    errors = []
    painted = to_scene(occ_map, regions, POSE, FixedVotes(PAINTED))
    plain = to_scene(occ_map, regions, POSE)
    unnamed = to_scene(occ_map, regions, POSE, FixedVotes(UNNAMED))

    def fields(scene):
        return [(o["color"], o["color_rgb"], o["color_share"]) for o in scene["objects"]]

    for scene, want in ((painted, (PAINTED.name, PAINTED.rgb, PAINTED.share)), (plain, (UNKNOWN_COLOR, None, None)),
                        (unnamed, (UNKNOWN_COLOR, None, None))):
        if fields(scene) != [want]:
            errors.append(f"object color fields {fields(scene)}, want {want}")
    if not all(key in plain["robot"] for key in ("chassis", "wheels", "footprint")):
        errors.append(f"robot fields {sorted(plain['robot'])}")
    return errors


def main():
    failed = 0
    for check in (check_color_names, check_votes_on_the_foot, check_box_climbs, check_shares, check_robot_shape,
                  check_scene_fields):
        errors = check()
        print(f"{'PASS' if not errors else 'FAIL'} {check.__name__}")
        for e in errors:
            print("   ", e)
        failed += bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
