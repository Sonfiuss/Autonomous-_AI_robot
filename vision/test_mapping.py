"""Mapping tests on synthetic depth - no camera, no model. Run: python test_mapping.py

A pinhole camera 0.24 m high, pitched 5 deg down (a non-zero pitch on purpose, to exercise the tilt
math - the robot's own camera is level), with the Astra's FOV and range limits,
renders a box world: floor, four walls, a "chair" and an unlabelled box. The whole capture path
(map_builder.integrate_capture -> OccupancyMap -> scene_export) runs on those frames and the map is
checked against where things really are. Accuracy targets are loose on purpose (minimal strategy).
"""
import collections
import math
import sys

import numpy as np

from detector import Detection
from floor_geometry import CameraMount, backproject, camera_to_body, fit_floor, pixel_heights
from map_builder import integrate_capture
from object_distance import intrinsics_from_fov, object_mask
from occupancy_map import PIXEL_FLOOR, PIXEL_NONE, PIXEL_OBSTACLE, OccupancyMap, classify_heights
from scene_export import extract_regions, to_scene

H, W = 480, 640
INTR = intrinsics_from_fov(W, H, math.radians(58.59), math.radians(45.64))
MOUNT = CameraMount(height_m=0.24, pitch_deg=5.0, forward_m=0.0)
MIN_RANGE_M, MAX_RANGE_M = 0.6, 8.0

Box = collections.namedtuple("Box", "name x0 y0 x1 y1 height")   # axis-aligned, standing on the floor
WALLS = [Box("wall", -1.6, -1.6, 2.6, -1.5, 1.0), Box("wall", -1.6, 1.5, 2.6, 1.6, 1.0),
         Box("wall", -1.6, -1.5, -1.5, 1.5, 1.0), Box("wall", 2.5, -1.5, 2.6, 1.5, 1.0)]
CHAIR = Box("chair", 1.0, 0.4, 1.4, 0.8, 0.5)
CRATE = Box("crate", -0.9, -1.0, -0.6, -0.7, 0.3)   # not a detector class -> must come out "unknown"
WORLD = WALLS + [CHAIR, CRATE]
WALL_LINES = [((-1.5, -1.5), (2.5, -1.5)), ((-1.5, 1.5), (2.5, 1.5)), ((-1.5, -1.5), (-1.5, 1.5)), ((2.5, -1.5), (2.5, 1.5))]
CHAIR_ID = 56                     # COCO "chair"


def render(pose, boxes=WORLD, mount=MOUNT):
    """uint16 depth (mm) + per-pixel hit index (-1 floor/none) seen from pose = (x, y, theta)."""
    v, u = np.mgrid[0:H, 0:W].astype(np.float64)
    dx, dy = (u - INTR.cx) / INTR.fx, (v - INTR.cy) / INTR.fy   # camera ray with z = 1 -> t is the depth
    p = math.radians(mount.pitch_deg)
    fwd, left, up = math.cos(p) + dy * math.sin(p), -dx, -math.sin(p) - dy * math.cos(p)
    x, y, th = pose
    wx, wy, wz = math.cos(th) * fwd - math.sin(th) * left, math.sin(th) * fwd + math.cos(th) * left, up
    ox, oy, oz = x + mount.forward_m * math.cos(th), y + mount.forward_m * math.sin(th), mount.height_m
    t = np.full((H, W), np.inf)
    hit = np.full((H, W), -1)
    with np.errstate(divide="ignore", invalid="ignore"):
        floor_t = -oz / wz
        t = np.where((wz < 0) & (floor_t > 0), floor_t, t)
        for k, b in enumerate(boxes):
            near, far = np.full((H, W), -np.inf), np.full((H, W), np.inf)
            for o, d, lo, hi in ((ox, wx, b.x0, b.x1), (oy, wy, b.y0, b.y1), (oz, wz, 0.0, b.height)):
                t1, t2 = (lo - o) / d, (hi - o) / d
                near, far = np.maximum(near, np.minimum(t1, t2)), np.minimum(far, np.maximum(t1, t2))
            closer = (near <= far) & (near > 0) & (near < t)
            t = np.where(closer, near, t)
            hit = np.where(closer, k, hit)
    in_range = (t >= MIN_RANGE_M) & (t <= MAX_RANGE_M)
    return np.where(in_range, np.round(t * 1000), 0).astype(np.uint16), np.where(in_range, hit, -1)


def _detections(hit, boxes=WORLD):
    """A perfect detector that knows chairs only: bbox around the chair's visible pixels."""
    rows, cols = np.nonzero(hit == boxes.index(CHAIR))
    if rows.size < 200:
        return []
    return [Detection(CHAIR_ID, "chair", 0.9, (float(cols.min()), float(rows.min()), float(cols.max() + 1), float(rows.max() + 1)))]


def _dist_to_segment(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
    return math.dist(p, (a[0] + t * ax, a[1] + t * ay))


def check_geometry():
    errors = []
    depth, _ = render((0.0, 0.0, 0.0), boxes=[])   # floor only
    heights = camera_to_body(backproject(depth, INTR, 2), MOUNT)[:, 2]
    if heights.size == 0 or np.abs(heights).max() > 0.01:
        errors.append(f"floor seen through the true mount must sit at height 0, max |h| = {np.abs(heights).max():.3f} m")
    for assumed, label in ((MOUNT, "true mount"), (MOUNT._replace(pitch_deg=-5.0), "pitch sign flipped"),
                           (MOUNT._replace(height_m=0.30), "height 6 cm off")):
        fit = fit_floor(camera_to_body(backproject(depth, INTR, 2), assumed), assumed)
        if fit is None or abs(fit.height_m - 0.24) > 0.01 or abs(fit.pitch_deg - 5.0) > 0.5:
            errors.append(f"fit_floor with {label}: expected 0.24 m / 5.0 deg, got {fit}")
    # Walls and obstacles in view: their feet fall in the floor band and once dragged a plain
    # least-squares fit to 0.29-0.43 m / 9-16 deg.
    for heading in range(0, 360, 45):
        depth, _ = render((0.0, 0.0, math.radians(heading)))
        fit = fit_floor(camera_to_body(backproject(depth, INTR, 2), MOUNT), MOUNT)
        if fit is not None and (abs(fit.height_m - 0.24) > 0.01 or abs(fit.pitch_deg - 5.0) > 0.5):
            errors.append(f"fit_floor in the room, heading {heading} deg: expected 0.24 m / 5.0 deg, got {fit}")
    return errors


def check_pixel_classes():
    """The per-pixel view (NNN_floor.png, key f) must agree with what integrate() puts in the map."""
    errors = []
    depth, hit = render((0.0, 0.0, math.atan2(0.6, 1.2)))   # facing the chair
    labels = classify_heights(pixel_heights(depth, INTR, MOUNT))
    seen = (depth > 0) & (depth <= 3500)   # floor_geometry.MAX_RANGE_M
    floor_px, chair_px = seen & (hit == -1), seen & (hit == WORLD.index(CHAIR))
    if (labels[floor_px] != PIXEL_FLOOR).mean() > 0.001:
        errors.append(f"{(labels[floor_px] != PIXEL_FLOOR).mean():.1%} of floor pixels not classed floor")
    chair_heights = pixel_heights(depth, INTR, MOUNT)[chair_px]
    expected = (chair_heights > 0.08) & (chair_heights <= 0.6)
    if (labels[chair_px][expected] != PIXEL_OBSTACLE).any():
        errors.append("chair pixels 8 cm..0.6 m high must be classed obstacle")
    if (labels[~seen] != PIXEL_NONE).any():
        errors.append("pixels without depth must stay unclassified")
    return errors


def check_object_mask():
    errors = []
    depth = np.full((H, W), 3000, np.uint16)
    depth[200:280, 300:340] = 1200
    mask = object_mask(depth, (280.0, 180.0, 360.0, 300.0), 1.2)
    if mask.sum() != 80 * 40 or not mask[200:280, 300:340].all():
        errors.append(f"object_mask should keep exactly the 1.2 m pixels (3200), got {mask.sum()}")
    return errors


def _build_map():
    occ_map = OccupancyMap()
    poses = [(0.0, 0.0, math.radians(a)) for a in range(0, 360, 45)]
    poses += [(0.8, -0.6, math.radians(a)) for a in range(0, 360, 45)]
    for pose in poses:
        depth, hit = render(pose)
        integrate_capture(occ_map, [depth] * 3, _detections(hit), INTR, MOUNT, pose)
    return occ_map, poses


def check_map(occ_map, poses):
    errors = []
    regions = extract_regions(occ_map)
    walls = [r for r in regions if r.kind == "wall"]
    objects = [r for r in regions if r.kind == "object"]
    print(f"    walls: {[(tuple(round(v, 2) for v in r.polygon[0]), tuple(round(v, 2) for v in r.polygon[1])) for r in walls]}")
    print(f"    objects: {[(r.label, tuple(round(v, 2) for v in r.center), round(r.confidence, 2), int(r.mask.sum())) for r in objects]}")

    if len(walls) != 4:
        errors.append(f"expected 4 walls, got {len(walls)}")
    for w in walls:
        mid = w.center
        d = min(_dist_to_segment(mid, a, b) for a, b in WALL_LINES)
        if d > 0.15:
            errors.append(f"wall {w.polygon} is {d:.2f} m off every real wall")

    def near(label, center, tol):
        return [r for r in objects if r.label == label and math.dist(r.center, center) < tol]
    if not near("chair", (1.2, 0.6), 0.3):
        errors.append("no object labelled chair within 0.3 m of the chair")
    if not near("unknown", (-0.75, -0.85), 0.3):
        errors.append("no unknown object within 0.3 m of the crate")
    expected = {id(r) for r in near("chair", (1.2, 0.6), 0.3) + near("unknown", (-0.75, -0.85), 0.3)}
    stray = [r for r in objects if id(r) not in expected and r.mask.sum() * occ_map.res ** 2 > 0.05]
    if stray:
        errors.append(f"stray objects larger than 0.05 m2: {[(r.label, r.center) for r in stray]}")

    occupied, free = occ_map.occupied(), occ_map.free()
    for p in ((1.5, -0.5), (-1.0, 0.5), (0.0, 0.0)):
        ix, iy = occ_map.world_to_cell(*p)
        if not free[iy, ix]:
            errors.append(f"floor at {p} should be free (log-odds {occ_map.log_odds[iy, ix]:.2f})")
    ix, iy = occ_map.world_to_cell(1.02, 0.6)
    if not occupied[iy, ix]:
        errors.append("chair face at (1.02, 0.6) should be occupied")

    scene = to_scene(occ_map, regions, poses[-1])
    room = scene["room"]
    if not (3.8 < room["width"] < 4.4 and 2.8 < room["length"] < 3.4):
        errors.append(f"room should be ~4.0 x 3.0 m, got {room}")
    for o in scene["objects"]:
        poly = [(p["x"], p["y"]) for p in o["polygon"]]
        area = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1])) / 2
        if not 3 <= len(poly) <= 6 or area <= 0:
            errors.append(f"{o['id']}: polygon must be 3..6 vertices CCW, got {len(poly)} vertices area {area:.3f}")
        if any(not (-0.01 <= x <= room["width"] + 0.01 and -0.01 <= y <= room["length"] + 0.01) for x, y in poly):
            errors.append(f"{o['id']} polygon leaves the room box")
        if len(o["near"]) != min(2, len(scene["objects"]) - 1):
            errors.append(f"{o['id']} near = {o['near']}")
    chair = next((o for o in scene["objects"] if o["class"] == "chair"), None)
    if chair is not None and not chair["free_sides"]:
        errors.append("the chair stands in open floor, it should have at least one free side")
    print(f"    room {room}, robot {scene['robot']}")
    print(f"    scene objects: {[(o['id'], o['class'], o['confidence'], o['free_sides'], o['near']) for o in scene['objects']]}")
    return errors


def main():
    failed = 0
    occ_map, poses = _build_map()
    checks = (("check_geometry", check_geometry), ("check_pixel_classes", check_pixel_classes),
              ("check_object_mask", check_object_mask),
              ("check_map", lambda: check_map(occ_map, poses)))
    for name, check in checks:
        errors = check()
        print(f"{'PASS' if not errors else 'FAIL'} {name}")
        for e in errors:
            print("   ", e)
        failed += bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
