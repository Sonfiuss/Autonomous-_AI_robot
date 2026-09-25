"""Mapping tests on synthetic depth - no camera, no model. Run: python test_mapping.py

A pinhole camera 0.24 m high, pitched 5 deg down (a non-zero pitch on purpose, to exercise the tilt
math - the robot's own camera is level), with the Astra's FOV and range limits,
renders a box world: floor, four walls, a "chair" and an unlabelled box. The whole capture path
(map_builder.integrate_capture -> OccupancyMap -> scene_export) runs on those frames and the map is
checked against where things really are. Accuracy targets are loose on purpose (minimal strategy).

Depth Anything is stood in for by the true inverse depth under a random scale and shift per frame -
the model's output is affine-invariant, so the floor code must not rely on either. The Astra depth of
the floor is blanked, as on the real glossy tiles: free floor has to come from the stand-in.
"""
import collections
import math
import sys

import numpy as np

from detector import Detection
from floor_geometry import CameraMount, backproject, camera_to_body, fit_floor, pixel_heights
from floor_segment import analyze_floor, floor_hit, floor_xy
from map_builder import integrate_capture
from object_distance import intrinsics_from_fov, object_mask
from occupancy_map import PIXEL_DROP, PIXEL_FREE, PIXEL_NONE, PIXEL_OBSTACLE, OccupancyMap, classify_heights
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
    t, hit = _trace(pose, boxes, mount)
    in_range = (t >= MIN_RANGE_M) & (t <= MAX_RANGE_M)
    return np.where(in_range, np.round(t * 1000), 0).astype(np.uint16), np.where(in_range, hit, -1)


def render_disparity(pose, rng, boxes=WORLD, mount=MOUNT):
    """Depth Anything stand-in: scale / z + shift, scale and shift random per call. On the real floor the
    model's shift came out at about -0.13 * scale; both signs are drawn."""
    t, _ = _trace(pose, boxes, mount)
    scale = rng.uniform(2.0, 8.0)
    return (scale / t + rng.uniform(-0.13, 0.2) * scale).astype(np.float32)   # t = inf above the walls -> shift


def glossy(depth, hit):
    """The Astra depth with the floor blanked, as the real glossy tiles return it."""
    return np.where(hit == -1, 0, depth).astype(np.uint16)


def _trace(pose, boxes, mount):
    """Depth along the optical axis (m, inf where nothing is hit) + hit index (-1 floor/none)."""
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
    return t, hit


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
    """The Astra's per-pixel classes (NNN_floor.png, key f) must agree with what integrate() puts in
    the map - obstacles only, never floor."""
    errors = []
    depth, hit = render((0.0, 0.0, math.atan2(0.6, 1.2)))   # facing the chair
    labels = classify_heights(pixel_heights(depth, INTR, MOUNT))
    seen = (depth > 0) & (depth <= 3500)   # floor_geometry.MAX_RANGE_M
    floor_px, chair_px = seen & (hit == -1), seen & (hit == WORLD.index(CHAIR))
    if (labels[floor_px] != PIXEL_NONE).any():
        errors.append(f"{(labels[floor_px] != PIXEL_NONE).mean():.1%} of floor pixels classed by the Astra")
    chair_heights = pixel_heights(depth, INTR, MOUNT)[chair_px]
    expected = (chair_heights > 0.08) & (chair_heights <= 0.6)
    if (labels[chair_px][expected] != PIXEL_OBSTACLE).any():
        errors.append("chair pixels 8 cm..0.6 m high must be classed obstacle")
    if (labels[~seen] != PIXEL_NONE).any():
        errors.append("pixels without depth must stay unclassified")
    return errors


# A room seen straight on: back wall 2.5 m ahead, a 30 cm crate on the left, a 3 cm tube on the right.
SEG_CRATE = Box("crate", 1.5, 0.15, 1.8, 0.55, 0.30)
SEG_TUBE = Box("tube", 1.0, -0.8, 1.04, -0.1, 0.03)
SEG_WORLD = WALLS + [SEG_CRATE, SEG_TUBE]
CONTACT_NEAR_TOL_M, CONTACT_FAR_TOL_M = 0.10, 0.02   # nearer than the object is the safe side


def _column_errors(view, what):
    """Free floor must end at the first obstacle of each column: nothing free above a contact."""
    rows = np.arange(H)[:, None]
    first = np.full(W, -1)
    first[view.contacts[:, 0]] = view.contacts[:, 1]
    beyond = (view.labels == PIXEL_FREE) & (rows <= first[None, :])
    return [f"{what}: {beyond.sum()} free pixels at or beyond a contact"] if beyond.any() else []


def _contact_errors(view, hit, mount, box, what):
    """Contacts in the columns of a box (seen from the origin, facing +x), placed on the floor with
    `mount`, must sit on its footprint - the front face, or a side face's foot further back - or up to
    CONTACT_NEAR_TOL_M nearer, never behind it. The outermost 5% of columns are let off: a column that
    grazes a corner sees a few rows of box and runs on to whatever stands behind."""
    errors = []
    cols = np.unique(np.nonzero(hit == SEG_WORLD.index(box))[1])
    mine = np.isin(view.contacts[:, 0], cols)
    if mine.sum() < 0.8 * cols.size:
        errors.append(f"{what}: {mine.sum()} contacts for {cols.size} columns of the {box.name}")
        return errors
    x = floor_xy(view.contacts[mine, 0], view.contacts[mine, 1], INTR, mount)[:, 0]
    lo, hi = np.percentile(x, [5, 95])
    if lo < box.x0 - CONTACT_NEAR_TOL_M or hi > box.x1 + CONTACT_FAR_TOL_M:
        errors.append(f"{what}: {box.name} contacts at x {lo:.3f}..{hi:.3f} m (5-95%), footprint {box.x0}..{box.x1}")
    return errors


def check_floor_segment():
    """Free floor from the Depth Anything stand-in with the Astra blind on the floor: the floor up to
    each object is free, the 30 cm crate and the 3 cm tube both stop it, at their front faces."""
    errors = []
    rng = np.random.default_rng(1)
    for trial in range(3):   # a new scale / shift each time
        depth, hit = render((0.0, 0.0, 0.0), SEG_WORLD)
        view = analyze_floor(render_disparity((0.0, 0.0, 0.0), rng, SEG_WORLD), glossy(depth, hit), [], INTR, MOUNT)
        what = f"trial {trial}"
        if view.plane is None:
            errors.append(f"{what}: no floor plane found")
            continue
        errors += _column_errors(view, what)
        rows = np.arange(H)[:, None]
        first = np.full(W, -1)
        first[view.contacts[:, 0]] = view.contacts[:, 1]
        open_floor = view.trapezoid & (hit == -1) & (rows > first[None, :])
        if (view.labels[open_floor] == PIXEL_FREE).mean() < 0.95:
            errors.append(f"{what}: only {(view.labels[open_floor] == PIXEL_FREE).mean():.1%} of the open floor free")
        for box in (SEG_CRATE, SEG_TUBE):
            free = (view.labels[hit == SEG_WORLD.index(box)] == PIXEL_FREE).mean()
            if free > 0.005:   # a corner sliver a few rows tall passes for floor; a face must not
                errors.append(f"{what}: {free:.1%} of the {box.name}'s pixels marked free")
            errors += _contact_errors(view, hit, MOUNT, box, what)
    return errors


HOLE_X0, HOLE_X1, HOLE_HALF_W, HOLE_DEPTH = 1.2, 2.2, 0.3, 0.05


def check_floor_drop():
    """A shallow hole in the floor ahead: 5 cm deep, 1.2..2.2 m, 0.6 m wide. Its bottom must read as a
    drop, nothing from its near edge on may be free, and it must not pass for an object's foot."""
    errors = []
    depth, hit = render((0.0, 0.0, 0.0), WALLS)
    t, _ = _trace((0.0, 0.0, 0.0), WALLS, MOUNT)
    v, u = np.mgrid[0:H, 0:W]
    forward, left, z_top = floor_hit(u, v, INTR, MOUNT)
    with np.errstate(invalid="ignore"):
        over = (hit == -1) & (forward >= HOLE_X0) & (forward <= HOLE_X1) & (np.abs(left) < HOLE_HALF_W)
    stretch = (MOUNT.height_m + HOLE_DEPTH) / MOUNT.height_m   # same ray, lower floor: everything scales
    sees_bottom = over & (forward * stretch <= HOLE_X1)
    t = np.where(sees_bottom, z_top * stretch, np.where(over, z_top * HOLE_X1 / forward, t))   # else: far wall
    view = analyze_floor((4.0 / t - 0.5).astype(np.float32), glossy(depth, hit), [], INTR, MOUNT)
    if view.plane is None:
        return ["no floor plane found"]
    if (view.labels[sees_bottom] == PIXEL_DROP).mean() < 0.5:
        errors.append(f"only {(view.labels[sees_bottom] == PIXEL_DROP).mean():.0%} of the hole's bottom marked drop")
    hole_cols = np.nonzero(over.sum(axis=0) >= 10)[0]
    near_edge = H - 1 - np.argmax(over[::-1, hole_cols], axis=0)   # lowest row over the hole, per column
    beyond = (view.labels[:, hole_cols] == PIXEL_FREE) & (np.arange(H)[:, None] <= near_edge[None, :])
    if beyond.any():
        errors.append(f"{beyond.sum()} free pixels at or beyond the hole's near edge")
    stops = view.contacts[np.isin(view.contacts[:, 0], hole_cols)]
    xy = floor_xy(stops[:, 0], stops[:, 1], INTR, MOUNT)
    if (xy[:, 0] < HOLE_X1 - 0.1).any():
        errors.append(f"{(xy[:, 0] < HOLE_X1 - 0.1).sum()} obstacle contacts made from the hole's bottom")
    return errors


def check_yolo_veto():
    """An object Depth Anything misses entirely but YOLO boxes: no free floor in or beyond the box, its
    bottom becomes contacts, and with no Astra depth in the box its label lands at those contacts."""
    errors = []
    depth, hit = render((0.0, 0.0, 0.0), SEG_WORLD)
    rows, cols = np.nonzero(hit == SEG_WORLD.index(SEG_CRATE))
    box = (float(cols.min()), float(rows.min()), float(cols.max() + 1), float(rows.max() + 1))
    missed = render_disparity((0.0, 0.0, 0.0), np.random.default_rng(2), WALLS + [SEG_TUBE])   # no crate
    view = analyze_floor(missed, glossy(depth, hit), [box], INTR, MOUNT)
    if view.plane is None:
        return ["no floor plane found"]
    x1, y1, x2, y2 = (int(b) for b in box)
    if (view.labels[:y2, x1:x2] == PIXEL_FREE).any():
        errors.append("free floor inside or beyond the YOLO box")
    errors += _column_errors(view, "yolo")
    errors += _contact_errors(view, hit, MOUNT, SEG_CRATE, "yolo")

    occ_map = OccupancyMap()
    blind = np.where(hit == SEG_WORLD.index(SEG_CRATE), 0, glossy(depth, hit)).astype(np.uint16)
    integrate_capture(occ_map, [blind] * 3, [Detection(0, "crate", 0.9, box)], INTR, MOUNT, (0.0, 0.0, 0.0), missed)
    votes = occ_map.votes.get("crate")
    iy, ix = np.nonzero(votes) if votes is not None else ((), ())
    on_footprint = [SEG_CRATE.x0 - CONTACT_NEAR_TOL_M - 0.05 < x < SEG_CRATE.x1 + 0.05 and
                    SEG_CRATE.y0 - 0.05 < y < SEG_CRATE.y1 + 0.05
                    for x, y in (occ_map.cell_to_world(i, j) for i, j in zip(ix, iy))]
    if len(on_footprint) < 5 or np.mean(on_footprint) < 0.9:
        errors.append(f"crate label votes should sit on its footprint: {sum(on_footprint)} of {len(on_footprint)} cells")
    return errors


def check_pitch_from_contacts():
    """A camera really pitched 3 or 7 deg down, configured as 5: the pitch measured from the contacts
    under the walls comes back, and a crate's contacts land on its face with it."""
    errors = []
    for true_pitch in (3.0, 7.0):
        truth = MOUNT._replace(pitch_deg=true_pitch)
        depth, hit = render((0.0, 0.0, 0.0), SEG_WORLD, truth)
        view = analyze_floor(render_disparity((0.0, 0.0, 0.0), np.random.default_rng(3), SEG_WORLD, truth),
                             glossy(depth, hit), [], INTR, MOUNT)
        if view.pitch_deg is None or abs(view.pitch_deg - true_pitch) > 0.3:
            errors.append(f"true pitch {true_pitch}: measured {view.pitch_deg}")
            continue
        errors += _contact_errors(view, hit, MOUNT._replace(pitch_deg=view.pitch_deg), SEG_CRATE, f"pitch {true_pitch}")
    return errors


def check_object_mask():
    errors = []
    depth = np.full((H, W), 3000, np.uint16)
    depth[200:280, 300:340] = 1200
    mask = object_mask(depth, (280.0, 180.0, 360.0, 300.0), 1.2)
    if mask.sum() != 80 * 40 or not mask[200:280, 300:340].all():
        errors.append(f"object_mask should keep exactly the 1.2 m pixels (3200), got {mask.sum()}")
    return errors


def _build_map(with_da=True):
    """16 captures on the glossy floor: the Astra sees no floor, free space comes from the stand-in."""
    occ_map = OccupancyMap()
    poses = [(0.0, 0.0, math.radians(a)) for a in range(0, 360, 45)]
    poses += [(0.8, -0.6, math.radians(a)) for a in range(0, 360, 45)]
    rng = np.random.default_rng(0)
    for pose in poses:
        depth, hit = render(pose)
        disparity = render_disparity(pose, rng) if with_da else None
        integrate_capture(occ_map, [glossy(depth, hit)] * 3, _detections(hit), INTR, MOUNT, pose, disparity)
    return occ_map, poses


def check_no_da_no_free():
    """Without Depth Anything the Astra alone never marks a cell free - not even where it sees floor."""
    occ_map = OccupancyMap()
    for heading in range(0, 360, 45):
        pose = (0.0, 0.0, math.radians(heading))
        integrate_capture(occ_map, [render(pose)[0]] * 3, [], INTR, MOUNT, pose)
    return [f"{occ_map.free().sum()} cells free from the Astra alone"] if occ_map.free().any() else []


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
              ("check_object_mask", check_object_mask), ("check_floor_segment", check_floor_segment),
              ("check_floor_drop", check_floor_drop), ("check_yolo_veto", check_yolo_veto),
              ("check_pitch_from_contacts", check_pitch_from_contacts), ("check_no_da_no_free", check_no_da_no_free),
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
