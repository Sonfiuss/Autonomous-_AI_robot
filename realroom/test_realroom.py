"""Offline tests of realroom - no camera, no robot. Run: python3 realroom/test_realroom.py

map_store: publishing a drive_map run (scene + known grid, the grid's origin moved into the scene frame),
moving the robot's pose and its history, publishing the current map again (the pose stays, unless reset or
rebuilt in another frame) and the pictures it keeps drawn. map_image: free / occupied / unseen floor far apart
in lightness and each drawn where the grid has it, the robot's chassis where it stands with its front wheel
ahead. planner: the swept corridor, and a path refused for touching an occupied cell, for crossing unseen
floor, and for a start MV had to move; a clear room plans ROTATE / FORWARD legs. The planner checks need CM's
mv_client and libmv (project/tools/build_mv.sh) and are skipped without.
"""
import json
import math
import os
import sys
import tempfile

import cv2
import numpy as np

import map_image
import map_store
import planner

sys.path.insert(0, os.path.join(os.path.dirname(map_store.BASE_DIR), "vision"))
from scene_export import robot_shape  # noqa: E402  (vision/, path set just above)

RES = 0.05
ROOM_W, ROOM_L = 3.0, 2.0
ROBOT = {"x": 0.5, "y": 1.0, "theta": 0.0, "radius": 0.225}
BOX = {"id": "obj_1", "class": "chair", "confidence": 0.9, "color": "unknown", "center": {"x": 2.4, "y": 1.6},
       "yaw": 0.0, "polygon": [{"x": 2.2, "y": 1.4}, {"x": 2.6, "y": 1.4}, {"x": 2.6, "y": 1.8}, {"x": 2.2, "y": 1.8}],
       "free_sides": ["front", "back"], "near": []}
OFFSET = {"x": -1.0, "y": -0.5}                    # scene = map - offset
MAP_ORIGIN = (-2.0, -1.5)                          # map-frame corner of cell (0, 0)
GOAL = (2.0, 1.0, 0.0)
POSE_TOL = 1e-3
LENGTH_TOL_M = 0.01                                # the checked polyline against the legs' metres
MOVE_M = 0.5
OTHER_OFFSET = {"x": -1.2, "y": -0.5}              # the same run rebuilt with its box shifted
MIN_LIGHTNESS_GAP = 40                             # CIE L* between any two of free / occupied / unseen
OCCUPIED_BLOCK = ((1.5, 1.7), (0.3, 0.5))          # x, y ranges of occupied cells in the picture check
UNSEEN_BAND_X = (2.5, 3.0)
FREE_SPOT, OCCUPIED_SPOT, UNSEEN_SPOT = (1.3, 0.8), (1.6, 0.4), (2.7, 0.2)
FACING_UP = dict(ROBOT, x=0.8, theta=math.pi / 2)  # W1 then sits 0.21 m above the centre in the picture
BEHIND_M, WHEEL_L_M = 0.1, 0.21
WHEEL_WINDOW_PX = 5
BLUE_OVER_RED = 40                                 # the translucent chassis on white reads clearly blue


def _scene():
    return {"version": "2.0", "seed": None, "units": "m", "frame": "world", "source": "astra_map", "timestamp": 0.0,
            "room": {"width": ROOM_W, "length": ROOM_L}, "walls": [], "doors": [], "robot": dict(ROBOT),
            "objects": [dict(BOX)], "map_offset": dict(OFFSET)}


def _cells(fill):
    """A grid covering the room with a cell of margin, in the scene frame: (cells, origin in the scene)."""
    cols, rows = int(ROOM_W / RES) + 2, int(ROOM_L / RES) + 2
    return np.full((rows, cols), fill, np.int8), (-RES, -RES)


def _write_run(run_dir, cells, scene_origin):
    map_dir = os.path.join(run_dir, map_store.RUN_MAP_DIR)
    os.makedirs(map_dir)
    with open(os.path.join(map_dir, map_store.RUN_SCENE), "w", encoding="utf-8") as f:
        json.dump(_scene(), f)
    np.save(os.path.join(map_dir, map_store.RUN_GRID), cells)
    map_origin = [scene_origin[0] + OFFSET["x"], scene_origin[1] + OFFSET["y"]]
    with open(os.path.join(map_dir, map_store.RUN_META), "w", encoding="utf-8") as f:
        json.dump({"res_m": RES, "origin": map_origin}, f)


def check_publish_and_pose():
    """A published run becomes latest.json with its grid; the grid's origin lands in the scene frame; moving
    the robot composes in its own frame and is recorded, with the predicted flag."""
    errors = []
    cells, origin = _cells(map_store.KNOWN_FREE)
    with tempfile.TemporaryDirectory() as tmp:
        run_dir, maps_dir = os.path.join(tmp, "20260101_000000"), os.path.join(tmp, "maps")
        _write_run(run_dir, cells, origin)
        map_store.publish_run(run_dir, maps_dir)
        scene = map_store.load_latest(maps_dir)
        grid = map_store.load_grid(scene, maps_dir)
        if scene["map_id"] != "20260101_000000" or grid.cells.shape != cells.shape:
            errors.append(f"published {scene.get('map_id')} with a {grid.cells.shape} grid")
        if max(abs(a - b) for a, b in zip(grid.origin, origin)) > POSE_TOL:
            errors.append(f"grid origin {grid.origin} in the scene frame, want {origin}")
        map_store.move_robot((0.5, 0.0, math.pi / 2), "test", maps_dir=maps_dir)
        moved = map_store.move_robot((0.2, 0.0, 0.0), "test", predicted=True, maps_dir=maps_dir)
        x, y, theta = map_store.robot_pose(moved)
        want = (ROBOT["x"] + 0.5, ROBOT["y"] + 0.2, math.pi / 2)
        if max(abs(a - b) for a, b in zip((x, y, theta), want)) > POSE_TOL:
            errors.append(f"robot at {(x, y, theta)}, want {want}")
        history = moved["pose_history"]
        if len(history) != 3 or not history[-1]["predicted"] or history[1]["predicted"]:
            errors.append(f"history {history}")
        if map_store.load_map("20260101_000000", maps_dir)["robot"]["x"] != ROBOT["x"]:
            errors.append("the map as built changed with the pose")
    with tempfile.TemporaryDirectory() as tmp:
        if map_store.move_robot((1.0, 0.0, 0.0), "test", maps_dir=tmp) is not None:
            errors.append("moved a robot with no map")
    return errors


def check_republish_keeps_pose():
    """Publishing the current map again (the run rebuilt) keeps the robot where it has moved since, and its
    history; reset_pose, or a rebuild in another frame, puts it back where the run ended. Both pictures
    exist after a publish, and a pose change redraws latest.png."""
    errors = []
    cells, origin = _cells(map_store.KNOWN_FREE)
    with tempfile.TemporaryDirectory() as tmp:
        run_dir, maps_dir = os.path.join(tmp, "20260101_000000"), os.path.join(tmp, "maps")
        _write_run(run_dir, cells, origin)
        map_store.publish_run(run_dir, maps_dir)
        pictures = [os.path.join(maps_dir, name) for name in
                    (map_store.IMAGE_PATTERN.format(id="20260101_000000"), map_store.LATEST_IMAGE)]
        if not all(os.path.exists(p) for p in pictures):
            errors.append(f"pictures after publishing: {[os.path.exists(p) for p in pictures]}")
        with open(pictures[1], "rb") as f:
            before = f.read()
        map_store.move_robot((MOVE_M, 0.0, 0.0), "test", maps_dir=maps_dir)
        with open(pictures[1], "rb") as f:
            if f.read() == before:
                errors.append("latest.png not redrawn after the robot moved")
        map_store.publish_run(run_dir, maps_dir)
        kept = map_store.load_latest(maps_dir)
        if abs(kept["robot"]["x"] - (ROBOT["x"] + MOVE_M)) > POSE_TOL or len(kept["pose_history"]) != 2:
            errors.append(f"republished: robot x {kept['robot']['x']}, {len(kept['pose_history'])} history entries")
        map_store.publish_run(run_dir, maps_dir, reset_pose=True)
        reset = map_store.load_latest(maps_dir)
        if abs(reset["robot"]["x"] - ROBOT["x"]) > POSE_TOL or len(reset["pose_history"]) != 1:
            errors.append(f"reset_pose: robot x {reset['robot']['x']}, {len(reset['pose_history'])} entries")
        map_store.move_robot((MOVE_M, 0.0, 0.0), "test", maps_dir=maps_dir)
        scene_path = os.path.join(run_dir, map_store.RUN_MAP_DIR, map_store.RUN_SCENE)
        with open(scene_path, encoding="utf-8") as f:
            scene = json.load(f)
        with open(scene_path, "w", encoding="utf-8") as f:
            json.dump(dict(scene, map_offset=OTHER_OFFSET), f)
        map_store.publish_run(run_dir, maps_dir)
        if abs(map_store.load_latest(maps_dir)["robot"]["x"] - ROBOT["x"]) > POSE_TOL:
            errors.append("a map rebuilt in another frame kept the old frame's pose")
    return errors


def _lightness(hex_color):
    """CIE L* (0..100) of "#rrggbb"."""
    lab = cv2.cvtColor(np.uint8([[map_image.bgr(hex_color)]]), cv2.COLOR_BGR2LAB)
    return float(lab[0, 0, 0]) * 100.0 / 255.0


def check_map_image():
    """Free, occupied and unseen floor at least MIN_LIGHTNESS_GAP apart and each drawn where the grid has it;
    the chassis blue where the robot stands, its front wheel W1 ahead of it and none straight behind."""
    errors = []
    shades = {name: _lightness(getattr(map_image, name)) for name in ("FREE", "OCCUPIED", "UNSEEN")}
    names = sorted(shades)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if abs(shades[a] - shades[b]) < MIN_LIGHTNESS_GAP:
                errors.append(f"{a} and {b} only {abs(shades[a] - shades[b]):.0f} apart in lightness")
    cells, origin = _cells(map_store.KNOWN_FREE)

    def cols(x0, x1):
        return slice(int((x0 - origin[0]) / RES), int((x1 - origin[0]) / RES))

    (bx, by), ux = OCCUPIED_BLOCK, UNSEEN_BAND_X
    cells[int((by[0] - origin[1]) / RES):int((by[1] - origin[1]) / RES), cols(*bx)] = map_store.KNOWN_OCCUPIED
    cells[:, cols(*ux)] = map_store.KNOWN_UNSEEN
    scene = dict(_scene(), robot=dict(FACING_UP, **robot_shape()))
    img = map_image.draw(scene, map_store.KnownGrid(cells, RES, origin))
    view = map_image._view(scene, None)

    def at(x, y):
        u, v = view.ipx(x, y)
        return tuple(int(c) for c in img[v, u])

    for spot, allowed in ((FREE_SPOT, ("FREE",)), (OCCUPIED_SPOT, ("OCCUPIED",)), (UNSEEN_SPOT, ("UNSEEN", "HATCH"))):
        if at(*spot) not in [map_image.bgr(getattr(map_image, name)) for name in allowed]:
            errors.append(f"{spot} drawn {at(*spot)}, want {allowed}")
    blue, _, red = at(FACING_UP["x"], FACING_UP["y"] - BEHIND_M)
    if blue - red < BLUE_OVER_RED:
        errors.append(f"chassis behind the centre drawn {at(FACING_UP['x'], FACING_UP['y'] - BEHIND_M)}")

    def wheel_pixels(x, y):
        u, v = view.ipx(x, y)
        window = img[v - WHEEL_WINDOW_PX:v + WHEEL_WINDOW_PX + 1, u - WHEEL_WINDOW_PX:u + WHEEL_WINDOW_PX + 1]
        return int(np.all(window == map_image.bgr(map_image.WHEEL), axis=2).sum())

    ahead = wheel_pixels(FACING_UP["x"], FACING_UP["y"] + WHEEL_L_M)
    behind = wheel_pixels(FACING_UP["x"], FACING_UP["y"] - WHEEL_L_M)
    if not ahead or behind:
        errors.append(f"wheel pixels ahead {ahead}, behind {behind}: W1 is the front wheel")
    return errors


def check_swept_cells():
    """A straight 1 m path sweeps a 1 m x 2r band of cells, and nothing far from it."""
    cells, origin = _cells(map_store.KNOWN_FREE)
    grid = map_store.KnownGrid(cells, RES, origin)
    swept = planner.swept_cells(grid, [(0.5, 1.0), (1.5, 1.0)], ROBOT["radius"])
    iy, ix = np.divmod(swept, cells.shape[1])
    x = origin[0] + (ix + 0.5) * RES
    y = origin[1] + (iy + 0.5) * RES
    errors = []
    reach = ROBOT["radius"] + RES
    if x.min() < 0.5 - reach or x.max() > 1.5 + reach or np.abs(y - 1.0).max() > reach:
        errors.append(f"swept x {x.min():.2f}..{x.max():.2f}, y off {np.abs(y - 1.0).max():.2f}")
    band = (1.0 + 2 * ROBOT["radius"]) * 2 * ROBOT["radius"] / RES ** 2
    if not 0.8 * band < swept.size < 1.6 * band:
        errors.append(f"{swept.size} cells swept, about {band:.0f} expected")
    return errors


def check_planner():
    """A clear room plans turn-go-turn legs; an occupied cell on the way, a band of unseen floor, and a start
    inside an obstacle's margin are refused."""
    try:
        planner._mv_client().load_library()
    except Exception as exc:                          # no libmv: CM's MV build is missing
        print(f"    skipped: {exc}")
        return []
    errors = []
    scene = _scene()
    cells, origin = _cells(map_store.KNOWN_FREE)
    clear = planner.plan(scene, map_store.KnownGrid(cells, RES, origin), GOAL)
    kinds = {kind for kind, _ in clear.legs}
    forward = sum(v for kind, v in clear.legs if kind == planner.FORWARD)
    if not clear.ok or not kinds <= {planner.ROTATE, planner.FORWARD} or abs(forward - 1.5) > 0.1:
        errors.append(f"clear room: {clear.ok} {clear.reason} legs {clear.legs}")
    driven = sum(math.dist(a, b) for a, b in zip(clear.path, clear.path[1:]))
    if abs(driven - forward) > LENGTH_TOL_M:
        errors.append(f"checked path {driven:.3f} m long, the legs drive {forward:.3f} m: not the driven polyline")
    blocked = cells.copy()
    blocked[int((1.0 - origin[1]) / RES), int((1.2 - origin[0]) / RES)] = map_store.KNOWN_OCCUPIED
    result = planner.plan(scene, map_store.KnownGrid(blocked, RES, origin), GOAL)
    if result.ok or result.occupied_cells < 1:
        errors.append(f"occupied cell on the way: {result.ok} {result.reason}")
    unseen = cells.copy()
    unseen[:, int((0.8 - origin[0]) / RES):int((1.8 - origin[0]) / RES)] = map_store.KNOWN_UNSEEN
    result = planner.plan(scene, map_store.KnownGrid(unseen, RES, origin), GOAL)
    if result.ok or not result.unseen_share > planner.MAX_UNSEEN_SHARE:
        errors.append(f"unseen band: {result.ok} {result.reason} ({result.unseen_share})")
    squeezed = dict(scene, robot=dict(ROBOT, x=2.4, y=1.2))     # 0.2 m from the chair: inside its margin
    result = planner.plan(squeezed, map_store.KnownGrid(cells, RES, origin), GOAL)
    if result.ok:
        errors.append("a start inside an obstacle's margin was planned from")
    return errors


def main():
    failed = 0
    for check in (check_publish_and_pose, check_republish_keeps_pose, check_map_image, check_swept_cells,
                  check_planner):
        errors = check()
        print(f"{'PASS' if not errors else 'FAIL'} {check.__name__}")
        for e in errors:
            print("   ", e)
        failed += bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
