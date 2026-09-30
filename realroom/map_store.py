"""The real room's map and the robot's place in it - realroom's counterpart of simulation/room's scenes/.

A map is a Scene JSON v2.0 (schema: simulation/room/README.md - the one CM and MV read) that
vision/drive_map built from a recording, with the grid of what the camera saw:
  maps/map_<id>.json       the scene as published; id = the run it came from
  maps/map_<id>_grid.npz   cells: int8 [iy, ix], 1 occupied / 0 free (seen, or under the robot's path) /
                           -1 unseen; res_m; origin_m: the scene-frame corner of cell (0, 0)
  maps/latest.json         the map later commands use; "robot" holds the robot's CURRENT pose and
                           "pose_history" every change of it (what moved it, measured or predicted)
  maps/map_<id>.png, maps/latest.png   map_image.py's pictures of both, redrawn with every change
Frame: the scene's (world) frame, metres and radians - x forward and y left at the start of the run
that built the map, shifted so the box around everything seen starts at (0, 0).

  python3 realroom/map_store.py publish vision/output/<run> [--reset-pose]    # as drive_map --publish does
  python3 realroom/map_store.py show
"""
import argparse
import collections
import json
import logging
import math
import os
import sys
import time

import cv2
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAPS_DIR_ENV = "REALROOM_MAPS_DIR"      # another maps folder, for every process that inherits it (the tests)
MAPS_DIR = os.environ.get(MAPS_DIR_ENV) or os.path.join(BASE_DIR, "maps")
LATEST_FILE = os.path.join(MAPS_DIR, "latest.json")
MAP_PATTERN = "map_{id}.json"
GRID_PATTERN = "map_{id}_grid.npz"
IMAGE_PATTERN, LATEST_IMAGE = "map_{id}.png", "latest.png"     # map_image.py's pictures
# What vision/drive_map writes into <run>/map/.
RUN_MAP_DIR, RUN_SCENE, RUN_GRID, RUN_META = "map", "scene.json", "known_grid.npy", "map_meta.json"
KNOWN_OCCUPIED, KNOWN_FREE, KNOWN_UNSEEN = 1, 0, -1      # vision/occupancy_map.known_grid
POSE_DIGITS = 4
TIME_DIGITS = 1
SOURCE_BUILT = "map built"
FULL_TURN = 2.0 * math.pi
POSE_KEYS = ("x", "y", "theta")         # the robot fields a pose update changes

# cells: int8 [iy, ix] (KNOWN_*); res: m per cell; origin: scene (x, y) of the corner of cell (0, 0).
KnownGrid = collections.namedtuple("KnownGrid", "cells res origin")

logger = logging.getLogger(__name__)


class MapError(RuntimeError):
    """No map yet, or a run without the files drive_map writes."""


def compose(pose, motion):
    """Pose (x, y, theta) moved by motion (dx, dy, dtheta) given in the pose's own body frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * motion[0] - s * motion[1], pose[1] + s * motion[0] + c * motion[1], pose[2] + motion[2]


def _write_json(path, data):
    """Atomic: a reader never sees half a map."""
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


def _read_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _history_entry(pose, source, predicted=False):
    """One pose_history item: when, what moved the robot, whether it was measured, where it ended."""
    return {"t": round(time.time(), TIME_DIGITS), "source": source, "predicted": predicted,
            "x": round(pose[0], POSE_DIGITS), "y": round(pose[1], POSE_DIGITS), "theta": round(pose[2], POSE_DIGITS)}


def _draw(scene, maps_dir, name):
    """map_image's picture of scene into maps_dir/name. A side output: failing to draw only warns, it
    never undoes the map or pose just saved."""
    import map_image                                        # it imports this module
    try:
        map_image.write_png(scene, load_grid(scene, maps_dir), os.path.join(maps_dir, name))
    except (OSError, ValueError, KeyError, cv2.error) as exc:
        logger.warning("map picture %s not drawn: %s", name, exc)


def publish_run(run_dir, maps_dir=MAPS_DIR, reset_pose=False):
    """The map drive_map built for run_dir becomes the real room's: map_<id>.json + its grid, and
    latest.json. The robot's pose is where that run ended - unless latest.json already holds this map in
    the same frame (the run rebuilt, e.g. with object colors) and reset_pose is False: the robot has moved
    since, so its current pose and history stay. Returns latest.json's path."""
    run_dir = run_dir.rstrip("/")
    map_dir = os.path.join(run_dir, RUN_MAP_DIR)
    paths = [os.path.join(map_dir, name) for name in (RUN_SCENE, RUN_GRID, RUN_META)]
    missing = [p for p in paths if not os.path.exists(p)]
    if missing:
        raise MapError(f"{run_dir}: no {', '.join(os.path.basename(p) for p in missing)} - run vision/drive_map.py first")
    scene, meta = _read_json(paths[0]), _read_json(paths[2])
    offset = scene["map_offset"]
    map_id = os.path.basename(run_dir)
    grid_name = GRID_PATTERN.format(id=map_id)
    os.makedirs(maps_dir, exist_ok=True)
    np.savez_compressed(os.path.join(maps_dir, grid_name), cells=np.load(paths[1]), res_m=meta["res_m"],
                        origin_m=[meta["origin"][0] - offset["x"], meta["origin"][1] - offset["y"]])
    robot = scene["robot"]
    built = dict(scene, map_id=map_id, source_run=run_dir, grid_file=grid_name,
                 pose_history=[_history_entry((robot["x"], robot["y"], robot["theta"]), SOURCE_BUILT)])
    _write_json(os.path.join(maps_dir, MAP_PATTERN.format(id=map_id)), built)
    scene = built
    previous = None if reset_pose else load_latest(maps_dir)
    if previous is not None and previous.get("map_id") == map_id:
        if previous.get("map_offset") == offset:
            scene = dict(built, robot=dict(robot, **{key: previous["robot"][key] for key in POSE_KEYS}),
                         pose_history=previous["pose_history"])
        else:
            logger.warning("map %s rebuilt in another frame: the robot's pose is reset to where the run ended", map_id)
    latest = os.path.join(maps_dir, os.path.basename(LATEST_FILE))
    _write_json(latest, scene)
    _draw(built, maps_dir, IMAGE_PATTERN.format(id=map_id))
    _draw(scene, maps_dir, LATEST_IMAGE)
    return latest


def load_latest(maps_dir=MAPS_DIR):
    """latest.json, or None before any map was published."""
    path = os.path.join(maps_dir, os.path.basename(LATEST_FILE))
    return _read_json(path) if os.path.exists(path) else None


def map_version(maps_dir=MAPS_DIR):
    """latest.json's modification time (ns), None before any map: it changes with every publish and every
    pose change, so a viewer that saw another value has an outdated map or robot."""
    try:
        return os.stat(os.path.join(maps_dir, os.path.basename(LATEST_FILE))).st_mtime_ns
    except FileNotFoundError:
        return None


def load_map(map_id, maps_dir=MAPS_DIR):
    """A published map as it was built, or None."""
    path = os.path.join(maps_dir, MAP_PATTERN.format(id=map_id))
    return _read_json(path) if os.path.exists(path) else None


def load_grid(scene, maps_dir=MAPS_DIR):
    """The KnownGrid published with a scene."""
    with np.load(os.path.join(maps_dir, scene["grid_file"])) as data:
        return KnownGrid(data["cells"], float(data["res_m"]), tuple(float(v) for v in data["origin_m"]))


def robot_pose(scene):
    """(x, y, theta) of the robot in a scene."""
    robot = scene["robot"]
    return robot["x"], robot["y"], robot["theta"]


def save_pose(pose, source, predicted=False, maps_dir=MAPS_DIR):
    """Sets the robot's pose in latest.json, recording what put it there. Returns the scene, or None
    when there is no map."""
    scene = load_latest(maps_dir)
    if scene is None:
        return None
    x, y, theta = pose
    scene["robot"] = dict(scene["robot"], x=round(x, POSE_DIGITS), y=round(y, POSE_DIGITS),
                          theta=round(theta % FULL_TURN, POSE_DIGITS))
    scene.setdefault("pose_history", []).append(_history_entry(pose, source, predicted))
    _write_json(os.path.join(maps_dir, os.path.basename(LATEST_FILE)), scene)
    _draw(scene, maps_dir, LATEST_IMAGE)
    return scene


def move_robot(motion, source, predicted=False, maps_dir=MAPS_DIR):
    """Moves the robot in latest.json by motion (dx, dy, dtheta) in its own body frame - what a command
    measured. Returns the scene, or None when there is no map."""
    scene = load_latest(maps_dir)
    if scene is None:
        return None
    return save_pose(compose(robot_pose(scene), motion), source, predicted, maps_dir)


def _parse_args():
    parser = argparse.ArgumentParser(description="The real room's map.")
    sub = parser.add_subparsers(dest="command", required=True)
    publish = sub.add_parser("publish", help="make a drive_map run the real room's map")
    publish.add_argument("run", help="vision/output/<run_id>")
    publish.add_argument("--reset-pose", action="store_true",
                         help="put the robot back where the run ended even when this map is already the current one")
    sub.add_parser("show", help="print the current map's summary and robot pose")
    return parser.parse_args()


def main():
    args = _parse_args()
    if args.command == "publish":
        try:
            print(publish_run(args.run, reset_pose=args.reset_pose))
        except MapError as exc:
            print(exc)
            return 1
        return 0
    scene = load_latest()
    if scene is None:
        print("no map yet: python3 vision/drive_map.py --run vision/output/<run> --publish")
        return 1
    x, y, theta = robot_pose(scene)
    print(f"map {scene['map_id']}: room {scene['room']['width']} x {scene['room']['length']} m, "
          f"{len(scene['objects'])} objects; robot x {x:.3f} y {y:.3f} m, heading "
          f"{math.degrees(math.atan2(math.sin(theta), math.cos(theta))):+.1f} deg "
          f"({scene['pose_history'][-1]['source']})")
    for obj in scene["objects"]:
        print(f"  {obj['id']:<7} {obj['class']:<10} {obj['color']:<8} at {obj['center']['x']:.2f}, "
              f"{obj['center']['y']:.2f}  free sides {obj['free_sides']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
