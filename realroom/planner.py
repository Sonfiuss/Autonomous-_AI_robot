"""Paths on the real room's map - realroom's counterpart of the MV planning CM does on the simulator's rooms.

MV (project/src/MV, through CM's mv_client) plans on the scene: its objects as polygons, everything else
in the room's box free. In a real map much of that box was never seen, so every MV path is checked against
the map's KnownGrid before the robot may drive it - the robot's disc swept along the path:
  - may touch no occupied cell (the grid is the evidence; MV's polygons are only hulls of what was seen);
  - may cross at most MAX_UNSEEN_SHARE of unseen cells - floor no camera looked at, the blind zone right in
    front of the robot included (it sees the floor from ~0.78 m ahead of its centre);
  - must start where the robot is: MV snaps a start in an obstacle's margin to free space, and a plan from
    elsewhere would be driven from here.
The legs are MV's non-holonomic plan, turn - go - turn: ("ROTATE", rad relative) and ("FORWARD", m), straight
between MV's waypoints - so that polyline, not MV's dense grid path (which bends around corners the legs cut),
is what the swept check walks and what RoomPlan.path returns.
"""
import collections
import math
import os
import sys

import numpy as np

from map_store import KNOWN_OCCUPIED, KNOWN_UNSEEN, robot_pose

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CM_DIR = os.path.join(REPO_ROOT, "project", "src", "CM")        # mv_client.py + its config
ROTATE, FORWARD, STOP = "ROTATE", "FORWARD", "STOP"
SAMPLE_STEP_M = 0.025          # the swept disc is checked this often along the path (half a cell)
MAX_UNSEEN_SHARE = 0.35        # of the swept cells; map 20260928_222009 routes read 9-29 % - realroom/README.md
MAX_START_SNAP_M = 0.05        # MV moved the start further than this: the plan is not from here
MIN_LEG = 1e-4                 # mc::cfg::MIN_LEG_LENGTH_M / MIN_LEG_ANGLE_RAD: shorter legs are no legs

# ok; reason (why not); legs [(ROTATE | FORWARD, value)]; path [(x, y)] and length_m: the polyline driven;
# the swept cells' shares: unseen, occupied (count); goal (x, y, theta) planned to.
RoomPlan = collections.namedtuple("RoomPlan", "ok reason legs path length_m unseen_share occupied_cells goal")


def _mv_client():
    """CM's mv_client (ctypes over libmv)."""
    if CM_DIR not in sys.path:
        sys.path.insert(0, CM_DIR)
    import mv_client
    return mv_client


def swept_cells(grid, path, radius):
    """Flat ids of the grid cells within radius of the polyline path [(x, y), ...], sampled every
    SAMPLE_STEP_M. Cells outside the grid are left out."""
    rows, cols = grid.cells.shape
    reach = int(math.ceil(radius / grid.res))
    dy, dx = np.mgrid[-reach:reach + 1, -reach:reach + 1]
    disc = np.hypot(dx, dy) * grid.res <= radius
    dx, dy = dx[disc], dy[disc]
    points = [np.asarray(path[0], np.float64)]
    for a, b in zip(path, path[1:]):
        a, b = np.asarray(a, np.float64), np.asarray(b, np.float64)
        steps = max(int(math.ceil(np.linalg.norm(b - a) / SAMPLE_STEP_M)), 1)
        points += [a + (b - a) * k / steps for k in range(1, steps + 1)]
    centers = np.unique(np.floor((np.array(points) - grid.origin) / grid.res).astype(np.int64), axis=0)
    ix = (centers[:, 0:1] + dx[None, :]).ravel()
    iy = (centers[:, 1:2] + dy[None, :]).ravel()
    inside = (ix >= 0) & (ix < cols) & (iy >= 0) & (iy < rows)
    return np.unique(iy[inside] * cols + ix[inside])


def _refuse(reason, goal, path=(), length_m=0.0, unseen=math.nan, occupied=0):
    """A RoomPlan that may not be driven, saying why."""
    return RoomPlan(False, reason, [], list(path), length_m, unseen, occupied, goal)


def plan(scene, grid, goal):
    """RoomPlan from the robot's pose in scene to goal (x, y, theta), checked against grid (map_store.KnownGrid)."""
    start = robot_pose(scene)
    goal = (float(goal[0]), float(goal[1]), float(goal[2]))
    result = _mv_client().plan_path(scene, {"x": goal[0], "y": goal[1], "theta": goal[2]},
                                    {"x": start[0], "y": start[1], "theta": start[2]}, holonomic=False)
    if not result["ok"]:
        return _refuse(f"MV found no path ({result['reason']})", goal)
    snapped = result["snapped_start"]
    snap = math.hypot(snapped["x"] - start[0], snapped["y"] - start[1])
    path = [(start[0], start[1])] + [(p["x"], p["y"]) for p in result["waypoints"]]   # what the legs drive
    length_m = sum(math.dist(a, b) for a, b in zip(path, path[1:]))                   # not MV's dense path's
    if snap > MAX_START_SNAP_M:
        return _refuse(f"MV started {snap:.2f} m from the robot (inside an obstacle's margin)", goal, path,
                       length_m)
    cells = swept_cells(grid, path, scene["robot"]["radius"]).astype(np.int64)
    values = grid.cells.reshape(-1)[cells]
    occupied = int((values == KNOWN_OCCUPIED).sum())
    unseen = float((values == KNOWN_UNSEEN).mean()) if values.size else 1.0
    if occupied:
        return _refuse(f"the path touches {occupied} occupied cell(s)", goal, path, length_m, unseen, occupied)
    if unseen > MAX_UNSEEN_SHARE:
        return _refuse(f"{unseen:.0%} of the path's floor was never seen (limit {MAX_UNSEEN_SHARE:.0%})", goal, path,
                       length_m, unseen)
    legs = []
    for prim in result["primitives"]:
        if prim["type"] == STOP:
            continue
        if prim["type"] not in (ROTATE, FORWARD):
            return _refuse(f"MV returned a {prim['type']} leg (only ROTATE / FORWARD are driven)", goal, path,
                           length_m, unseen)
        if abs(prim["a"]) >= MIN_LEG:
            legs.append((prim["type"], float(prim["a"])))
    return RoomPlan(True, "", legs, path, length_m, unseen, 0, goal)
