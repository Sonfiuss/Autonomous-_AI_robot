"""ctypes bridge from CM (Python) to the MV path planner (C API, `include/MV/mv_api.h`).

The shared library is built by `project/tools/build_mv.sh` (or CMake target `mv_shared`) and must
match the Python interpreter's bitness (64-bit here). Set `MV_LIB` to override the search.

Usage:
    import mv_client
    result = mv_client.plan_path(scene, {"x": 1.2, "y": 0.8, "theta": 1.57})
    # -> {"ok": True, "path": [{x, y}...], "waypoints": [...], "primitives": [...],
    #     "length_m": 3.14, "snapped_start": {...}, "snapped_goal": {...}}
    # -> {"ok": False, "status": 5, "reason": "no path ..."} when the goal is unreachable
"""
import ctypes
import logging
import os

import config

logger = logging.getLogger(__name__)

# ---- MvStatus (mirrors enum MvStatus in MV/mv_api.h)
MV_OK = 0
MV_ERR_ARGS = 1
MV_ERR_ROOM_TOO_BIG = 2
MV_ERR_START_BLOCKED = 3
MV_ERR_GOAL_BLOCKED = 4
MV_ERR_NO_PATH = 5
MV_ERR_BUFFER = 6

PRIMITIVE_NAMES = ("ROTATE", "FORWARD", "MOVE", "STOP")   # MvPrimitive.type is the index


class MvError(RuntimeError):
    """The MV library is missing, unloadable or was called with a malformed scene."""


# ---- C structs (field order and types must match MV/mv_api.h exactly)
class MvPoint(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float)]


class MvPose(ctypes.Structure):
    _fields_ = [("x", ctypes.c_float), ("y", ctypes.c_float), ("theta", ctypes.c_float)]


class MvPrimitive(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("a", ctypes.c_float), ("b", ctypes.c_float)]


class MvRequest(ctypes.Structure):
    _fields_ = [("room_w", ctypes.c_float),
                ("room_l", ctypes.c_float),
                ("robot_radius", ctypes.c_float),
                ("vertices", ctypes.POINTER(MvPoint)),
                ("poly_sizes", ctypes.POINTER(ctypes.c_int)),
                ("n_polys", ctypes.c_int),
                ("start", MvPose),
                ("goal", MvPose),
                ("holonomic", ctypes.c_int)]


class MvResult(ctypes.Structure):
    _fields_ = [("path", ctypes.POINTER(MvPoint)),
                ("path_cap", ctypes.c_int),
                ("path_len", ctypes.c_int),
                ("waypoints", ctypes.POINTER(MvPoint)),
                ("wp_cap", ctypes.c_int),
                ("wp_len", ctypes.c_int),
                ("prims", ctypes.POINTER(MvPrimitive)),
                ("prim_cap", ctypes.c_int),
                ("prim_len", ctypes.c_int),
                ("length_m", ctypes.c_float),
                ("snapped_start", MvPoint),
                ("snapped_goal", MvPoint)]


_lib = None                                        # cached handle; None until the first plan_path


def library_candidates():
    """Absolute paths the loader tries, most specific first. MV_LIB wins when set."""
    override = os.environ.get("MV_LIB", "").strip()
    if override:
        return [os.path.abspath(override)]
    names = ("mv.dll", "libmv.so", "mv.so")        # build_mv.sh / CMake mv_shared output names
    dirs = (os.path.join(config.PROJECT_DIR, "build", "mv"),          # tools/build_mv.sh
            os.path.join(config.PROJECT_DIR, "build"),                # cmake single-config
            os.path.join(config.PROJECT_DIR, "build", "Release"))     # cmake multi-config
    return [os.path.join(d, n) for d in dirs for n in names]


def load_library():
    """Loads (and caches) the MV shared library. Raises MvError when it cannot be found."""
    global _lib
    if _lib is not None:
        return _lib
    tried = library_candidates()
    path = next((p for p in tried if os.path.isfile(p)), None)
    if path is None:
        raise MvError("MV library not found; build it with project/tools/build_mv.sh. Tried: "
                      + os.pathsep.join(tried))
    try:
        lib = ctypes.CDLL(path)
    except OSError as exc:                          # wrong bitness, missing runtime DLL
        raise MvError("cannot load {}: {}".format(path, exc))
    lib.mv_plan.argtypes = [ctypes.POINTER(MvRequest), ctypes.POINTER(MvResult)]
    lib.mv_plan.restype = ctypes.c_int
    lib.mv_grid.argtypes = [ctypes.POINTER(MvRequest), ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int,
                            ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
                            ctypes.POINTER(ctypes.c_float)]
    lib.mv_grid.restype = ctypes.c_int
    lib.mv_status_text.argtypes = [ctypes.c_int]
    lib.mv_status_text.restype = ctypes.c_char_p
    lib.mv_version.argtypes = []
    lib.mv_version.restype = ctypes.c_int
    logger.info("MV library %s (version %d)", path, lib.mv_version())
    _lib = lib
    return _lib


def status_text(status):
    """Human-readable MvStatus text, from the library itself so the wording stays in one place."""
    try:
        text = load_library().mv_status_text(int(status))
    except MvError as exc:
        return str(exc)
    return text.decode("utf-8", "replace") if text else "status {}".format(status)


def _build_request(scene, start, goal, holonomic):
    """Packs a scene dict + poses into an MvRequest.

    Returns (request, buffers): the request only points at the vertex arrays, so the caller must keep
    the returned buffers alive for as long as it uses the request.
    """
    objects = scene.get("objects", [])
    flat, sizes = [], []
    for obj in objects:
        poly = obj.get("polygon") or []
        if len(poly) < 3:                            # MV rejects degenerate polygons
            logger.warning("skipping object %s: %d vertices", obj.get("id"), len(poly))
            continue
        sizes.append(len(poly))
        flat.extend(poly)
    vertices = (MvPoint * max(len(flat), 1))(*[MvPoint(float(p["x"]), float(p["y"])) for p in flat])
    poly_sizes = (ctypes.c_int * max(len(sizes), 1))(*sizes)
    room = scene.get("room") or {}
    robot = scene.get("robot") or {}
    req = MvRequest()
    req.room_w = float(room.get("width", 0.0))
    req.room_l = float(room.get("length", 0.0))
    req.robot_radius = float(robot.get("radius", config.MV_DEFAULT_ROBOT_RADIUS_M))
    req.vertices = vertices
    req.poly_sizes = poly_sizes
    req.n_polys = len(sizes)
    req.start = MvPose(float(start["x"]), float(start["y"]), float(start.get("theta", 0.0)))
    req.goal = MvPose(float(goal["x"]), float(goal["y"]), float(goal.get("theta", 0.0)))
    req.holonomic = 1 if holonomic else 0
    return req, (vertices, poly_sizes)


def _points(buf, count):
    return [{"x": round(buf[i].x, config.MV_ROUND_DIGITS),
             "y": round(buf[i].y, config.MV_ROUND_DIGITS)} for i in range(count)]


def plan_path(scene, goal, start=None, holonomic=False):
    """Plans start -> goal on the scene's occupancy grid.

    scene: scene JSON v2.0 dict (room, objects, robot). start defaults to the scene robot pose.
    goal:  dict with x, y and optional theta (world frame, m / rad).
    Returns a JSON-ready dict; `ok` is False (with `status` + `reason`) when no path exists.
    Raises MvError only when the library itself is unusable or the scene is malformed.
    """
    lib = load_library()
    robot = scene.get("robot") or {}
    if start is None:
        start = robot
    if "x" not in start or "y" not in start:
        raise MvError("start pose has no x/y")
    if "x" not in goal or "y" not in goal:
        raise MvError("goal pose has no x/y")
    req, _buffers = _build_request(scene, start, goal, holonomic)   # req points into _buffers

    path = (MvPoint * config.MV_MAX_PATH)()
    waypoints = (MvPoint * config.MV_MAX_PATH)()
    prims = (MvPrimitive * config.MV_MAX_PRIMS)()
    res = MvResult()
    res.path, res.path_cap = path, config.MV_MAX_PATH
    res.waypoints, res.wp_cap = waypoints, config.MV_MAX_PATH
    res.prims, res.prim_cap = prims, config.MV_MAX_PRIMS

    status = lib.mv_plan(ctypes.byref(req), ctypes.byref(res))
    if status != MV_OK:
        reason = status_text(status)
        logger.info("mv_plan failed: %s (%d)", reason, status)
        return {"ok": False, "status": status, "reason": reason}
    return {
        "ok": True,
        "status": MV_OK,
        "path": _points(path, res.path_len),
        "waypoints": _points(waypoints, res.wp_len),
        "primitives": [{"type": PRIMITIVE_NAMES[prims[i].type] if 0 <= prims[i].type < len(PRIMITIVE_NAMES) else "?",
                        "a": round(prims[i].a, config.MV_FINE_ROUND_DIGITS),
                        "b": round(prims[i].b, config.MV_FINE_ROUND_DIGITS)} for i in range(res.prim_len)],
        "length_m": round(res.length_m, config.MV_ROUND_DIGITS),
        "snapped_start": {"x": round(res.snapped_start.x, config.MV_ROUND_DIGITS),
                          "y": round(res.snapped_start.y, config.MV_ROUND_DIGITS)},
        "snapped_goal": {"x": round(res.snapped_goal.x, config.MV_ROUND_DIGITS),
                         "y": round(res.snapped_goal.y, config.MV_ROUND_DIGITS)},
    }


def grid(scene, start=None, goal=None):
    """Rasterised inflated grid, for debugging: {ok, cols, rows, res_m, cells} (row 0 at y = 0)."""
    lib = load_library()
    origin = {"x": 0.0, "y": 0.0, "theta": 0.0}
    req, _buffers = _build_request(scene, start or scene.get("robot") or origin, goal or origin, False)
    cells = (ctypes.c_ubyte * config.MV_MAX_GRID_BYTES)()
    cols, rows, res_m = ctypes.c_int(0), ctypes.c_int(0), ctypes.c_float(0.0)
    status = lib.mv_grid(ctypes.byref(req), cells, config.MV_MAX_GRID_BYTES,
                         ctypes.byref(cols), ctypes.byref(rows), ctypes.byref(res_m))
    if status != MV_OK:
        return {"ok": False, "status": status, "reason": status_text(status)}
    used = cols.value * rows.value
    return {"ok": True, "status": MV_OK, "cols": cols.value, "rows": rows.value,
            "res_m": round(res_m.value, config.MV_FINE_ROUND_DIGITS), "cells": bytes(cells[:used]).hex()}


def _main():
    """Smoke test: plan from the robot pose in the latest scene to the room centre."""
    import json
    logging.basicConfig(level=logging.INFO)
    with open(os.path.join(config.SCENES_DIR, "latest.json"), encoding="utf-8") as f:
        scene = json.load(f)
    goal = {"x": scene["room"]["width"] / 2.0, "y": scene["room"]["length"] / 2.0, "theta": 0.0}
    result = plan_path(scene, goal)
    print(json.dumps({k: v for k, v in result.items() if k != "path"}, indent=2))
    print("path points:", len(result.get("path", [])))


if __name__ == "__main__":
    _main()
