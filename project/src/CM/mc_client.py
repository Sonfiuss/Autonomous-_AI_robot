"""ctypes bridge from CM (Python) to the MC motion executor (C API, `include/MC/mc_api.h`).

Takes the primitive list MV produced and returns one row per control tick: the wheel speeds, the
pulse rate per wheel and the pose reached. The shared library is built by `project/tools/build_mc.sh`
(or CMake target `mc_shared`) and must match this interpreter's bitness. Set `MC_LIB` to override.

Usage:
    import mc_client
    trajectory = mc_client.run(plan["primitives"], start_theta=1.57)
    # -> {"ok": True, "steps": [{t, u, v, r, w[3], hz[3], dir[3], x, y, theta}...],
    #     "duration_s": 8.14, "end": {...}, "stride": 1, "tick_s": 0.02}
"""
import ctypes
import logging
import os

import config

logger = logging.getLogger(__name__)

# ---- McStatus (mirrors enum McStatus in MC/mc_api.h)
MC_OK = 0
MC_ERR_ARGS = 1
MC_ERR_PRIMITIVE = 2
MC_ERR_KINEMATICS = 3
MC_ERR_BUFFER = 4

# MV primitive names in the order MC expects them (MvPrimitive.type == index).
PRIMITIVE_CODES = {"ROTATE": 0, "FORWARD": 1, "MOVE": 2, "STOP": 3}
NUM_WHEELS = 3


class McError(RuntimeError):
    """The MC library is missing, unloadable or was called with a malformed request."""


# ---- C structs (field order and types must match MC/mc_api.h exactly)
class McPrimitive(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("a", ctypes.c_float), ("b", ctypes.c_float)]


class McStep(ctypes.Structure):
    _fields_ = [("t", ctypes.c_float),
                ("u", ctypes.c_float), ("v", ctypes.c_float), ("r", ctypes.c_float),
                ("w", ctypes.c_float * NUM_WHEELS),
                ("hz", ctypes.c_float * NUM_WHEELS),
                ("dir", ctypes.c_byte * NUM_WHEELS),
                ("x", ctypes.c_float), ("y", ctypes.c_float), ("theta", ctypes.c_float)]


class McRequest(ctypes.Structure):
    _fields_ = [("prims", ctypes.POINTER(McPrimitive)),
                ("n_prims", ctypes.c_int),
                ("start_theta", ctypes.c_float),
                ("cruise_speed", ctypes.c_float),
                ("yaw_rate", ctypes.c_float),
                ("dt", ctypes.c_float)]


class McResult(ctypes.Structure):
    _fields_ = [("steps", ctypes.POINTER(McStep)),
                ("step_cap", ctypes.c_int),
                ("step_len", ctypes.c_int),
                ("duration_s", ctypes.c_float),
                ("end_x", ctypes.c_float),
                ("end_y", ctypes.c_float),
                ("end_theta", ctypes.c_float)]


_lib = None                                        # cached handle; None until the first run()


def library_candidates():
    """Absolute paths the loader tries, most specific first. MC_LIB wins when set."""
    override = os.environ.get("MC_LIB", "").strip()
    if override:
        return [os.path.abspath(override)]
    names = ("mc.dll", "libmc.so", "mc.so")        # build_mc.sh / CMake mc_shared output names
    dirs = (os.path.join(config.PROJECT_DIR, "build", "mc"),
            os.path.join(config.PROJECT_DIR, "build"),
            os.path.join(config.PROJECT_DIR, "build", "Release"))
    return [os.path.join(d, n) for d in dirs for n in names]


def load_library():
    """Loads (and caches) the MC shared library. Raises McError when it cannot be found."""
    global _lib
    if _lib is not None:
        return _lib
    tried = library_candidates()
    path = next((p for p in tried if os.path.isfile(p)), None)
    if path is None:
        raise McError("MC library not found; build it with project/tools/build_mc.sh. Tried: "
                      + os.pathsep.join(tried))
    try:
        lib = ctypes.CDLL(path)
    except OSError as exc:                          # wrong bitness, missing runtime DLL
        raise McError("cannot load {}: {}".format(path, exc))
    lib.mc_run.argtypes = [ctypes.POINTER(McRequest), ctypes.POINTER(McResult)]
    lib.mc_run.restype = ctypes.c_int
    lib.mc_max_speed.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                                 ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)]
    lib.mc_max_speed.restype = ctypes.c_int
    lib.mc_status_text.argtypes = [ctypes.c_int]
    lib.mc_status_text.restype = ctypes.c_char_p
    lib.mc_version.argtypes = []
    lib.mc_version.restype = ctypes.c_int
    logger.info("MC library %s (version %d)", path, lib.mc_version())
    _lib = lib
    return _lib


def status_text(status):
    """Human-readable McStatus text, taken from the library so the wording lives in one place."""
    try:
        text = load_library().mc_status_text(int(status))
    except McError as exc:
        return str(exc)
    return text.decode("utf-8", "replace") if text else "status {}".format(status)


def _primitive_code(primitive):
    """Accepts MV's JSON form ({'type': 'FORWARD'}) or a raw int code."""
    raw = primitive.get("type")
    if isinstance(raw, int):
        return raw
    code = PRIMITIVE_CODES.get(str(raw).upper())
    if code is None:
        raise McError("unknown primitive type {!r}".format(raw))
    return code


def run(primitives, start_theta=0.0, cruise_speed=0.0, yaw_rate=0.0, dt=0.0, max_rows=0):
    """Expands a primitive list into per-tick wheel speeds.

    primitives: MV's `primitives` list, as returned by mv_client.plan_path.
    start_theta: robot heading at the first tick (rad) — MOVE legs need it.
    cruise_speed / yaw_rate / dt: 0 means the library's compiled default.
    max_rows: cap on the rows returned (0 = every tick). The whole trajectory is always executed;
              a longer run is thinned by an integer stride, which the reply reports.
    Returns a JSON-ready dict; `ok` is False with `reason` when the library refuses the request.
    """
    lib = load_library()
    if max_rows < 0:
        raise McError("max_rows must be >= 0")
    count = len(primitives)
    prims = (McPrimitive * max(count, 1))()
    for i, primitive in enumerate(primitives):
        prims[i].type = _primitive_code(primitive)
        prims[i].a = float(primitive.get("a", 0.0))
        prims[i].b = float(primitive.get("b", 0.0))

    steps = (McStep * config.MC_MAX_STEPS)()
    req = McRequest(prims, count, float(start_theta), float(cruise_speed), float(yaw_rate), float(dt))
    res = McResult()
    res.steps = steps
    res.step_cap = config.MC_MAX_STEPS

    status = lib.mc_run(ctypes.byref(req), ctypes.byref(res))
    if status != MC_OK:
        reason = status_text(status)
        logger.info("mc_run failed: %s (%d)", reason, status)
        return {"ok": False, "status": status, "reason": reason}

    # Thin the reply only once the real tick count is known.
    stride = 1
    if max_rows and res.step_len > max_rows:
        stride = (res.step_len + max_rows - 1) // max_rows
    digits = config.MV_ROUND_DIGITS
    fine = config.MV_FINE_ROUND_DIGITS
    rows = []
    for i in range(0, res.step_len, stride):
        s = steps[i]
        rows.append({
            "t": round(s.t, fine),
            "u": round(s.u, fine), "v": round(s.v, fine), "r": round(s.r, fine),
            "w": [round(s.w[k], fine) for k in range(NUM_WHEELS)],
            "hz": [round(s.hz[k], config.MC_HZ_ROUND_DIGITS) for k in range(NUM_WHEELS)],
            "dir": [int(s.dir[k]) for k in range(NUM_WHEELS)],
            "x": round(s.x, digits), "y": round(s.y, digits), "theta": round(s.theta, fine),
        })
    tick = dt if dt > 0.0 else (res.duration_s / res.step_len if res.step_len else 0.0)
    return {
        "ok": True,
        "status": MC_OK,
        "steps": rows,
        "step_count": res.step_len,          # ticks executed, before stride
        "stride": stride,
        "tick_s": round(tick, 5),
        "duration_s": round(res.duration_s, digits),
        "end": {"x": round(res.end_x, digits), "y": round(res.end_y, digits),
                "theta": round(res.end_theta, fine)},
    }


def max_speed(u, v, r, requested=0.0):
    """Feasible chassis speed and acceleration along a body-frame direction."""
    lib = load_library()
    v_max = ctypes.c_float(0.0)
    accel = ctypes.c_float(0.0)
    status = lib.mc_max_speed(float(u), float(v), float(r), float(requested),
                              ctypes.byref(v_max), ctypes.byref(accel))
    if status != MC_OK:
        return {"ok": False, "status": status, "reason": status_text(status)}
    return {"ok": True, "v_max": round(v_max.value, config.MV_FINE_ROUND_DIGITS),
            "accel": round(accel.value, config.MV_FINE_ROUND_DIGITS)}


def _main():
    """Smoke test: drive half a metre, turn 90 degrees, stop."""
    import json
    logging.basicConfig(level=logging.INFO)
    prims = [{"type": "FORWARD", "a": 0.5, "b": 0.0},
             {"type": "ROTATE", "a": 1.5708, "b": 0.0},
             {"type": "STOP", "a": 0.0, "b": 0.0}]
    result = run(prims, max_rows=20)
    print(json.dumps({k: v for k, v in result.items() if k != "steps"}, indent=2))
    for row in result["steps"]:
        print("t={t:6.2f}  w={w}  pose=({x:.3f}, {y:.3f}, {theta:.3f})".format(**row))
    print("limits:", json.dumps(max_speed(1, 0, 0)), json.dumps(max_speed(0, 0, 1)))


if __name__ == "__main__":
    _main()
