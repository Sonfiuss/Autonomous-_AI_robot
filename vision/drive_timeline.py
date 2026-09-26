"""The commanded motion of a demo_drive run (communication/demo_drive.py), as a function of time.

run.json holds the plan's legs (MV primitive, wire line, MC duration) and the moment demo_drive
printed each leg's status. That moment is not when the robot moved: robot_link waits up to
READY_WAIT_MS (2 s) before the first leg and each later leg waits for the previous one's ack, so a
leg's shape is known exactly - MC runs the RM chain the firmware runs - but its start is not.
align_leg() finds the start by matching the leg's speed profile to the motion measured in the images.

Units: m, rad, s. A leg's displacement is metres along +x for FORWARD, radians CCW for ROTATE.
"""
import collections
import json
import logging
import os
import sys

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CM_DIR = os.path.join(REPO_ROOT, "project", "src", "CM")   # mc_client.py + its config
RUN_FILE = "run.json"
ROTATE, FORWARD, STOP = "ROTATE", "FORWARD", "STOP"
PROFILE_DT_S = 0.02            # MC's tick; also the fallback profile's
STATUS_PREFIX = "leg "         # demo_drive status text: "leg 2/5  T 90.000 0.300  (...)"

# kind: ROTATE / FORWARD / STOP. amount: rad or m. status_t: wall time demo_drive announced the leg.
# profile_t / profile_s: leg-relative time and displacement along the leg, from MC (or the fallback).
Leg = collections.namedtuple("Leg", "index kind amount duration_s status_t profile_t profile_s")
Run = collections.namedtuple("Run", "legs cruise_speed yaw_rate end_t")

logger = logging.getLogger(__name__)


def _mc_profile(kind, amount, cruise_speed, yaw_rate):
    """(t, s) from MC, or None when libmc / mc_client is unavailable."""
    if CM_DIR not in sys.path:
        sys.path.insert(0, CM_DIR)
    try:
        import mc_client
        result = mc_client.run([{"type": kind, "a": amount, "b": 0.0}], cruise_speed=cruise_speed,
                               yaw_rate=yaw_rate, dt=PROFILE_DT_S)
    except (ImportError, OSError, RuntimeError) as exc:   # McError is a RuntimeError
        logger.warning("MC unavailable (%s): legs use a symmetric trapezoid instead", exc)
        return None
    if not result["ok"] or not result["steps"]:
        return None
    key = "theta" if kind == ROTATE else "x"
    t = np.array([0.0] + [row["t"] for row in result["steps"]])
    s = np.array([0.0] + [row[key] for row in result["steps"]])
    return t, s


def trapezoid_profile(amount, speed, duration_s, dt=PROFILE_DT_S):
    """(t, s) of a symmetric trapezoid covering |amount| at cruise `speed` in duration_s (a triangle
    when there is no time to cruise) - the fallback when MC is missing."""
    dist = abs(amount)
    t = np.arange(0.0, duration_s + dt, dt)
    if dist == 0.0 or duration_s <= 0.0:
        return t, np.zeros_like(t)
    peak = min(speed, 2.0 * dist / duration_s)
    ramp = max(duration_s - dist / peak, 0.0) if peak > 0 else 0.0
    ramp = min(ramp, duration_s / 2.0)
    accel = peak / ramp if ramp > 0 else np.inf
    up = np.minimum(t, ramp)
    down = np.clip(t - (duration_s - ramp), 0.0, ramp)
    cruise = np.clip(t, ramp, duration_s - ramp) - ramp
    s = 0.5 * accel * up ** 2 + peak * cruise + (peak * down - 0.5 * accel * down ** 2)
    s = np.where(np.isfinite(s), s, dist * t / duration_s)
    return t, np.sign(amount) * np.minimum(s, dist)


def load_run(run_dir):
    """Run of run_dir/run.json; STOP legs are kept (duration 0) so indices match demo_drive's."""
    with open(os.path.join(run_dir, RUN_FILE), encoding="utf-8") as f:
        data = json.load(f)
    plan = data["plan"]
    status_t = {}
    for event in data["events"]:
        text = event.get("text", "")
        if event["event"] == "status" and text.startswith(STATUS_PREFIX):
            number = int(text[len(STATUS_PREFIX):].split("/")[0])
            status_t.setdefault(number - 1, event["t"])
    end_t = max(e["t"] for e in data["events"])
    legs = []
    for i, leg in enumerate(plan["legs"]):
        kind, *args = leg["primitive"].split()
        amount = float(args[0]) if args else 0.0
        if kind not in (ROTATE, FORWARD, STOP):
            raise ValueError(f"leg {i + 1}: {kind} is not supported (demo_drive plans are F / T / S)")
        duration = float(leg["duration_s"])
        if kind == STOP or duration <= 0.0:
            t, s = np.zeros(1), np.zeros(1)
        else:
            profile = _mc_profile(kind, amount, plan["cruise_speed"], plan["yaw_rate"])
            speed = plan["yaw_rate"] if kind == ROTATE else plan["cruise_speed"]
            t, s = profile if profile is not None else trapezoid_profile(amount, speed, duration)
        logger.debug("leg %d %s %.4f: %.2f s, status %s", i + 1, kind, amount, duration, status_t.get(i))
        legs.append(Leg(i, kind, amount, duration, status_t.get(i), t, s))
    return Run(legs, plan["cruise_speed"], plan["yaw_rate"], end_t)


def displacement(leg, start_t, t):
    """Displacement along `leg` (started at start_t) reached at times t (array)."""
    return np.interp(np.asarray(t, np.float64) - start_t, leg.profile_t, leg.profile_s,
                     left=0.0, right=float(leg.profile_s[-1]))


def align_leg(leg, times, measured, window_lo, window_hi):
    """Start time in [window_lo, window_hi - duration] at which the leg's speed profile best matches
    the measured motion, by normalised correlation - which ignores the scale between command and
    reality. measured[k]: signed speed along the leg (rad/s or m/s) over (times[k-1], times[k]].
    Returns (start, score), score in [-1, 1]."""
    times = np.asarray(times, np.float64)
    inside = (times >= window_lo) & (times <= window_hi)
    if leg.duration_s <= 0.0 or inside.sum() < 3:
        return window_lo, 0.0
    t, m = times[inside], np.asarray(measured, np.float64)[inside][1:]
    step = float(np.median(np.diff(t)))
    best = (window_lo, -np.inf)
    for start in np.arange(window_lo, max(window_hi - leg.duration_s, window_lo) + step, step):
        cmd = np.diff(displacement(leg, start, t)) / np.diff(t)       # mean commanded speed per interval
        norm = np.linalg.norm(cmd) * np.linalg.norm(m)
        score = float(cmd @ m / norm) if norm > 0 else 0.0
        if score > best[1]:
            best = (float(start), score)
    return best
