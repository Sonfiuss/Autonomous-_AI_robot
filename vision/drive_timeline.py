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
# MC's profile of a leg vs the duration run.json recorded (ms): the same build agrees to float noise, a
# build with other constants is off by whole ticks.
DURATION_TOL_S = 0.5 * PROFILE_DT_S
STATUS_PREFIX = "leg "         # demo_drive status text: "leg 2/5  T 90.000 0.300  (...)"
# run.json has carried plan.wheel_radius_m since 2026-09-27; every run recorded before it was commanded
# with constants.h's old 0.055 m (the real wheel is 0.040 m).
LEGACY_WHEEL_RADIUS_M = 0.055
MS_PER_S = 1000.0
# A leg the firmware acked in under this share of its planned duration was cut short on the ESP32, not
# slowed by the floor: 2026-09-28 backward legs ran the whole trapezoid ~1.6x fast (acked after 2.6 of
# 4.3 s, 24 of 40 cm) while their O odometry summed the full command. The ack arrives late, never early,
# by the poll and print chain (0.14-0.22 s measured), so the margin only has to cover a real shortfall.
FIRMWARE_SHORT_SHARE = 0.9

# kind: ROTATE / FORWARD / STOP. amount: rad or m. status_t: wall time demo_drive announced the leg.
# profile_t / profile_s: leg-relative time and displacement along the leg, from MC (or the fallback).
# ack_t: wall time the firmware's K for this leg reached demo_drive - robot_link announces the next leg the
# moment it arrives - or None (the last leg, or a run that failed before the next line). waits_ready: the
# leg opened its robot_link call, which holds it READY_WAIT_MS after announcing it.
Leg = collections.namedtuple("Leg", "index kind amount duration_s status_t profile_t profile_s ack_t waits_ready",
                             defaults=(None, False))
ACKED_EVENT = "leg_acked"      # leg_executor's: {"leg": N}, the moment executed leg N's K came in
# wheel_radius_m: the r the run's legs were turned into steps with (commanded metres x real r / it = metres
# the wheels rolled).
Run = collections.namedtuple("Run", "legs cruise_speed yaw_rate end_t wheel_radius_m")

logger = logging.getLogger(__name__)


def _mc_client():
    """CM's mc_client module, CM_DIR put on sys.path first. ImportError when it is missing."""
    if CM_DIR not in sys.path:
        sys.path.insert(0, CM_DIR)
    import mc_client
    return mc_client


def _mc_profile(kind, amount, cruise_speed, yaw_rate):
    """(t, s) from MC, or None when libmc / mc_client is unavailable."""
    try:
        result = _mc_client().run([{"type": kind, "a": amount, "b": 0.0}], cruise_speed=cruise_speed,
                                  yaw_rate=yaw_rate, dt=PROFILE_DT_S)
    except (ImportError, OSError, RuntimeError) as exc:   # McError is a RuntimeError
        logger.warning("MC unavailable (%s): legs use a symmetric trapezoid instead", exc)
        return None
    if not result["ok"] or not result["steps"]:
        return None
    key = "theta" if kind == ROTATE else "x"
    # A row's t is its tick's START, its pose where the tick ENDS: pair each pose with t + tick, so the
    # profile runs 0 .. duration_s and does not lead the motion by a tick.
    t = np.array([0.0] + [row["t"] + result["tick_s"] for row in result["steps"]])
    s = np.array([0.0] + [row[key] for row in result["steps"]])
    return t, s


def distance_scale(run):
    """Metres the wheels rolled per commanded metre: the real wheel radius (constants.h, measured
    2026-09-27) over the one the run was commanded with - 1 since that fix, 0.040 / 0.055 before it.
    Slip is not in it: that is what the floor odometry measures on top."""
    try:
        real = _mc_client().chassis()["wheel_radius_m"]
    except (ImportError, RuntimeError) as exc:   # McError is a RuntimeError
        logger.warning("chassis constants unavailable (%s): commanded distances taken as rolled", exc)
        return 1.0
    return real / run.wheel_radius_m


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
    """Run of run_dir/run.json; STOP legs are kept (duration 0) so indices match demo_drive's. A
    compensated run (communication/leg_executor) lists the legs it actually sent, one robot_link call
    each, in executed_legs, and stamps each K with an ACKED_EVENT: those are used instead."""
    with open(os.path.join(run_dir, RUN_FILE), encoding="utf-8") as f:
        data = json.load(f)
    plan = data["plan"]
    status_t, acked_t = {}, {}
    for event in data["events"]:
        text = event.get("text", "")
        if event["event"] == "status" and text.startswith(STATUS_PREFIX):
            number = int(text[len(STATUS_PREFIX):].split("/")[0])
            status_t.setdefault(number - 1, event["t"])
        elif event["event"] == ACKED_EVENT:
            acked_t[event["leg"] - 1] = event["t"]
    end_t = max(e["t"] for e in data["events"])
    legs = []
    for i, leg in enumerate(data.get("executed_legs") or plan["legs"]):
        kind, *args = leg["primitive"].split()
        amount = float(args[0]) if args else 0.0
        if kind not in (ROTATE, FORWARD, STOP):
            raise ValueError(f"leg {i + 1}: {kind} is not supported (demo_drive plans are F / T / S)")
        duration = float(leg["duration_s"])
        if kind == STOP or duration <= 0.0:
            t, s = np.zeros(1), np.zeros(1)
        else:
            profile = _mc_profile(kind, amount, plan["cruise_speed"], plan["yaw_rate"])
            if profile is not None and abs(profile[0][-1] - duration) > DURATION_TOL_S:
                # Today's libmc plans the leg differently from the build that drove it (constants.h has
                # changed since - the 2026-09-27 wheel radius): the recorded duration is what happened.
                logger.info("leg %d: MC now plans %.2f s, the run recorded %.2f s: using a trapezoid of the "
                            "recorded duration", i + 1, profile[0][-1], duration)
                profile = None
            speed = plan["yaw_rate"] if kind == ROTATE else plan["cruise_speed"]
            t, s = profile if profile is not None else trapezoid_profile(amount, speed, duration)
        logger.debug("leg %d %s %.4f: %.2f s, status %s", i + 1, kind, amount, duration, status_t.get(i))
        legs.append(Leg(i, kind, amount, duration, status_t.get(i), t, s, acked_t.get(i, status_t.get(i + 1)),
                        leg.get("waits_ready", i == 0)))
    return Run(legs, plan["cruise_speed"], plan["yaw_rate"], end_t, plan.get("wheel_radius_m", LEGACY_WHEEL_RADIUS_M))


def ready_wait_s():
    """robot_link's grace period for READY before a plan's first leg (seq READY_WAIT_MS in constants.h),
    or None when the header cannot be read."""
    try:
        return _mc_client().constant("READY_WAIT_MS") / MS_PER_S
    except (ImportError, RuntimeError) as exc:   # McError is a RuntimeError
        logger.warning("READY_WAIT_MS unavailable (%s): firmware leg times not checked", exc)
        return None


def firmware_duration(leg, ready_wait):
    """Seconds from the leg's wire line to its K, or None when either end is unknown. robot_link sends the
    first leg of a call ready_wait after announcing it (earlier only if READY came in the meantime - a
    boot, which voids the run anyway) and every later leg the moment it announces it."""
    if leg.status_t is None or leg.ack_t is None or ready_wait is None:
        return None
    return leg.ack_t - leg.status_t - (ready_wait if leg.waits_ready else 0.0)


def cut_short(leg, ready_wait):
    """True when the firmware acked the leg well before its planned duration (FIRMWARE_SHORT_SHARE)."""
    took = firmware_duration(leg, ready_wait)
    return took is not None and leg.duration_s > 0.0 and took < FIRMWARE_SHORT_SHARE * leg.duration_s


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
