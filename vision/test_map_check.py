"""Offline tests of scan_match and map_check - no camera, no robot, no model. Run: python test_map_check.py

scan_match: a synthetic room seen from a stop whose pose carries a known error is laid back onto the room
seen from the start; a corridor, an empty map and an error past the search window are not trusted.
map_check: the obstacle distance of two stops, splitting a run into stops (a top-up leg, turn reversals),
the closure report, the truth report, and on a rendered floor (test_drive_map's) the keyframe phases of
a turn and the first / last frame closure.
"""
import json
import math
import os
import sys
import tempfile
from types import SimpleNamespace

import cv2
import numpy as np

import drive_map
import drive_timeline
import map_check
import scan_match
from frame_motion import Motion
from leg_odometry import RAW_DIR, RAW_PATTERN
from occupancy_map import MIN_HITS_PER_CELL, OccupancyMap
from test_drive_map import FPS, INTR, MOUNT, _render_floor, _texture

POINT_STEP_M = 0.01                 # wall points: dense enough for MIN_HITS_PER_CELL per cell
FREE_STEP_M = 0.03
ROOM_X0, ROOM_X, ROOM_Y0, ROOM_Y1 = -0.5, 3.0, -1.0, 1.2   # side walls from ROOM_X0 to the back wall at ROOM_X
BOX = (1.5, 1.9, 0.3, 0.6)                       # x0, x1, y0, y1 of a box in it
FREE_X0 = -0.3                                   # the floor seen free starts here ...
FREE_MARGIN_M = 0.1                              # ... and stays this far from the walls
BOX_CLEAR_M = 0.05                               # and this far from the box
START = (0.0, 0.0, 0.0)                          # the first stop: the world origin
STOP_B = (0.8, 0.0, math.pi)                     # the second stop: 0.8 m ahead, turned around
FULL_WEIGHT = 1.0                                # one map_builder capture's evidence: one view decides a cell
CORRIDOR_ERROR = (0.1, 0.0, 0.0)                 # along the corridor: nothing pins it
TURNED_AROUND = (0.0, 0.0, math.pi)              # check_closure: looking the other way, no floor in common
# (dx, dy m, dtheta rad) pose errors of the second stop; all at least a cell (scan_match's resolution).
POSE_ERRORS = ((0.0, 0.0, 0.0), (0.07, -0.04, math.radians(3.0)), (-0.12, 0.05, math.radians(-5.5)),
               (0.0, 0.15, math.radians(2.0)))
MATCH_TOL = (0.035, 0.035, math.radians(0.5))    # both maps snap walls to cell centres: up to 3 cm seen
GHOST_BEFORE_MIN, GHOST_AFTER_MAX = 0.25, 0.05   # check_gap: ghost share of the shifted stop, before / after the match
OUTSIDE_ERROR = (0.45, 0.0, 0.0)                 # past WINDOW_M
GAP_SHIFT_M = 0.10
TURN_DEG, TURN_FRAMES = 30.0, 60                 # check_repeat_turn: a turn rendered frame by frame
STILL_FRAMES = 8
REPEAT_TOL_DEG = 0.2
CLOSURE_TRUTH = (0.02, -0.01, math.radians(2.0))
CLOSURE_TOL = (0.005, 0.005, math.radians(0.1))
# check_find_stops: (kind, amount, start s, duration s) of a run TIMELINE_S long - turn, forward, top-up,
# turn left, turn right. A FORWARD leg adds its amount to x when it ends.
TIMELINE_S = 30.0
TIMELINE = ((drive_timeline.ROTATE, math.radians(90.0), 1.0, 4.0), (drive_timeline.FORWARD, 0.8, 6.0, 4.0),
            (drive_timeline.FORWARD, 0.03, 12.0, 0.5), (drive_timeline.ROTATE, math.radians(90.0), 14.0, 4.0),
            (drive_timeline.ROTATE, math.radians(-90.0), 20.0, 4.0))
# check_reports
TURN_OVERREAD = 1.06                             # the odometry reads a whole turn this long
TURN_SCALE, SCALED_CLOSURE_DEG = 0.9, 10.0       # a vo_turn_scale, and a closure turn at it
PERFECT_CLOSURE = Motion(0.0, 0.0, 0.0, 100, 100, 0.5)
END_MAP = (0.83, 0.02, math.pi)                  # the map's end pose ...
END_TAPE = {"x": 0.8, "y": 0.0, "theta_deg": 180.0}   # ... and the tape's: 3.6 cm apart
LANDMARK_XY = (1.2, 0.4)


def _inverse(pose, xy):
    """World points xy (N, 2) in the body frame of pose."""
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    rel = xy - (x, y)
    return np.column_stack((c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]))


def _room_walls():
    xs = np.arange(ROOM_X0, ROOM_X, POINT_STEP_M)
    ys = np.arange(ROOM_Y0, ROOM_Y1, POINT_STEP_M)
    bx, by = np.arange(BOX[0], BOX[1], POINT_STEP_M), np.arange(BOX[2], BOX[3], POINT_STEP_M)
    parts = [np.column_stack((xs, np.full_like(xs, ROOM_Y1))), np.column_stack((xs, np.full_like(xs, ROOM_Y0))),
             np.column_stack((np.full_like(ys, ROOM_X), ys)),
             np.column_stack((bx, np.full_like(bx, BOX[2]))), np.column_stack((bx, np.full_like(bx, BOX[3]))),
             np.column_stack((np.full_like(by, BOX[0]), by)), np.column_stack((np.full_like(by, BOX[1]), by))]
    return np.vstack(parts)


def _room_free():
    grid = np.array([(x, y) for x in np.arange(FREE_X0, ROOM_X - FREE_MARGIN_M, FREE_STEP_M)
                     for y in np.arange(ROOM_Y0 + FREE_MARGIN_M, ROOM_Y1 - FREE_MARGIN_M, FREE_STEP_M)])
    near_box = ((grid[:, 0] > BOX[0] - BOX_CLEAR_M) & (grid[:, 0] < BOX[1] + BOX_CLEAR_M)
                & (grid[:, 1] > BOX[2] - BOX_CLEAR_M) & (grid[:, 1] < BOX[3] + BOX_CLEAR_M))
    return grid[~near_box]


def _room_map(true_pose, used_pose, walls=None, free=None):
    """The room as a stop at true_pose sees it, placed from used_pose (its pose with an error)."""
    occ = OccupancyMap()
    walls = _room_walls() if walls is None else walls
    free = _room_free() if free is None else free
    occ.integrate_floor(_inverse(true_pose, free), _inverse(true_pose, walls), used_pose, weight=FULL_WEIGHT)
    return occ


def _with_error(pose, error):
    return pose[0] + error[0], pose[1] + error[1], pose[2] + error[2]


def check_scan_match():
    """The second stop's map, built from its pose with POSE_ERRORS, is laid back: the pose corrected by
    the match is the true one within MATCH_TOL, and the match is trusted."""
    errors = []
    ref = _room_map(START, START)
    for error in POSE_ERRORS:
        used = _with_error(STOP_B, error)
        found = scan_match.match(ref, _room_map(STOP_B, used), used[:2])
        got = scan_match.corrected_pose(used, found)
        off = (got[0] - STOP_B[0], got[1] - STOP_B[1], math.atan2(math.sin(got[2] - STOP_B[2]), math.cos(got[2] - STOP_B[2])))
        if not found.trusted or any(abs(o) > tol for o, tol in zip(off, MATCH_TOL)):
            errors.append(f"error {error}: corrected pose off by {off[0]:+.3f} {off[1]:+.3f} m "
                          f"{math.degrees(off[2]):+.2f} deg, trusted {found.trusted} ({found.why})")
    return errors


def check_scan_match_untrusted():
    """A corridor (two parallel walls) is not pinned along it, a map of a few cells and an error past the
    window are not trusted either."""
    errors = []
    xs = np.arange(ROOM_X0, ROOM_X, POINT_STEP_M)
    corridor = np.vstack((np.column_stack((xs, np.full_like(xs, ROOM_Y1))), np.column_stack((xs, np.full_like(xs, ROOM_Y0)))))
    free = np.empty((0, 2))
    used = _with_error(STOP_B, CORRIDOR_ERROR)
    found = scan_match.match(_room_map(START, START, corridor, free),
                             _room_map(STOP_B, used, corridor, free), used[:2])
    if found.trusted or "dx" not in found.why:
        errors.append(f"corridor: trusted {found.trusted}, why '{found.why}' (want not pinned along dx)")
    few = OccupancyMap()
    few.integrate_floor(np.empty((0, 2)), np.array([LANDMARK_XY] * MIN_HITS_PER_CELL), START, weight=FULL_WEIGHT)
    found = scan_match.match(_room_map(START, START), few, START[:2])
    if found.trusted:
        errors.append("a one-cell map: trusted")
    used = _with_error(STOP_B, OUTSIDE_ERROR)
    found = scan_match.match(_room_map(START, START), _room_map(STOP_B, used), used[:2])
    if found.trusted:
        errors.append(f"error {OUTSIDE_ERROR} past the window: trusted ({found.dx:+.3f} {found.dy:+.3f} m)")
    return errors


def _stop(number, occ_map, pose=START):
    return map_check.Stop(number, 0, 1, pose, occ_map=occ_map)


def check_gap():
    """Two stops that saw the same room: no ghost, distance 0; one placed GAP_SHIFT_M off: a ghost share
    of at least GHOST_BEFORE_MIN that the match takes under GHOST_AFTER_MAX, and both verdicts FAIL."""
    errors = []
    a = _stop(1, _room_map(START, START))
    same = map_check.compare(a, _stop(2, _room_map(STOP_B, STOP_B), STOP_B))
    if same.gap[:2] != (0.0, 0.0):
        errors.append(f"same room: ghost share {same.gap[0]:.2f}, median {same.gap[1]:.3f} m, want 0")
    used = _with_error(STOP_B, (0.0, GAP_SHIFT_M, 0.0))
    shifted = map_check.compare(a, _stop(2, _room_map(STOP_B, used), used))
    if shifted.gap[0] < GHOST_BEFORE_MIN or shifted.gap_after[0] > GHOST_AFTER_MAX:
        errors.append(f"shifted {GAP_SHIFT_M} m: ghost share {shifted.gap[0]:.2f} before, {shifted.gap_after[0]:.2f} "
                      f"after the match (want >= {GHOST_BEFORE_MIN}, then <= {GHOST_AFTER_MAX})")
    verdicts = dict((name, ok) for name, ok, _ in map_check.pair_verdicts(shifted))
    if verdicts.get("stops 1-2 ghosts") is not False or verdicts.get("stops 1-2 pose") is not False:
        errors.append(f"shifted {GAP_SHIFT_M} m: verdicts {verdicts}, want both FAIL")
    return errors


def _fit(index, kind, amount, start, duration, span):
    leg = drive_timeline.Leg(index, kind, amount, duration, start, np.zeros(1), np.zeros(1), None, False)
    return drive_map.LegFit(leg, start, 1.0, span=span, firmware_s=duration)


def check_find_stops():
    """TIMELINE: two stops, the top-up joining the second; --sweeps splits the second where its turns
    reverse."""
    errors = []
    times = np.arange(0.0, TIMELINE_S, 1.0 / FPS)

    def frame(t):
        return int(np.searchsorted(times, t))

    poses = np.zeros((len(times), 3))
    fits = []
    for k, (kind, amount, start, duration) in enumerate(TIMELINE):
        end = frame(TIMELINE[k + 1][2]) if k + 1 < len(TIMELINE) else len(times)
        fits.append(_fit(k, kind, amount, start, duration, (frame(start), end)))
        if kind == drive_timeline.FORWARD:
            poses[frame(start + duration):, 0] += amount
    dm = SimpleNamespace(times=times, poses=poses, fits=fits)
    forward_end = TIMELINE[1][2] + TIMELINE[1][3]
    reverse_start = TIMELINE[4][2]
    stops = map_check.find_stops(dm)
    want = [(0, frame(TIMELINE[1][2])), (frame(forward_end + map_check.MOVE_TAIL_S), len(times))]
    if [(s.first, s.last) for s in stops] != want:
        errors.append(f"stops {[(s.first, s.last) for s in stops]}, want {want}")
    sweeps = map_check.find_stops(dm, sweeps=True)
    want = want[:1] + [(want[1][0], frame(reverse_start)), (frame(reverse_start), len(times))]
    if [(s.first, s.last) for s in sweeps] != want:
        errors.append(f"sweeps {[(s.first, s.last) for s in sweeps]}, want {want}")
    return errors


def check_reports():
    """closure_report: a whole turn the odometry read 6 % long gives its ratio and a FAIL; truth_report: the
    end pose and a landmark on an obstacle cell; load_truth refuses a landmark without its y."""
    errors = []
    dm = SimpleNamespace(poses=np.array([[0.0, 0.0, 0.0], [0.0, 0.0, TURN_OVERREAD * 2.0 * math.pi]]), vo_scale=1.0,
                         vo_turn_scale=1.0)
    lines, verdicts = map_check.closure_report(dm, PERFECT_CLOSURE)
    if f"odometry / true = {TURN_OVERREAD:.4f}" not in " ".join(lines) or verdicts[0][1] is not False:
        errors.append(f"closure of a turn read {TURN_OVERREAD} long: {lines}, {verdicts}")
    # The direct match is scaled like the poses: a floor fit reading SCALED_CLOSURE_DEG / TURN_SCALE is a
    # closure of SCALED_CLOSURE_DEG, where the (scaled) poses end.
    dm = SimpleNamespace(poses=np.array([[0.0, 0.0, 0.0], [0.0, 0.0, math.radians(SCALED_CLOSURE_DEG)]]), vo_scale=1.0,
                         vo_turn_scale=TURN_SCALE)
    direct = PERFECT_CLOSURE._replace(dtheta=math.radians(SCALED_CLOSURE_DEG / TURN_SCALE))
    _, verdicts = map_check.closure_report(dm, direct)
    if verdicts[0][1] is not True or "drift 0.000 m / +0.00 deg" not in verdicts[0][2]:
        errors.append(f"closure at vo_turn_scale {TURN_SCALE}: {verdicts}, want no drift")
    occ = OccupancyMap()
    occ.integrate_floor(np.empty((0, 2)), np.array([LANDMARK_XY] * MIN_HITS_PER_CELL), START, weight=FULL_WEIGHT)
    dm = SimpleNamespace(poses=np.array([[0.0, 0.0, 0.0], END_MAP]), occ_map=occ)
    truth = {"end": END_TAPE, "landmarks": [{"name": "leg", "x": LANDMARK_XY[0], "y": LANDMARK_XY[1]}]}
    _, verdicts = map_check.truth_report(dm, [_stop(1, occ)], truth)
    if [ok for _, ok, _ in verdicts] != [True, True]:
        errors.append(f"truth: {verdicts}, want the end within limits and the landmark found")
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "truth.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"end": None, "landmarks": [{"name": "leg", "x": 1.0}]}, f)
        try:
            map_check.load_truth(path)
            errors.append("a landmark without y: accepted")
        except ValueError:
            pass
    return errors


def _rendered_run(run_dir, poses):
    """raw/ frames of the textured floor from `poses`, and the DriveMap fields map_check reads."""
    texture = _texture()
    os.makedirs(os.path.join(run_dir, RAW_DIR))
    frames = []
    for seq, pose in enumerate(poses):
        cv2.imwrite(os.path.join(run_dir, RAW_DIR, RAW_PATTERN.format(seq=seq)), _render_floor(texture, pose))
        frames.append({"seq": seq})
    return SimpleNamespace(run_dir=run_dir, frames=frames, times=np.arange(len(poses)) / FPS, intrinsics=INTR, mount=MOUNT,
                           vo_scale=1.0, vo_turn_scale=1.0)


def check_repeat_turn():
    """A turn rendered frame by frame on the textured floor: every keyframe phase measures it within
    REPEAT_TOL_DEG."""
    errors = []
    still = [0.0] * STILL_FRAMES
    angles = still + list(np.linspace(0.0, math.radians(TURN_DEG), TURN_FRAMES)) + [math.radians(TURN_DEG)] * STILL_FRAMES
    with tempfile.TemporaryDirectory() as run_dir:
        dm = _rendered_run(run_dir, [(0.0, 0.0, a) for a in angles])
        fit = _fit(0, drive_timeline.ROTATE, math.radians(TURN_DEG), 0.0, 0.0, (STILL_FRAMES, len(angles)))
        phases = map_check.repeat_turn(dm, fit, {})
    got = [math.degrees(v) for v, _ in phases]
    if len(got) != map_check.REPEAT_PHASES or any(abs(g - TURN_DEG) > REPEAT_TOL_DEG for g in got):
        errors.append(f"phases {[f'{g:.2f}' for g in got]}, want {TURN_DEG} +- {REPEAT_TOL_DEG}")
    return errors


def check_closure():
    """closure() on two rendered frames CLOSURE_TRUTH apart returns that motion; two frames that share no
    floor return None."""
    errors = []
    with tempfile.TemporaryDirectory() as run_dir:
        motion = map_check.closure(_rendered_run(run_dir, [START, CLOSURE_TRUTH]))
    got = None if motion is None else (motion.dx, motion.dy, motion.dtheta)
    if got is None or any(abs(g - w) > tol for g, w, tol in zip(got, CLOSURE_TRUTH, CLOSURE_TOL)):
        errors.append(f"closure {got}, want {CLOSURE_TRUTH}")
    with tempfile.TemporaryDirectory() as run_dir:
        motion = map_check.closure(_rendered_run(run_dir, [START, TURNED_AROUND]))
    if motion is not None:
        errors.append(f"frames looking opposite ways matched: {motion}")
    return errors


def main():
    failed = 0
    for check in (check_scan_match, check_scan_match_untrusted, check_gap, check_find_stops, check_reports,
                  check_repeat_turn, check_closure):
        errors = check()
        print(f"{'PASS' if not errors else 'FAIL'} {check.__name__}")
        for e in errors:
            print("   ", e)
        failed += bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
