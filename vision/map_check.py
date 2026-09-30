"""How well the places a run looked from agree with each other - the multi-stop map test
(task 2026-09-29_multi-stop-map-test, plan step 2).

  python3 vision/map_check.py --run vision/output/<run> [--truth truth.json] [--sweeps] [--no-repeat]
  python3 vision/map_check.py --truth-template > truth.json

A stop: the frames between two FORWARD legs (before the first, after the last) - the robot stands at one
place and turns. The frames of a FORWARD leg in motion belong to no stop; a leg that moved the robot less
than STOP_MERGE_M (a top-up) does not split one. --sweeps also splits a stop wherever its turns reverse
(a quarter turn left, then right), to compare the sweeps of a scan made in one place.
Each stop's floor analyses go into a map of its own, with drive_map's rules and poses. Where two stops saw
the same wall the two maps must agree; where one stop's poses are off, its map is displaced by exactly that
error - a ghost - and scan_match measures it from the maps alone, without a tape measure.

check.txt:
  pairs    per pair of stops, scan_match's motion of the later map onto the earlier (the pose error between
           them, about the later stop's position) and, where both saw the floor, the distance from each
           obstacle cell of one to the nearest obstacle cell of the other, before and after that motion:
           the ghost share - cells with no partner within GHOST_DIST_M, 1.5 cells, so a cell's 8 neighbours
           count as its partner - and the median / p90. The median does not see a ghost: walls along the
           error's direction still coincide with themselves (a 10 cm shift of the test room: median 5 cm,
           ghost share 33 %);
  closure  the first and the last raw frame matched directly (ORB, refined by Lucas-Kanade, frame_motion's
           floor fit), against the end pose the odometry integrated: when the run ends where it started, the
           drift of the whole run - and, after whole turns, the true total turn against the odometry's;
  truth    (--truth) the end pose and each landmark against the tape measure (--truth-template);
  VO       every turn of at least REPEAT_MIN_TURN_DEG: commanded, drive_map's and the leg executor's
           measure, and the turn re-measured with REPEAT_PHASES keyframe phases (their spread is the
           odometry's repeatability); the turn axis (drive_map.rotation_axis).
Each check ends PASS / FAIL against the task's expected output 3, or n/a when it could not be made.
stops.png: every stop's obstacle cells in its own color over the whole map, black where two or more stops
agree, dark grey where only the frames of a FORWARD leg saw one; two parallel colored lines are a ghost.
stops.json: the same numbers for tools.

drive_map.build is run first (its motion.npz is reused); the camera overrides are drive_map's.
"""
import argparse
import itertools
import json
import logging
import math
import os
import sys
from dataclasses import dataclass, field

import cv2
import numpy as np

import drive_map
import drive_timeline
import scan_match
from drawing import MAP_FREE, MAP_UNKNOWN, map_px, put_text, view_window
from frame_motion import estimate_motion, floor_rows, track
from leg_odometry import ODOMETRY_STRIDE, RAW_DIR, RAW_PATTERN
from occupancy_map import OccupancyMap

CHECK_FILE, STOPS_IMAGE, STOPS_JSON = "check.txt", "stops.png", "stops.json"
RUN_FILE = "run.json"
LEGACY_TURN_SCALE = 1.0        # a run.json without vo_turn_scale: its legs were measured before 2026-09-29
STOP_MERGE_M = 0.15            # a FORWARD leg moving the robot less than this (a top-up) does not split a stop
MOVE_TAIL_S = 0.5              # a FORWARD leg is in motion from its aligned start to its duration plus this
STOP_MIN_FRAMES = 3
REPEAT_PHASES = ODOMETRY_STRIDE
REPEAT_MIN_TURN_DEG = 30.0     # smaller turns are top-ups: too short to show the odometry's repeatability
QUARTER_TURN_DEG = 90.0        # repeatability is reported per quarter turn
COMMON_MIN_CELLS = 10          # obstacle cells both stops saw the floor around, for the distance check
GAP_CAP_M = 1.0                # an obstacle the other stop has nothing near counts as this far
GHOST_DIST_M = 0.075           # an obstacle cell with no obstacle cell of the other stop this close is a ghost
LANDMARK_SEARCH_M = 0.5        # nearest obstacle cell to a landmark of the truth file, searched this far
CLOSURE_ORB_FEATURES = 1500
CLOSURE_MIN_INLIERS = 30       # a first / last frame match with fewer is not a closure
CLOSURE_LK_WIN_PX = 21
CLOSURE_LK_LEVELS = 0          # the ORB match is Lucas-Kanade's starting point: no pyramid needed
CLOSURE_FB_MAX_PX = 0.7
# Pass thresholds: the task's expected output 3 (user 2026-09-29). The ghost share replaced the median obstacle
# distance (module docstring): the sweeps of run 20260929_184936, a scan the user passed, show 12-16 %; the
# test room shifted 5 cm 4 %, 10 cm 33 %.
PASS_OFFSET_M, PASS_OFFSET_DEG = 0.05, 1.5
PASS_GHOST_SHARE = 0.20
PASS_END_M, PASS_END_DEG = 0.10, 3.0
PASS_LANDMARK_M = 0.10
PERCENTILE = 90
M_DIGITS, DEG_DIGITS, SHARE_DIGITS = 3, 2, 2
IMAGE_PX = 900
STOP_COLORS = ((230, 110, 0), (0, 0, 220), (0, 170, 0), (0, 140, 255), (200, 0, 200), (170, 170, 0))   # BGR
AGREE_COLOR = (0, 0, 0)
DRIVE_ONLY_COLOR = (90, 90, 90)
TRACK_COLOR = (160, 160, 160)
TEXT_COLOR = (255, 255, 255)
TEXT_SCALE = 0.45
LEGEND_X, LEGEND_Y0, LEGEND_DY = 8, 18, 18
STOP_DOT_PX, ARROW_M, ARROW_TIP, ARROW_LINE_PX = 5, 0.3, 0.3, 2
TRACK_LINE_PX = 1
LABEL_DX_PX = 7
NO_VALUE = "-"
TRUTH_TEMPLATE = {
    "frame": "robot centre where the run started: x forward, y left (m), theta CCW (deg)",
    "end": {"x": 0.0, "y": 0.0, "theta_deg": 0.0},
    "landmarks": [{"name": "front-left leg of the chair, the side facing the robot", "x": 1.2, "y": 0.4}],
}

logger = logging.getLogger(__name__)


@dataclass
class Stop:
    """Frames first .. last - 1 of a run, at one place; pose: the robot's at `first`; occ_map: their floor
    analyses (floors of them used)."""
    number: int
    first: int
    last: int
    pose: tuple
    turned: float = 0.0
    floors: int = 0
    occ_map: OccupancyMap = field(default=None, repr=False)


@dataclass
class PairCheck:
    """Stop b's map against stop a's: scan_match's Match, and the obstacle distances over `common` cells -
    (ghost share, median m, p90 m) - before and after moving b by the Match."""
    a: int
    b: int
    found: scan_match.Match
    common: int
    gap: tuple
    gap_after: tuple


def _within(value, limit, digits):
    """|value| within limit at the precision check.txt prints it: a one-cell distance (0.0500001 in float32)
    meets a 0.05 m limit."""
    return round(abs(value), digits) <= limit


def moving_frames(dm):
    """bool per frame: a FORWARD leg in motion - its aligned start to its duration (the firmware's, when that
    was longer) plus MOVE_TAIL_S."""
    moving = np.zeros(len(dm.times), bool)
    for fit in dm.fits:
        if fit.leg.kind != drive_timeline.FORWARD:
            continue
        took = max(fit.leg.duration_s, fit.firmware_s) if math.isfinite(fit.firmware_s) else fit.leg.duration_s
        moving |= (dm.times >= fit.start) & (dm.times < fit.start + took + MOVE_TAIL_S)
    return moving


def _runs(mask):
    """(first, last) of every run of True in mask, last exclusive."""
    edges = np.flatnonzero(np.diff(np.concatenate(([0], mask.astype(np.int8), [0]))))
    return list(zip(edges[::2].tolist(), edges[1::2].tolist()))


def sweep_starts(dm):
    """Frame indices where a turn of at least REPEAT_MIN_TURN_DEG starts turning the other way from the
    previous one."""
    starts, previous = [], 0.0
    for fit in dm.fits:
        if fit.leg.kind != drive_timeline.ROTATE or abs(math.degrees(fit.leg.amount)) < REPEAT_MIN_TURN_DEG:
            continue
        if previous and math.copysign(1.0, fit.leg.amount) != math.copysign(1.0, previous):
            starts.append(fit.span[0])
        previous = fit.leg.amount
    return starts


def find_stops(dm, sweeps=False):
    """The stops of a run (module docstring), numbered from 1, maps not built yet."""
    spans = []
    for first, last in _runs(~moving_frames(dm)):
        moved = math.hypot(*(dm.poses[first][:2] - dm.poses[spans[-1][1] - 1][:2])) if spans else math.inf
        if moved < STOP_MERGE_M:
            spans[-1] = (spans[-1][0], last)
        else:
            spans.append((first, last))
    if sweeps:
        cuts = sweep_starts(dm)
        spans = [piece for first, last in spans
                 for piece in zip([first] + [c for c in cuts if first < c < last], [c for c in cuts if first < c < last] + [last])]
    spans = [(first, last) for first, last in spans if last - first >= STOP_MIN_FRAMES]
    logger.debug("stops (frames): %s", spans)
    return [Stop(n + 1, first, last, tuple(dm.poses[first]), float(dm.poses[last - 1][2] - dm.poses[first][2]))
            for n, (first, last) in enumerate(spans)]


def build_stop_maps(dm, stops):
    """Each stop's floor analyses into its own map, with drive_map's rules and poses."""
    seqs = [fr["seq"] for fr in dm.frames]
    lines = drive_map.floor_lines(dm.run_dir)
    for stop in stops:
        stop.occ_map = OccupancyMap()
        lo, hi = seqs[stop.first], seqs[stop.last - 1]
        for line in lines:
            if not lo <= line["seq"] <= hi or not drive_map.floor_usable(line, dm.pose_of, dm.rate_of):
                continue
            labels = drive_map.floor_labels(dm.run_dir, line)
            if labels is not None:
                drive_map.integrate_floor_line(stop.occ_map, labels, line, dm.pose_of[line["seq"]], dm.intrinsics, dm.mount)
                stop.floors += 1


def _known(occ_map):
    return occ_map.free() | occ_map.occupied()


def _stats(distances):
    """(ghost share, median, p90) of distances, NaN when there are too few."""
    if len(distances) < COMMON_MIN_CELLS:
        return math.nan, math.nan, math.nan
    return (float(np.mean(distances > GHOST_DIST_M)), float(np.median(distances)),
            float(np.percentile(distances, PERCENTILE)))


def compare(a, b):
    """PairCheck of stop b's map against stop a's."""
    found = scan_match.match(a.occ_map, b.occ_map, b.pose[:2])
    common = _known(a.occ_map) & _known(b.occ_map)
    pts_a = scan_match.cell_points(a.occ_map, a.occ_map.occupied() & common)
    pts_b = scan_match.cell_points(b.occ_map, b.occ_map.occupied() & common)
    field_a = np.minimum(scan_match.distance_field(a.occ_map), GAP_CAP_M)
    field_b = np.minimum(scan_match.distance_field(b.occ_map), GAP_CAP_M)

    def gap(moved_b, moved_a):
        return _stats(np.concatenate((scan_match.sample(field_a, a.occ_map, moved_b, GAP_CAP_M),
                                      scan_match.sample(field_b, b.occ_map, moved_a, GAP_CAP_M))))

    before = gap(pts_b, pts_a)
    # b moved by the match onto a; a moved by its inverse onto b.
    after = gap(scan_match.transform(pts_b, found.dx, found.dy, found.dtheta, found.pivot),
                scan_match.transform(pts_a - (found.dx, found.dy), 0.0, 0.0, -found.dtheta, found.pivot))
    logger.debug("stops %d-%d: %d + %d common obstacle cells, ghost share %.2f -> %.2f", a.number, b.number,
                 len(pts_a), len(pts_b), before[0], after[0])
    return PairCheck(a.number, b.number, found, len(pts_a) + len(pts_b), before, after)


def pair_verdicts(pair):
    """Verdicts of a PairCheck: the pose error between the stops, and the obstacle distance."""
    f = pair.found
    name = f"stops {pair.a}-{pair.b}"
    offset = math.hypot(f.dx, f.dy)
    if f.trusted:
        ok = _within(offset, PASS_OFFSET_M, M_DIGITS) and _within(math.degrees(f.dtheta), PASS_OFFSET_DEG, DEG_DIGITS)
        pose = (name + " pose", ok, f"{offset:.{M_DIGITS}f} m / {math.degrees(f.dtheta):+.{DEG_DIGITS}f} deg "
                f"(limit {PASS_OFFSET_M} m / {PASS_OFFSET_DEG} deg)")
    else:
        pose = (name + " pose", None, f"scan_match not trusted: {f.why}")
    if math.isnan(pair.gap[0]):
        gap = (name + " ghosts", None, f"only {pair.common} obstacle cells where both saw the floor")
    else:
        gap = (name + " ghosts", _within(pair.gap[0], PASS_GHOST_SHARE, SHARE_DIGITS),
               f"{pair.gap[0]:.0%} of {pair.common} obstacle cells unmatched within {GHOST_DIST_M} m "
               f"(limit {PASS_GHOST_SHARE:.0%})")
    return [pose, gap]


def pair_lines(pair):
    """check.txt's two lines of a PairCheck."""
    f = pair.found
    return [f"stops {pair.a} -> {pair.b}: scan_match {f.dx:+.{M_DIGITS}f} {f.dy:+.{M_DIGITS}f} m "
            f"{math.degrees(f.dtheta):+.{DEG_DIGITS}f} deg about ({f.pivot[0]:+.2f}, {f.pivot[1]:+.2f}), "
            f"{'trusted' if f.trusted else 'NOT trusted: ' + f.why}; spread {f.spread[0]:.2f} / {f.spread[1]:.2f} m / "
            f"{math.degrees(f.spread[2]):.1f} deg; score {f.score:.3f} (grid median {f.base:.3f}), {f.points} cells",
            f"    obstacle cells both saw around: {pair.common}; ghost share / median / p{PERCENTILE} "
            f"{pair.gap[0]:.0%} / {pair.gap[1]:.{M_DIGITS}f} / {pair.gap[2]:.{M_DIGITS}f} m before, "
            f"{pair.gap_after[0]:.0%} / {pair.gap_after[1]:.{M_DIGITS}f} / {pair.gap_after[2]:.{M_DIGITS}f} m after the match"]


def _floor_mask(dm, shape):
    top, bottom = floor_rows(dm.intrinsics, dm.mount, shape[0])
    mask = np.zeros(shape, np.uint8)
    mask[top:bottom] = 255
    return mask


def _gray(dm, k):
    return cv2.imread(os.path.join(dm.run_dir, RAW_DIR, RAW_PATTERN.format(seq=dm.frames[k]["seq"])), cv2.IMREAD_GRAYSCALE)


def closure(dm):
    """frame_motion.Motion of the robot at the last raw frame in the body frame of the first, matched
    directly (ORB on the floor, refined by Lucas-Kanade), or None when the two share too little floor."""
    first, last = _gray(dm, 0), _gray(dm, len(dm.frames) - 1)
    if first is None or last is None:
        return None
    mask = _floor_mask(dm, first.shape)
    orb = cv2.ORB_create(CLOSURE_ORB_FEATURES)
    kp0, des0 = orb.detectAndCompute(first, mask)
    kp1, des1 = orb.detectAndCompute(last, mask)
    if des0 is None or des1 is None:
        return None
    matches = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True).match(des0, des1)
    if len(matches) < CLOSURE_MIN_INLIERS:
        return None
    p0 = np.float32([kp0[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    guess = np.float32([kp1[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
    lk = dict(winSize=(CLOSURE_LK_WIN_PX, CLOSURE_LK_WIN_PX), maxLevel=CLOSURE_LK_LEVELS)
    p1, st1, _ = cv2.calcOpticalFlowPyrLK(first, last, p0, guess.copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **lk)
    back, st2, _ = cv2.calcOpticalFlowPyrLK(last, first, p1, p0.copy(), flags=cv2.OPTFLOW_USE_INITIAL_FLOW, **lk)
    good = (st1.ravel() == 1) & (st2.ravel() == 1) & (np.linalg.norm((back - p0).reshape(-1, 2), axis=1) < CLOSURE_FB_MAX_PX)
    motion = estimate_motion(p0.reshape(-1, 2)[good].astype(np.float64), p1.reshape(-1, 2)[good].astype(np.float64),
                             dm.intrinsics, dm.mount, np.random.default_rng(0))
    logger.debug("closure: %d ORB matches, %d kept by Lucas-Kanade, motion %s", len(matches), int(good.sum()), motion)
    if motion is None or motion.inliers < CLOSURE_MIN_INLIERS:
        return None
    return motion


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def closure_report(dm, direct):
    """(lines, verdicts) of the first / last frame match against the integrated end pose."""
    if direct is None:
        return (["closure: the first and last frames share too little floor to match (the run does not end where it "
                 "started, or the floor is bare)"], [("closure", None, "first / last frames do not match")])
    x, y, theta = dm.poses[-1]
    # The direct match is a floor fit like the odometry's: the same scales as the poses.
    dx, dy, dtheta = direct.dx * dm.vo_scale, direct.dy * dm.vo_scale, direct.dtheta * dm.vo_turn_scale
    moved = math.hypot(x - dx, y - dy)
    turns = round((theta - dtheta) / (2.0 * math.pi))
    true_turn = dtheta + 2.0 * math.pi * turns
    drift = theta - true_turn
    lines = [f"closure: first -> last frame directly: {dx:+.{M_DIGITS}f} {dy:+.{M_DIGITS}f} m "
             f"{math.degrees(dtheta):+.{DEG_DIGITS}f} deg ({direct.inliers} inliers, rms {direct.rms_px:.2f} px); "
             f"integrated end pose {x:+.{M_DIGITS}f} {y:+.{M_DIGITS}f} m {math.degrees(theta):+.{DEG_DIGITS}f} deg "
             f"-> drift {moved:.{M_DIGITS}f} m / {math.degrees(drift):+.{DEG_DIGITS}f} deg"]
    if turns:
        lines.append(f"    whole turns: {turns}; the odometry turned {math.degrees(theta):+.1f} deg, the robot "
                     f"{math.degrees(true_turn):+.1f} deg -> odometry / true = {theta / true_turn:.4f}")
    ok = _within(moved, PASS_END_M, M_DIGITS) and _within(math.degrees(drift), PASS_END_DEG, DEG_DIGITS)
    return lines, [("closure", ok, f"drift {moved:.{M_DIGITS}f} m / {math.degrees(drift):+.{DEG_DIGITS}f} deg "
                                   f"(limit {PASS_END_M} m / {PASS_END_DEG} deg)")]


def load_truth(path):
    """The truth file (TRUTH_TEMPLATE's layout); ValueError naming what is missing."""
    with open(path, encoding="utf-8") as f:
        truth = json.load(f)
    end = truth.get("end")
    if end is not None and not all(k in end for k in ("x", "y", "theta_deg")):
        raise ValueError(f"{path}: 'end' needs x, y, theta_deg (or null)")
    for lm in truth.get("landmarks", []):
        if not all(k in lm for k in ("name", "x", "y")):
            raise ValueError(f"{path}: every landmark needs name, x, y")
    return truth


def _nearest_obstacle(points, xy):
    """Distance from xy to the nearest of points (occupied cell centres) within LANDMARK_SEARCH_M, or None."""
    if not len(points):
        return None
    d = float(np.min(np.hypot(points[:, 0] - xy[0], points[:, 1] - xy[1])))
    return d if d <= LANDMARK_SEARCH_M else None


def truth_report(dm, stops, truth):
    """(lines, verdicts) of the end pose and the landmarks against the tape measure."""
    whole_points = scan_match.occupied_points(dm.occ_map)
    stop_points = [(s.number, scan_match.occupied_points(s.occ_map)) for s in stops]

    def distance_text(points, xy):
        d = _nearest_obstacle(points, xy)
        return NO_VALUE if d is None else f"{d:.{M_DIGITS}f}"

    lines, verdicts = [], []
    end = truth.get("end")
    if end is not None:
        x, y, theta = dm.poses[-1]
        off = math.hypot(x - end["x"], y - end["y"])
        turn = math.degrees(_wrap(theta - math.radians(end["theta_deg"])))
        lines.append(f"truth end: tape {end['x']:+.{M_DIGITS}f} {end['y']:+.{M_DIGITS}f} m {end['theta_deg']:+.1f} deg, "
                     f"map {x:+.{M_DIGITS}f} {y:+.{M_DIGITS}f} m {math.degrees(theta):+.1f} deg -> "
                     f"{off:.{M_DIGITS}f} m / {turn:+.{DEG_DIGITS}f} deg")
        verdicts.append(("truth end", _within(off, PASS_END_M, M_DIGITS) and _within(turn, PASS_END_DEG, DEG_DIGITS),
                         f"{off:.{M_DIGITS}f} m / {turn:+.{DEG_DIGITS}f} deg (limit {PASS_END_M} m / {PASS_END_DEG} deg)"))
    for lm in truth.get("landmarks", []):
        whole = _nearest_obstacle(whole_points, (lm["x"], lm["y"]))
        per_stop = ", ".join(f"stop {number} {distance_text(points, (lm['x'], lm['y']))}" for number, points in stop_points)
        lines.append(f"truth landmark '{lm['name']}' at {lm['x']:+.2f} {lm['y']:+.2f} m: nearest obstacle cell "
                     + ("none" if whole is None else f"{whole:.{M_DIGITS}f} m") + f" ({per_stop})")
        verdicts.append((f"landmark {lm['name']}", None if whole is None else _within(whole, PASS_LANDMARK_M, M_DIGITS),
                         f"none within {LANDMARK_SEARCH_M} m" if whole is None
                         else f"{whole:.{M_DIGITS}f} m (limit {PASS_LANDMARK_M} m)"))
    return lines, verdicts


def repeat_turn(dm, fit, cache):
    """The turn of `fit` measured REPEAT_PHASES times, each with its keyframes shifted by one frame, over the
    same frames (from ODOMETRY_STRIDE before the leg's span to its end): [(dtheta rad at dm.vo_turn_scale,
    filled pairs)], [] when
    none of its frames can be read. A keyframe pair that does not match is measured frame by frame, as
    leg_odometry.measure_motion does; a frame pair that still fails (a blurred frame) takes the mean of the
    others of its keyframe pair, so that every phase covers the same turn."""
    start, end = max(fit.span[0] - ODOMETRY_STRIDE, 0), fit.span[1] - 1
    for k in range(start, end + 1):
        if k not in cache:
            cache[k] = _gray(dm, k)
    shape = next((cache[k].shape for k in range(start, end + 1) if cache[k] is not None), None)
    if shape is None:
        return []
    mask = _floor_mask(dm, shape)
    rng = np.random.default_rng(0)

    def turn(k0, k1):
        if cache[k0] is None or cache[k1] is None or dm.times[k1] <= dm.times[k0]:
            return None
        motion = estimate_motion(*track(cache[k0], cache[k1], mask), dm.intrinsics, dm.mount, rng)
        return None if motion is None else motion.dtheta

    results = []
    for phase in range(REPEAT_PHASES):
        keys = sorted({start, end, *range(start + phase, end, ODOMETRY_STRIDE)})
        total, filled = 0.0, 0
        for k0, k1 in zip(keys, keys[1:]):
            step = turn(k0, k1)
            if step is None:
                steps = [turn(k, k + 1) for k in range(k0, k1)]
                known = [s for s in steps if s is not None]
                filled += len(steps) - len(known)
                step = float(np.mean(known)) * len(steps) if known else 0.0
            total += step
        results.append((total * dm.vo_turn_scale, filled))
    return results


def vo_report(dm, executed, repeat, executed_turn_scale=LEGACY_TURN_SCALE):
    """(lines, spread per quarter turn in deg or None) of the turns of the run. executed_turn_scale: the
    vo_turn_scale the leg executor measured its legs at (run.json) - its column is brought to
    dm.vo_turn_scale, so both columns read the same odometry at the same scale."""
    rescale = dm.vo_turn_scale / executed_turn_scale
    lines = ["VO turns (deg): leg  commanded  drive_map  executor" + ("  phases (spread)" if repeat else "")]
    spreads = []
    cache = {}
    for fit in dm.fits:
        if fit.leg.kind != drive_timeline.ROTATE or abs(math.degrees(fit.leg.amount)) < REPEAT_MIN_TURN_DEG:
            continue
        leg = executed[fit.leg.index] if fit.leg.index < len(executed) else {}
        mine = (f"{math.degrees(leg['measured'] * rescale):+9.{DEG_DIGITS}f}" if leg.get("measured") is not None
                else f"{NO_VALUE:>9}")
        text = (f"    {fit.leg.index + 1:3d}  {math.degrees(fit.leg.amount):+9.{DEG_DIGITS}f}  "
                f"{math.degrees(fit.measured):+9.{DEG_DIGITS}f}  {mine}")
        phases = repeat_turn(dm, fit, cache) if repeat else []
        if phases:
            values = [math.degrees(v) for v, _ in phases]
            spread = max(values) - min(values)
            spreads.append(spread * QUARTER_TURN_DEG / abs(math.degrees(fit.leg.amount)))
            text += "  " + " ".join(f"{v:+.{DEG_DIGITS}f}" for v in values) + f"  ({spread:.{DEG_DIGITS}f})"
            filled = sum(n for _, n in phases)
            if filled:
                text += f"  [{filled} unmatched frame pairs filled from their neighbours]"
        elif repeat:
            text += "  (no frame readable)"
        lines.append(text)
        cache.clear()
    per_quarter = float(np.mean(spreads)) if spreads else None
    if per_quarter is not None:
        lines.append(f"    repeatability: keyframe phases spread {per_quarter:.2f} deg per {QUARTER_TURN_DEG:.0f} deg "
                     "turn (mean)")
    lines.append(drive_map.axis_line(drive_map.rotation_axis(dm.motion), dm.mount))
    return lines, per_quarter


def draw_stops(dm, stops, pairs):
    """stops.png (module docstring)."""
    poses = [tuple(p) for p in dm.poses]
    window = view_window(dm.occ_map, poses)
    x0, y0, span = window
    centres = (np.arange(IMAGE_PX) + 0.5) / IMAGE_PX * span
    occ = dm.occ_map
    ix = np.floor((x0 + centres - occ.origin[0]) / occ.res).astype(int)
    iy = np.floor((y0 + span - centres - occ.origin[1]) / occ.res).astype(int)     # row 0 = top = max y
    inside = (iy[:, None] >= 0) & (iy[:, None] < occ.n) & (ix[None, :] >= 0) & (ix[None, :] < occ.n)
    rows, cols = np.clip(iy, 0, occ.n - 1)[:, None], np.clip(ix, 0, occ.n - 1)[None, :]

    def at_pixels(grid):
        return inside & grid[rows, cols]

    img = np.full((IMAGE_PX, IMAGE_PX, 3), MAP_UNKNOWN, np.uint8)
    img[at_pixels(occ.free())] = (MAP_FREE,) * 3
    img[at_pixels(occ.occupied())] = DRIVE_ONLY_COLOR
    count = np.zeros((IMAGE_PX, IMAGE_PX), np.int32)
    for stop in stops:
        seen = at_pixels(stop.occ_map.occupied())
        img[seen & (count == 0)] = STOP_COLORS[(stop.number - 1) % len(STOP_COLORS)]
        count += seen
    img[count >= 2] = AGREE_COLOR

    px = map_px(occ, [], IMAGE_PX, window)
    cv2.polylines(img, [np.array([px(p) for p in poses], np.int32)], False, TRACK_COLOR, TRACK_LINE_PX)
    for stop in stops:
        color = STOP_COLORS[(stop.number - 1) % len(STOP_COLORS)]
        x, y, theta = stop.pose
        cv2.circle(img, px((x, y)), STOP_DOT_PX, color, cv2.FILLED)
        cv2.arrowedLine(img, px((x, y)), px((x + ARROW_M * math.cos(theta), y + ARROW_M * math.sin(theta))), color,
                        ARROW_LINE_PX, tipLength=ARROW_TIP)
        u, v = px((x, y))
        put_text(img, str(stop.number), (u + LABEL_DX_PX, v - LABEL_DX_PX), color, TEXT_SCALE)
    legend = [f"run {os.path.basename(dm.run_dir)}: black = 2+ stops agree, grey = seen only while driving"]
    legend += [f"stop {s.number}: x {s.pose[0]:+.2f} y {s.pose[1]:+.2f} m, heading {math.degrees(s.pose[2]):+.0f} deg, "
               f"turned {math.degrees(s.turned):+.0f} deg, {s.floors} floor frames" for s in stops]
    legend += [f"{p.b} onto {p.a}: {p.found.dx:+.3f} {p.found.dy:+.3f} m {math.degrees(p.found.dtheta):+.1f} deg"
               + ("" if p.found.trusted else " (not trusted)") for p in pairs]
    for k, text in enumerate(legend):
        color = STOP_COLORS[(stops[k - 1].number - 1) % len(STOP_COLORS)] if 1 <= k <= len(stops) else TEXT_COLOR
        put_text(img, text, (LEGEND_X, LEGEND_Y0 + k * LEGEND_DY), color, TEXT_SCALE)
    put_text(img, f"{span:.1f} m across", (LEGEND_X, IMAGE_PX - LEGEND_X), TEXT_COLOR, TEXT_SCALE)
    return img


def _pair_json(p):
    f = p.found
    return {"a": p.a, "b": p.b, "dx": round(f.dx, M_DIGITS), "dy": round(f.dy, M_DIGITS),
            "dtheta_deg": round(math.degrees(f.dtheta), DEG_DIGITS), "pivot": [round(v, M_DIGITS) for v in f.pivot],
            "trusted": f.trusted, "why": f.why, "common_cells": p.common,
            "ghost_share_median_p90": [None if math.isnan(v) else round(v, M_DIGITS) for v in p.gap],
            "after_match": [None if math.isnan(v) else round(v, M_DIGITS) for v in p.gap_after]}


def _parse_args():
    parser = argparse.ArgumentParser(description="Check how well the stops of a demo_drive run agree on its map.")
    parser.add_argument("--run", help="vision/output/<run_id>")
    parser.add_argument("--truth", help="tape measurements (JSON, see --truth-template)")
    parser.add_argument("--truth-template", action="store_true", help="print a truth file to fill in, and exit")
    parser.add_argument("--sweeps", action="store_true", help="also split a stop where its turns reverse")
    parser.add_argument("--no-repeat", action="store_true", help="skip re-measuring the turns (the slow part)")
    parser.add_argument("--fx", type=float, help="override the recording's fx (fy follows)")
    parser.add_argument("--cam-pitch", type=float, help="override the recording's pitch, deg, + = down")
    parser.add_argument("--cam-forward", type=float, help="override the lens offset ahead of the robot centre, m")
    parser.add_argument("--cam-left", type=float, help="override the lens offset left of the robot centre, m")
    parser.add_argument("--vo-scale", type=float, help="real / odometry metres (default: drive_map's)")
    parser.add_argument("--vo-turn-scale", type=float, help="real / odometry turn (default: drive_map's)")
    parser.add_argument("--redo-motion", action="store_true", help="recompute drive_map's motion.npz")
    args = parser.parse_args()
    if not args.truth_template and not args.run:
        parser.error("--run is required")
    return args


def main():
    args = _parse_args()
    if args.truth_template:
        print(json.dumps(TRUTH_TEMPLATE, indent=2, ensure_ascii=False))
        return 0
    run_dir = args.run.rstrip("/")
    truth = None
    if args.truth:
        try:
            truth = load_truth(args.truth)
        except (OSError, ValueError) as exc:
            raise SystemExit(f"truth file: {exc}") from exc
    recording, intrinsics, mount = drive_map.load_camera(run_dir, args.fx, args.cam_pitch, args.cam_forward, args.cam_left)
    try:
        dm = drive_map.build(run_dir, recording, intrinsics, mount, None, args.redo_motion, args.vo_scale,
                             args.vo_turn_scale)
    except FileNotFoundError as exc:
        raise SystemExit(str(exc)) from exc
    with open(os.path.join(run_dir, RUN_FILE), encoding="utf-8") as f:
        run_data = json.load(f)
    executed = run_data.get("executed_legs") or []
    executed_turn_scale = run_data.get("vo_turn_scale", LEGACY_TURN_SCALE)

    stops = find_stops(dm, args.sweeps)
    build_stop_maps(dm, stops)
    pairs = [compare(a, b) for a, b in itertools.combinations(stops, 2)]
    lines = [f"run {os.path.basename(run_dir)}   fx {intrinsics.fx:.1f}  pitch {mount.pitch_deg:+.2f} deg  forward "
             f"{mount.forward_m:.3f} m  left {mount.left_m:+.3f} m  vo_scale {dm.vo_scale:.3f}  "
             f"vo_turn_scale {dm.vo_turn_scale:.4f}",
             f"stops: {len(stops)}" + (" (split at turn reversals)" if args.sweeps else "")]
    lines += [f"  stop {s.number}: frames {s.first}-{s.last - 1}, x {s.pose[0]:+.{M_DIGITS}f} y {s.pose[1]:+.{M_DIGITS}f} m "
              f"heading {math.degrees(s.pose[2]):+.1f} deg, turned {math.degrees(s.turned):+.1f} deg, {s.floors} floor "
              f"frames, {int(s.occ_map.occupied().sum())} obstacle cells" for s in stops]
    verdicts = []
    lines.append("")
    if not pairs:
        lines.append("pairs: one stop, nothing to compare (--sweeps compares the sweeps of a scan)")
    for pair in pairs:
        lines += pair_lines(pair)
        verdicts += pair_verdicts(pair)
    lines.append("")
    closure_lines, closure_verdicts = closure_report(dm, closure(dm))
    lines += closure_lines
    verdicts += closure_verdicts
    if truth is not None:
        truth_lines, truth_verdicts = truth_report(dm, stops, truth)
        lines += [""] + truth_lines
        verdicts += truth_verdicts
    vo_lines, per_quarter = vo_report(dm, executed, not args.no_repeat, executed_turn_scale)
    lines += [""] + vo_lines
    lines += ["", "verdicts:"] + [f"  {'n/a ' if ok is None else 'PASS' if ok else 'FAIL'}  {name}: {text}"
                                  for name, ok, text in verdicts]

    out_dir = os.path.join(run_dir, drive_map.MAP_DIR)
    with open(os.path.join(out_dir, CHECK_FILE), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    if not cv2.imwrite(os.path.join(out_dir, STOPS_IMAGE), draw_stops(dm, stops, pairs)):
        raise OSError(f"cannot write {os.path.join(out_dir, STOPS_IMAGE)}")
    with open(os.path.join(out_dir, STOPS_JSON), "w", encoding="utf-8") as f:
        json.dump({"run": os.path.basename(run_dir), "sweeps": args.sweeps,
                   "stops": [{"number": s.number, "frames": [s.first, s.last - 1],
                              "pose": [round(s.pose[0], M_DIGITS), round(s.pose[1], M_DIGITS),
                                       round(math.degrees(s.pose[2]), DEG_DIGITS)],
                              "turned_deg": round(math.degrees(s.turned), DEG_DIGITS), "floor_frames": s.floors,
                              "obstacle_cells": int(s.occ_map.occupied().sum())} for s in stops],
                   "pairs": [_pair_json(p) for p in pairs],
                   "turn_repeatability_deg_per_quarter": None if per_quarter is None else round(per_quarter, DEG_DIGITS),
                   "verdicts": [{"check": name, "pass": ok, "text": text} for name, ok, text in verdicts]}, f, indent=2)
    print("\n".join(lines))
    print(f"\n-> {os.path.join(out_dir, CHECK_FILE)}, {os.path.join(out_dir, STOPS_IMAGE)}")
    return 0


if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    sys.exit(main())
