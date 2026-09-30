"""How the robot really moved, from a recording's raw frames: floor visual odometry over a whole run
(drive_map, after the fact) or over one leg while the recorder is still writing (communication/
leg_executor, right after the leg's K) - task 2026-09-27_explore-map Pha 1' B. The user trusts the images
over the wheels for all motion (2026-09-28): the wheels slip, and the firmware has cut legs short.

  measure_motion  frame_motion across raw frames ODOMETRY_STRIDE apart, one row per frame (COL_*)
  odometry_track  those rows chained into a heading and a position
  measure_leg     the robot centre's motion between two wall times, in its body frame at the first:
                  LegMotion (dx, dy at the tape scale, dtheta, how much of the span was measured)

A span whose pairs cover less than VO_MIN_PAIRS of its time is weak: a camera lost mid-leg (the USB hub
reset of 2026-09-28) or a floor without texture. Its motion is what was seen, which is short of the truth.
"""
import collections
import json
import logging
import os
import time

import cv2
import numpy as np

from frame_motion import estimate_motion, floor_rows, track

RAW_DIR, RAW_INDEX = "raw", "index.jsonl"      # raw_recorder's layout
RAW_PATTERN = "{seq:06d}.jpg"
ODOMETRY_STRIDE = 5                    # frames between odometry measurements (~2.3 deg apart at 0.3 rad/s)
LOG_EVERY_FRAMES = 100
# Columns of the per-frame odometry array (measure_motion).
COL_OK, COL_DX, COL_DY, COL_DTHETA, COL_INLIERS, COL_TRACKS, COL_RMS = range(7)
MOTION_COLUMNS = 7
# A span (a leg) whose measured pairs cover less of its time than this has weak odometry: kept out of the
# slip ratios, and a leg executor does not trust it. Run 20260928_221843 lost the camera 1.9 s into a
# 4.3 s leg: every recorded frame measured, 12 of 34 cm seen.
VO_MIN_PAIRS = 0.75
WAIT_POLL_S = 0.05                     # measure_leg: how often to look for the frame that closes the span
TAIL_BYTES = 4096                      # ... in this much of the index's end (an index line is ~110 bytes)

# dx, dy (m, at the scale asked for), dtheta (rad): the centre at t1 in its body frame at t0 (frame_motion's
# convention). pairs: share of [t0, t1] the measured pairs cover; weak: pairs < VO_MIN_PAIRS; frames: raw
# frames used; inliers: their median inlier count (0: none measured); t_first / t_last: the frames' times.
LegMotion = collections.namedtuple("LegMotion", "dx dy dtheta pairs weak frames inliers t_first t_last")

logger = logging.getLogger(__name__)


def load_jsonl(path):
    """Every line of a JSONL file. A last line that does not parse is skipped when it has no newline yet:
    the recorder keeps appending during a session and may be halfway through it."""
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except ValueError:
                if line.endswith("\n"):
                    raise
                logger.debug("%s: skipping a line still being written", path)
    return rows


def read_frames(run_dir):
    """raw/index.jsonl of a recording, in seq order; [] while it does not exist yet."""
    path = os.path.join(run_dir, RAW_DIR, RAW_INDEX)
    return sorted(load_jsonl(path), key=lambda fr: fr["seq"]) if os.path.exists(path) else []


def measure_motion(run_dir, frames, intrinsics, mount, times, stride=ODOMETRY_STRIDE):
    """frame_motion across frames `stride` apart, spread over the frames in between in proportion to
    their time: (N, MOTION_COLUMNS) array, columns COL_*; row 0 (no previous frame) and unmeasured
    frames have ok 0. On the real floor, steps of one frame (~0.5 deg) summed ~1 % short over a turn
    (72.7 vs 73.5 deg at stride 10); a stride that fails to match is measured frame by frame instead."""
    rows = np.zeros((len(frames), MOTION_COLUMNS))
    if len(frames) < 2:
        return rows
    rng = np.random.default_rng(0)
    masks = {}

    def gray(k):
        return cv2.imread(os.path.join(run_dir, RAW_DIR, RAW_PATTERN.format(seq=frames[k]["seq"])), cv2.IMREAD_GRAYSCALE)

    def measure(image0, image1):
        if image0 is None or image1 is None:
            return None
        if image0.shape not in masks:
            top, bottom = floor_rows(intrinsics, mount, image0.shape[0])
            masks[image0.shape] = np.zeros(image0.shape, np.uint8)
            masks[image0.shape][top:bottom, :] = 255
        return estimate_motion(*track(image0, image1, masks[image0.shape]), intrinsics, mount, rng)

    def store(first, last, motion):
        """motion between frames first and last onto rows first + 1 .. last."""
        share = np.diff(times[first:last + 1]) / (times[last] - times[first])
        for k, w in zip(range(first + 1, last + 1), share):
            rows[k] = (1, w * motion.dx, w * motion.dy, w * motion.dtheta, motion.inliers, motion.tracks, motion.rms_px)

    keys = list(range(0, len(frames), stride))
    if keys[-1] != len(frames) - 1:
        keys.append(len(frames) - 1)
    image0 = gray(keys[0])
    for k0, k1 in zip(keys, keys[1:]):
        image1 = gray(k1)
        motion = measure(image0, image1) if times[k1] > times[k0] else None
        if motion is not None:
            store(k0, k1, motion)
        else:
            logger.debug("odometry: frames %d-%d did not match, measured one by one", k0, k1)
            prev = image0
            for k in range(k0 + 1, k1 + 1):
                cur = image1 if k == k1 else gray(k)
                single = measure(prev, cur) if times[k] > times[k - 1] else None
                if single is not None:
                    store(k - 1, k, single)
                prev = cur
        image0 = image1
        if k1 // LOG_EVERY_FRAMES != k0 // LOG_EVERY_FRAMES:
            logger.info("odometry: %d / %d frames", k1, len(frames))
    return rows


def odometry_track(motion, scale=1.0, turn_scale=1.0):
    """(N,) heading and (N, 2) position of the robot centre, relative to the first frame, from the
    odometry alone: a failed pair counts as no motion. scale multiplies the translation (vo_scale),
    turn_scale the turn (vo_turn_scale)."""
    dtheta = motion[:, COL_DTHETA] * (motion[:, COL_OK] > 0) * turn_scale
    theta = np.cumsum(dtheta)
    mid = theta - 0.5 * dtheta
    dx, dy = motion[:, COL_DX] * scale, motion[:, COL_DY] * scale
    xy = np.column_stack((np.cumsum(dx * np.cos(mid) - dy * np.sin(mid)), np.cumsum(dx * np.sin(mid) + dy * np.cos(mid))))
    return theta, xy


def measured_share(times, motion, t0, t1):
    """Share of [t0, t1] covered by measured pairs: each counts for the part of the time between its two
    frames that falls inside. Time without frames - the camera lost, a span that runs past the last
    frame - counts as unmeasured."""
    if t1 <= t0 or len(times) < 2:
        return 0.0
    previous = np.concatenate(([times[0]], times[:-1]))
    overlap = np.clip(np.minimum(times, t1) - np.maximum(previous, t0), 0.0, None)
    return min(float(overlap[motion[:, COL_OK] > 0].sum()) / (t1 - t0), 1.0)


def _last_capture(run_dir):
    """t_capture of the last complete line of raw/index.jsonl, None when there is none yet. Reads only the
    file's tail: during a long session the index holds tens of thousands of lines."""
    path = os.path.join(run_dir, RAW_DIR, RAW_INDEX)
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(f.tell() - TAIL_BYTES, 0))
            pieces = f.read().decode("utf-8", "replace").split("\n")
    except OSError:
        return None
    for line in reversed(pieces[:-1]):           # after the last newline: a line still being written
        if line.strip():
            try:
                return float(json.loads(line)["t_capture"])
            except (ValueError, KeyError):
                return None                      # a malformed last line: the caller keeps waiting
    return None


def _frames_through(run_dir, t1, wait_s):
    """read_frames, once a frame at or after t1 is on disk (the raw writer runs a little behind the
    camera) or wait_s has passed."""
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        last = _last_capture(run_dir)
        if last is not None and last >= t1:
            break
        time.sleep(WAIT_POLL_S)
    return read_frames(run_dir)


def measure_leg(run_dir, t0, t1, intrinsics, mount, scale=1.0, wait_s=0.0, stride=ODOMETRY_STRIDE, turn_scale=1.0):
    """LegMotion of the robot centre from wall time t0 to t1, from the raw frames of run_dir: the last
    frame at or before t0 through the first at or after t1. wait_s: how long to wait for that last frame
    when the recorder is still writing. scale / turn_scale: odometry_track's. Too few frames gives a weak
    LegMotion of zero motion."""
    frames = _frames_through(run_dir, t1, wait_s)
    times = np.array([fr["t_capture"] for fr in frames])
    first = max(int(np.searchsorted(times, t0, side="right")) - 1, 0)
    last = min(int(np.searchsorted(times, t1, side="left")), len(frames) - 1)
    span = frames[first:last + 1]
    if len(span) < 2:
        logger.warning("leg %.2f-%.2f: %d raw frame(s) in %s", t0, t1, len(span), run_dir)
        t_first = span[0]["t_capture"] if span else float("nan")
        return LegMotion(0.0, 0.0, 0.0, 0.0, True, len(span), 0, t_first, t_first)
    span_times = times[first:last + 1]
    motion = measure_motion(run_dir, span, intrinsics, mount, span_times, stride)
    theta, xy = odometry_track(motion, scale, turn_scale)
    pairs = measured_share(span_times, motion, t0, t1)
    ok = motion[:, COL_OK] > 0
    inliers = int(np.median(motion[ok, COL_INLIERS])) if ok.any() else 0
    result = LegMotion(float(xy[-1, 0]), float(xy[-1, 1]), float(theta[-1]), pairs, pairs < VO_MIN_PAIRS,
                       len(span), inliers, float(span_times[0]), float(span_times[-1]))
    logger.debug("leg %.2f-%.2f: %s", t0, t1, result)
    return result
