"""Streaming budget of a demo_drive recording: how fast each stream ran live, how fast each map stage
runs on this Jetson, and how fast each must run for the map to keep up in real time
(task 2026-09-25_drive-map, step 14).

  python3 vision/stream_bench.py --run vision/output/<run_id> [--cam-forward M --cam-left M]

Real time here means two things. Throughput: a stage finishes an item before its next one is due, so
nothing queues and nothing is silently skipped. Latency: what it reports about the robot is still
true when it arrives. A stage need not see all ~30 camera frames a second - it needs the rate at
which the robot's motion between two of its frames stays inside what it can handle.

Printed, and written to RUN/map/stream.txt:
  live      from the recording's own logs: rate, per-item time and capture-to-result latency of the
            camera, YOLO and Depth Anything as they ran TOGETHER on the robot (they share the GPU).
  stages    drive_map's per-frame work timed on the recorded frames, one stage at a time, nothing else
            running and no model loaded: an upper bound on each stage's rate.
  vo sweep  visual odometry over every turning leg with frames 1..30 apart. The largest step it still
            measures right - every pair, the legs' total turn within MAX_TOTAL_ERR of drive_map's
            odometry, and so for every smaller stride too - bounds how far apart its frames may be.
  realtime  each stage's NEEDED rate at a turn rate against the rate it reaches. VO: turn rate / largest
            good step, and VO_MARGIN times that to stay clear of the edge. Floor analyses and YOLO: a
            spot stays in view hfov / turn rate seconds and needs FLOOR_ANALYSES_PER_SPOT resp.
            drive_map.LANDMARK_MIN_SIGHTINGS looks in that time. Latency: how far the robot moves before
            a result arrives - it delays the map, it does not smear it, since every result is placed
            at the pose of the frame it was computed on (its seq).
Only turning is measured: a run with FORWARD legs would also bound VO by the translation per step.
"""
import argparse
import collections
import logging
import math
import os
import sys
import tempfile
import time

import cv2
import numpy as np

import drive_map
import drive_timeline
from drawing import floor_overlay, render_map
from frame_motion import estimate_motion, floor_rows, track
from occupancy_map import OccupancyMap

STREAM_FILE = "stream.txt"
MS_PER_S = 1000.0
STRIDES = (1, 2, 5, 10, 15, 20, 30)
# A stride still measures right: its total turn within 3 % of drive_map's. Strides 1-20 scatter 1-3 %
# around it on the real floor (run 20260926_173930); a failing stride loses whole pairs (30: all of them).
MAX_TOTAL_ERR = 0.03
VO_MARGIN = 2.0                # VO frames at most half the largest good step apart
SMOOTH_FRAMES = 15             # turn rate smoothed over half a second
GAP_FACTOR = 1.5               # a capture interval this many medians long: the camera dropped a frame
TIMED_FRAMES = 60              # frames sampled for the per-frame stages
TIMED_RENDERS = 20
FLOOR_ANALYSES_PER_SPOT = 2    # drive_map.FLOOR_WEIGHT 0.5: a cell needs two agreeing analyses
DISPLAY_FPS = 10.0             # record.VIDEO_FPS: the live view's rate, and the loop's ceiling
# Turn rates to size the stages for, rad/s at the wheels' command; the images turn the run's measured
# ratio of it (0.79 on 2026-09-26).
COMMANDED_TURNS = (("demo cmd", 0.3), ("chassis max", 2.57))   # mc_max_speed: ~2.57 rad/s spinning
PERCENTILES = (50, 95)
PCT = 100.0
STREAM_YOLO, STREAM_FLOOR = "YOLO", "DA floor"
STAGE_DECODE, STAGE_FLOOR, STAGE_RENDER = "decode JPEG", "floor analysis -> map", "map render"
STAGE_CAMERA, STAGE_MAP, STAGE_ENCODE = "video: camera panel", "video: map panel", "video: encode frame"
VIEW_STAGES = (STAGE_DECODE, STAGE_CAMERA, STAGE_MAP, STAGE_ENCODE)   # one frame of the live view

# One stride of the VO sweep: median / largest turn between its frames (deg), median ms per pair, share
# of pairs measured, and the legs' total turn over drive_map's.
SweepRow = collections.namedtuple("SweepRow", "stride step_deg max_step_deg ms measured total_ratio")

logger = logging.getLogger(__name__)


def pct(values):
    """(median, p95) of values, NaN when there are none."""
    values = np.asarray(values, np.float64)
    return tuple(float(np.percentile(values, q)) for q in PERCENTILES) if values.size else (math.nan, math.nan)


def live_streams(dm):
    """(lines, live) of the camera, YOLO and Depth Anything streams as the recording logged them;
    live: {STREAM_YOLO: (rate /s, p95 capture-to-result latency s), STREAM_FLOOR: ...} for those it has."""
    times = dm.times
    dt = np.diff(times)
    period = np.median(dt)
    gaps = dt > GAP_FACTOR * period
    dropped = int(np.round(dt[gaps] / period).sum() - gaps.sum())   # a gap of n periods lost n - 1 frames
    fps = (len(times) - 1) / (times[-1] - times[0])
    arrival = [fr["t_arrival"] - fr["t_sensor"] for fr in dm.frames if fr.get("t_sensor") is not None]
    rates = {}
    lines = [f"camera   {fps:5.2f} fps  interval median {MS_PER_S * period:.1f} ms, max {MS_PER_S * dt.max():.0f} ms; "
             f"{dropped} frames dropped by the camera in {int(gaps.sum())} gaps (interval > {GAP_FACTOR} x median)"]
    if arrival:
        lines.append("         driver stamp -> Python: median %.0f ms, p95 %.0f ms" % tuple(MS_PER_S * v for v in pct(arrival)))
    rows = drive_map.load_jsonl(os.path.join(dm.run_dir, drive_map.DETECTIONS_FILE))
    if len(rows) > 1:
        t = np.array([r["t"] for r in rows])
        infer = pct([r["infer_ms"] for r in rows])
        latency = pct(MS_PER_S * (t - np.array([r["t_capture"] for r in rows])))
        step = np.median(np.diff([r["seq"] for r in rows]))
        rates[STREAM_YOLO] = ((len(rows) - 1) / (t[-1] - t[0]), latency[1] / MS_PER_S)
        lines.append(f"{STREAM_YOLO:8s} {rates[STREAM_YOLO][0]:5.2f} /s   infer median {infer[0]:.0f} ms, p95 {infer[1]:.0f} ms; "
                     f"capture -> drawn median {latency[0]:.0f} ms, p95 {latency[1]:.0f} ms; every {step:.0f}th frame")
        video = dm.recording.get("counts", {}).get("video_frames")
        if video:
            lines.append(f"         live view: {video} frames at {DISPLAY_FPS:.0f} fps, {len(rows)} distinct "
                         f"({PCT * (1.0 - len(rows) / video):.0f} % repeats)")
    floors = drive_map.floor_lines(dm.run_dir)
    if len(floors) > 1:
        done = np.array([f["t_done"] for f in floors])
        latency = pct(MS_PER_S * (done - np.array([f["t_capture"] for f in floors])))
        step = np.median(np.diff([f["seq"] for f in floors]))
        rates[STREAM_FLOOR] = ((len(floors) - 1) / (done[-1] - done[0]), latency[1] / MS_PER_S)
        lines.append(f"{STREAM_FLOOR:8s} {rates[STREAM_FLOOR][0]:5.2f} /s   one every {MS_PER_S * np.median(np.diff(done)):.0f} ms; "
                     f"capture -> done median {latency[0]:.0f} ms, p95 {latency[1]:.0f} ms; every {step:.0f}th frame")
    return lines, rates


def _timed(fn, *args):
    """(result, seconds) of fn(*args)."""
    t0 = time.perf_counter()
    result = fn(*args)
    return result, time.perf_counter() - t0


def raw_path(dm, k):
    return os.path.join(dm.run_dir, drive_map.RAW_DIR, drive_map.RAW_PATTERN.format(seq=dm.frames[k]["seq"]))


def turn_rate(dm):
    """(N,) turn rate the images measured, rad/s, smoothed over SMOOTH_FRAMES."""
    dt = np.diff(dm.times, prepend=dm.times[0])
    dt[0] = np.inf
    rate = np.where(dm.motion[:, drive_map.COL_OK] > 0, dm.motion[:, drive_map.COL_DTHETA], 0.0) / dt
    return np.convolve(rate, np.ones(SMOOTH_FRAMES) / SMOOTH_FRAMES, mode="same")


def vo_sweep(dm):
    """(rows, frames, peak) of the VO sweep over every turning leg's span: a SweepRow per stride, the
    frames swept, and the fastest smoothed turn rate in them (rad/s). None without a turning leg.
    Left and right legs count with their own sign, so opposite turns do not cancel."""
    spans = [fit.span for fit in dm.fits if fit.leg.kind == drive_timeline.ROTATE and fit.span[1] - fit.span[0] > 1]
    if not spans:
        return None
    peak = max(float(np.abs(turn_rate(dm)[first:last]).max()) for first, last in spans)
    top, bottom = floor_rows(dm.intrinsics, dm.mount, dm.recording["frame_size"][1])
    mask = np.zeros(tuple(reversed(dm.recording["frame_size"])), np.uint8)
    mask[top:bottom, :] = 255
    rng = np.random.default_rng(0)

    def measure(gray0, gray1):
        return estimate_motion(*track(gray0, gray1, mask), dm.intrinsics, dm.mount, rng)

    rows = []
    for stride in STRIDES:
        total = reference = 0.0
        measured, seconds, steps = 0, [], []
        for first, last in spans:
            sign = math.copysign(1.0, dm.motion[first:last, drive_map.COL_DTHETA].sum())
            keys = list(range(first, last, stride))
            prev = cv2.imread(raw_path(dm, keys[0]), cv2.IMREAD_GRAYSCALE)
            for k0, k1 in zip(keys, keys[1:]):
                cur = cv2.imread(raw_path(dm, k1), cv2.IMREAD_GRAYSCALE)
                if prev is None or cur is None:
                    logger.warning("VO sweep: raw frame %d or %d unreadable, pair left out", k0, k1)
                    prev = cur
                    continue
                motion, sec = _timed(measure, prev, cur)
                prev = cur
                step = float(dm.motion[k0 + 1:k1 + 1, drive_map.COL_DTHETA].sum())
                seconds.append(sec)
                steps.append(abs(step))
                reference += sign * step
                if motion is not None:
                    total += sign * motion.dtheta
                    measured += 1
        if not seconds:
            continue
        rows.append(SweepRow(stride, math.degrees(float(np.median(steps))), math.degrees(max(steps)),
                             MS_PER_S * float(np.median(seconds)), measured / len(seconds),
                             total / reference if reference else math.nan))
        logger.info("VO sweep: stride %d, %d / %d pairs measured", stride, measured, len(seconds))
    return rows, sum(last - first for first, last in spans), peak


def largest_good_step(rows):
    """The SweepRow of the largest stride that - with every smaller one - measured all its pairs and
    the turn within MAX_TOTAL_ERR, or None."""
    good = None
    for row in rows:
        if row.measured < 1.0 or not abs(row.total_ratio - 1.0) <= MAX_TOTAL_ERR:
            break
        good = row
    return good


def time_stages(dm, window):
    """{stage: median ms per item} of drive_map's per-frame work on the recorded frames."""
    picks = np.linspace(0, len(dm.frames) - 1, TIMED_FRAMES).astype(int)
    decode = [_timed(cv2.imread, raw_path(dm, int(k)))[1] for k in picks]

    occ_map = OccupancyMap()
    integrate = []
    for line in drive_map.floor_lines(dm.run_dir):
        if not drive_map.floor_usable(line, dm.pose_of, dm.rate_of):
            continue
        labels = drive_map.floor_labels(dm.run_dir, line)
        if labels is not None:
            integrate.append(_timed(drive_map.integrate_floor_line, occ_map, labels, line, dm.pose_of[line["seq"]],
                                    dm.intrinsics, dm.mount)[1])
    video = drive_map.MapVideo(dm, window)
    render = [_timed(render_map, dm.occ_map, [], [], None, video.map_size, None, (), window)[1]
              for _ in range(TIMED_RENDERS)]

    detections = {r["seq"]: r for r in drive_map.load_jsonl(os.path.join(dm.run_dir, drive_map.DETECTIONS_FILE))}
    floor = None                         # the overlay every timed frame gets: the first readable analysis
    for line in drive_map.floor_lines(dm.run_dir):
        labels = drive_map.floor_labels(dm.run_dir, line)
        if labels is not None:
            floor = line, floor_overlay(labels)
            break
    base = render_map(dm.occ_map, [], [], None, video.map_size, window=window)
    camera, board, encode = [], [], []
    with tempfile.TemporaryDirectory() as tmp:
        writer = video.open_writer(os.path.join(tmp, drive_map.VIDEO_FILE))
        try:
            for k in picks:
                k = int(k)
                seq = dm.frames[k]["seq"]
                # A frame YOLO skipped still gets boxes drawn: the cost of the frames that have them.
                row = detections.get(seq) or next(iter(detections.values()), None)
                image = cv2.imread(raw_path(dm, k))
                if image is None:
                    logger.warning("raw frame %d unreadable, not timed", seq)
                    continue
                left, sec = _timed(video.camera_panel, image, seq, row, floor)
                camera.append(sec)
                right, sec = _timed(video.map_panel, base.copy(), k, row)
                board.append(sec)
                encode.append(_timed(writer.write, np.hstack((left, right)))[1])
        finally:
            writer.release()
    return {name: MS_PER_S * float(np.median(values)) if values else math.nan for name, values in (
        (STAGE_DECODE, decode), (STAGE_FLOOR, integrate), (STAGE_RENDER, render),
        (STAGE_CAMERA, camera), (STAGE_MAP, board), (STAGE_ENCODE, encode))}


def realtime_table(stages, sweep, hfov, live, turn_ratio, cruise_speed):
    """Lines of the needed-vs-reached table. live: live_streams' {stream: (rate /s, p95 latency s)};
    cruise_speed: the run's forward speed, m/s, at the wheels' command."""
    good = largest_good_step(sweep[0]) if sweep else None
    lines = []
    if good is None:
        lines.append("VO: no stride measured the turns right - its needed rate is unknown")
    else:
        lines.append(f"VO measured every turn right up to {good.max_step_deg:.1f} deg between its frames (stride "
                     f"{good.stride}); one pair costs ~{good.ms:.0f} ms alone -> up to {MS_PER_S / good.ms:.1f} pairs/s")
    turns = [("this run, peak", sweep[2])] if sweep else []
    turns += [(f"{name} {cmd}", cmd * turn_ratio) for name, cmd in COMMANDED_TURNS]
    lines.append(f"{'turn (images)':26s} {'VO min':>8s} {'VO x' + str(int(VO_MARGIN)):>8s} {'floor':>8s} {'YOLO':>8s}   verdict")
    for name, omega in turns:
        in_view = hfov / omega                      # s a spot stays in view while turning
        need_vo = math.degrees(omega) / good.max_step_deg if good else math.nan
        need_floor = FLOOR_ANALYSES_PER_SPOT / in_view
        need_yolo = drive_map.LANDMARK_MIN_SIGHTINGS / in_view
        verdict = []
        if good:
            load = VO_MARGIN * need_vo * good.ms / MS_PER_S
            verdict.append(f"VO {'ok' if load < 1.0 else 'TOO SLOW'} ({PCT * load:.0f} % of a core)")
        for stream, need in ((STREAM_FLOOR, need_floor), (STREAM_YOLO, need_yolo)):
            if stream in live:
                verdict.append(f"{stream} {'ok' if live[stream][0] >= need else 'TOO SLOW'}")
        if omega > drive_map.MAX_TURN_RATE:
            verdict.append(f"floor frames skipped (> {drive_map.MAX_TURN_RATE} rad/s)")
        lines.append(f"{name + f' {omega:.2f} rad/s':26s} {need_vo:5.1f} Hz {VO_MARGIN * need_vo:5.1f} Hz "
                     f"{need_floor:5.2f} Hz {need_yolo:5.2f} Hz   " + ", ".join(verdict))
    speed = cruise_speed * drive_map.DEFAULT_DISTANCE_SCALE
    omega = turns[0][1] if turns else 0.0         # this run's peak turn, else the demo's
    for stream, (_, latency) in live.items():
        lines.append(f"{stream} p95 latency {MS_PER_S * latency:.0f} ms: the robot moves {PCT * speed * latency:.1f} cm "
                     f"at {speed:.2f} m/s, turns {math.degrees(omega * latency):.0f} deg at {omega:.2f} rad/s "
                     "before the result arrives")
    frame_ms = sum(stages[name] for name in VIEW_STAGES if not math.isnan(stages[name]))
    lines.append(f"live view: decode + panels + encode = {frame_ms:.0f} ms a frame -> up to {MS_PER_S / frame_ms:.0f} fps "
                 f"on one core (the recorder draws {DISPLAY_FPS:.0f} fps)")
    return lines


def _parse_args():
    parser = argparse.ArgumentParser(description="Measure the streaming budget of a demo_drive recording.")
    parser.add_argument("--run", required=True, help="vision/output/<run_id>")
    parser.add_argument("--cam-forward", type=float, help="override the recording's lens offset ahead of the robot centre, m")
    parser.add_argument("--cam-left", type=float, help="override the recording's lens offset left of the robot centre, m")
    return parser.parse_args()


def main():
    args = _parse_args()
    run_dir = args.run.rstrip("/")
    recording, intrinsics, mount = drive_map.load_camera(run_dir, forward_m=args.cam_forward, left_m=args.cam_left)
    try:
        dm = drive_map.build(run_dir, recording, intrinsics, mount)
    except FileNotFoundError as exc:
        raise SystemExit(str(exc)) from exc
    _, _, window = drive_map.map_view(dm)
    hfov = drive_map.horizontal_fov(intrinsics)
    turns = [f.ratio for f in dm.fits if f.leg.kind == drive_timeline.ROTATE and math.isfinite(f.ratio)]
    turn_ratio = float(np.median(turns)) if turns else 1.0

    lines = [f"run {os.path.basename(run_dir)}   {len(dm.frames)} raw frames over {dm.times[-1] - dm.times[0]:.1f} s   "
             f"hfov {math.degrees(hfov):.1f} deg   images turn {turn_ratio:.2f} of the command", "", "LIVE (as recorded)"]
    live, live_rates = live_streams(dm)
    lines += ["  " + line for line in live]

    logger.info("timing the map stages ...")
    stages = time_stages(dm, window)
    lines += ["", "STAGES (median ms per item, alone on this Jetson -> the most items per second)"]
    lines += [f"  {name:24s} {ms:7.1f} ms  -> {MS_PER_S / ms:6.1f} /s" for name, ms in stages.items() if not math.isnan(ms)]

    logger.info("VO sweep ...")
    sweep = vo_sweep(dm)
    if sweep is None:
        lines += ["", "VO SWEEP: the run never turns - nothing to sweep"]
    else:
        rows, swept, peak = sweep
        lines += ["", f"VO SWEEP over the {swept} frames of the turning legs (peak {math.degrees(peak):.1f} deg/s in the images)",
                  "  stride  step_deg  max_step  ms/pair  measured  total/drive_map"]
        lines += [f"  {r.stride:6d}  {r.step_deg:8.2f}  {r.max_step_deg:8.2f}  {r.ms:7.1f}  {PCT * r.measured:7.0f} %  "
                  f"{r.total_ratio:14.3f}" for r in rows]
    lines += ["", "REALTIME (needed vs reached)"] + ["  " + line for line in realtime_table(
        stages, sweep, hfov, live_rates, turn_ratio, dm.run.cruise_speed)]
    path = os.path.join(run_dir, drive_map.MAP_DIR, STREAM_FILE)
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"\n-> {path}")
    return 0


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    sys.exit(main())
