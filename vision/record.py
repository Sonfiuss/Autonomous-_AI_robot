"""Headless detection recorder for the drive demo. Per frame, drawn and written to a video:
  * YOLO boxes, each with a distance: measured by the Astra depth when it has one ("1.23 m"), else
    estimated from where the box meets the floor, by camera height and pitch ("~1.23 m");
  * the free floor ahead from Depth Anything (floor_segment.py, as map_builder uses it): free green,
    obstacle red, drop magenta, the searched trapezoid outlined, and how far the robot's corridor is
    clear. DA takes ~0.3 s a frame on the Orin, so it runs on its own thread with the floor analysis
    and the overlay lags the video by one inference; YOLO still runs on every frame;
  * the motion step being driven (--status-file), and the time since the video started.

Normally started by communication/demo_drive.py, which stops it with SIGINT. By hand:
  python3 vision/record.py --out vision/output/test
  python3 vision/record.py --out vision/output/test --no-depth --no-floor --max-seconds 20

Four streams consume the camera, each at its own pace; every output names the frame it came from by
its seq (frame_grabber.Frame.seq), so they map onto each other exactly:
  raw        every frame the camera delivers (~30 fps), on its own thread (raw_recorder.py)
  YOLO       the newest frame each cycle (<= VIDEO_FPS)
  floor      Depth Anything + analyze_floor on the newest frame YOLO handed it (~2 per s)
  video      the YOLO frame drawn, repeated or skipped to keep wall time

Written to --out:
  raw/               <seq>.jpg + index.jsonl (seq, t_capture, t_sensor, t_arrival): the undrawn frames
                     (--no-raw: none). Every seq named below is in it. t_capture everywhere is the wall
                     clock of the capture, from the V4L2 driver's stamp (frame_grabber.capture_wall).
  detections.mp4     annotated frames at VIDEO_FPS. A frame is repeated when inference runs slower than
                     that, so the video always plays in real time and lines up with the wall-clock
                     timeline in the orchestrator's run.json.
  detections.jsonl   one line per YOLO frame: seq, t_capture, depth_seq,
                     video {first, count} (the video frames showing it; count 0 = skipped), wall time,
                     depth status, motion step, detections (range_source depth | floor), floor summary
                     (with the seq and t_capture of the frame Depth Anything ran on: the overlay lags).
  floor/             <seq>.png (occupancy_map.PIXEL_* per pixel) + index.jsonl (seq, t_capture, t_done,
                     plane, pitch, the YOLO boxes used as vetoes, contacts, summary): every floor analysis.
  recording.json     mount, intrinsics, floor config, stream settings; rewritten at the end with counts
                     and light (light_check.py: brightness + ORB keypoints; a dark run is warned about).
  recorder.ready     created once the first frame is in the video; the orchestrator waits on it.
Hershey fonts are ASCII only, so the status file must be too. Depth and floor are both optional:
without the OpenNI2 binding / redist / device the recorder runs RGB ONLY, without transformers or the
DA weights it runs without the floor overlay. Neither stops the video.
"""
import argparse
import collections
import json
import logging
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime

import cv2
import numpy as np

from color_source import ColorSource
from detector import DEFAULT_CONF, DEFAULT_IMGSZ, DEFAULT_MODEL, Detector, select_device
from drawing import FRAME_H, FRAME_W, blend_floor, put_text, render_view
from floor_geometry import CameraMount, default_mount
from floor_segment import DEFAULT_FAR_M, FloorConfig, analyze_floor, box_floor_range, floor_hit
from frame_grabber import capture_wall, pair_frames
from light_check import LightCheck
from mono_depth import DEFAULT_INPUT_SIZE           # the module defers transformers to MonoDepth()
from object_distance import (default_intrinsics_file, fallback_intrinsics, intrinsics_from_fov, load_intrinsics,
                             measure_object, result_records)
from occupancy_map import PIXEL_FREE, PIXEL_OBSTACLE
from raw_recorder import CAPTURE_TIME_DIGITS, DEFAULT_JPEG_QUALITY, INDEX_NAME, RAW_DIR, RawFrameWriter

VIDEO_STEM = "detections"          # + the extension of the codec that opened
JSONL_NAME = "detections.jsonl"
READY_NAME = "recorder.ready"
STATUS_NAME = "status.txt"
RECORDING_NAME = "recording.json"
FLOOR_DIR = "floor"
FLOOR_LABELS_PATTERN = "{seq:06d}.png"   # same seq as raw/<seq>.jpg
BOX_DIGITS = 1
VIDEO_FPS = 10.0                   # written rate, and the processing ceiling
VIDEO_CODECS = (("avc1", ".mp4"), ("mp4v", ".mp4"), ("MJPG", ".avi"))   # the first that opens wins
DEFAULT_MAX_SECONDS = 900.0        # a recorder whose parent died still ends
MAX_SKEW_S = 0.040                 # as perception.py: RGB/depth further apart than this are not paired
MAX_AGE_S = 0.5                    # a source whose newest frame is older than this counts as lost
STATUS_OK = "OK"                   # pair_frames' word for a good RGB/depth pair
STATUS_RGB_ONLY = "RGB ONLY"
STATUS_NO_COLOR = "NO COLOR"
TIME_DIGITS = 3                    # seconds in detections.jsonl (ms)
MS_DIGITS = 1
PCT_DIGITS = 1
METRE_DIGITS = 3
MOTION_BAR_H = 24                  # each of the bottom bars: floor summary, then motion step
BOTTOM_BARS = 2
TEXT_X = 6                         # left margin of bar text, px
TEXT_BASELINE = 8                  # text baseline above a bar's bottom edge, px
MOTION_TEXT_COLOR = (0, 255, 255)
FLOOR_TEXT_COLOR = (0, 220, 0)
BAR_COLOR = (0, 0, 0)
FLOOR_OFF_TEXT = "floor: off"
FLOOR_WAITING_TEXT = "floor (DA): waiting"
FLOOR_NOT_IN_VIEW_TEXT = "floor not in view"
MS_PER_S = 1000.0
LOG_INTERVAL_S = 5.0
EXIT_NO_FRAME = 1                  # nothing was ever recorded
CORRIDOR_HALF_WIDTH_M = 0.25       # robot hull radius (~0.225, CM's MV_DEFAULT_ROBOT_RADIUS_M) + margin
FLOOR_MAX_AGE_S = 2.0              # an older floor analysis is not drawn: it no longer shows where we are
PCT = 100.0

# view: floor_segment.FloorView. clear_m: free floor straight ahead in the robot's corridor (None: floor
# not in view). t: time.monotonic() when the analysis finished. seq / t_capture: the analysed frame's
# (t_capture on the wall clock, frame_grabber.capture_wall).
FloorResult = collections.namedtuple("FloorResult", "view free_pct obstacle_pct clear_m t seq t_capture")

logger = logging.getLogger(__name__)


def clear_ahead_m(view, intrinsics, mount, far_m):
    """Distance ahead of the camera to the nearest contact inside the robot's corridor; far_m when
    the corridor is free as far as the trapezoid reaches; None when the trapezoid was not floor (no
    plane). From the camera, like far_m: it sits near the robot's front, ~18.5 cm ahead of its center.
    The corridor is the robot's, centred on its center line, not on the camera (mount.left_m to the side)."""
    if view.plane is None:
        return None
    if not len(view.contacts):
        return far_m
    forward, left, _ = floor_hit(view.contacts[:, 0], view.contacts[:, 1], intrinsics, mount)
    with np.errstate(invalid="ignore"):
        inside = np.isfinite(forward) & (np.abs(left) <= CORRIDOR_HALF_WIDTH_M)
    return float(forward[inside].min()) - mount.forward_m if inside.any() else far_m


class FloorLog:
    """Every floor analysis, keyed by the seq of the frame it ran on: FLOOR_DIR/<seq>.png holds the
    per-pixel labels, FLOOR_DIR/index.jsonl one line with the rest. Written on the floor thread."""

    def __init__(self, out_dir):
        self.dir = os.path.join(out_dir, FLOOR_DIR)
        os.makedirs(self.dir, exist_ok=True)
        self._index = open(os.path.join(self.dir, INDEX_NAME), "w", buffering=1, encoding="utf-8")
        self.written = 0

    def write(self, result, boxes):
        view = result.view
        name = FLOOR_LABELS_PATTERN.format(seq=result.seq)
        if not cv2.imwrite(os.path.join(self.dir, name), view.labels):
            raise OSError(f"cannot write {os.path.join(self.dir, name)}")
        self._index.write(json.dumps({
            "seq": result.seq, "t_capture": round(result.t_capture, CAPTURE_TIME_DIGITS),
            "t_done": round(time.time(), CAPTURE_TIME_DIGITS), "labels": name,
            "plane": None if view.plane is None else list(view.plane), "pitch_deg": view.pitch_deg,
            "free_pct": round(result.free_pct, PCT_DIGITS), "obstacle_pct": round(result.obstacle_pct, PCT_DIGITS),
            "clear_ahead_m": None if result.clear_m is None else round(result.clear_m, METRE_DIGITS),
            "boxes": [[round(float(v), BOX_DIGITS) for v in box] for box in boxes],
            "contacts": view.contacts.tolist(), "da_contact": view.da_contact.tolist()}) + "\n")
        self.written += 1

    def close(self):
        self._index.close()


class FloorWorker:
    """Depth Anything + analyze_floor on one daemon thread, the newest frame winning (like
    mono_depth.MonoDepthWorker, which runs only the first half). Both halves are slow - ~0.3 s and
    ~0.15 s on the Orin - and neither may stall the video loop. log (FloorLog or None) gets every
    result."""

    def __init__(self, mono, mount, floor_cfg, log=None):
        self._mono = mono
        self._mount = mount
        self._floor_cfg = floor_cfg
        self._log = log
        self._cond = threading.Condition()
        self._pending = None             # (frame, boxes, depth_mm, intrinsics) not yet started
        self._result = None
        self._failures = 0
        self._log_failures = 0
        self._running = False
        self._thread = None

    @property
    def analyses(self):
        return 0 if self._log is None else self._log.written

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, name="floor", daemon=True)
        self._thread.start()

    def submit(self, frame, boxes, depth_mm, intrinsics):
        """frame: frame_grabber.Frame; boxes: that frame's YOLO boxes."""
        with self._cond:
            self._pending = (frame, boxes, depth_mm, intrinsics)
            self._cond.notify_all()

    def latest(self):
        with self._cond:
            return self._result

    def close(self):
        with self._cond:
            self._running = False
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=FLOOR_MAX_AGE_S)
            if self._thread.is_alive():
                logger.warning("floor thread still busy after %.0f s: its last analysis is not logged",
                               FLOOR_MAX_AGE_S)
        if self._log is not None:
            self._log.close()

    def _run(self):
        while True:
            with self._cond:
                self._cond.wait_for(lambda: self._pending is not None or not self._running)
                if not self._running:
                    return
                frame, boxes, depth_mm, intrinsics = self._pending
                self._pending = None
            try:
                view = analyze_floor(self._mono.infer(frame.data), depth_mm, boxes, intrinsics, self._mount,
                                     self._floor_cfg)
            except Exception:
                # The video goes on without the overlay. One traceback, not one per frame.
                self._failures += 1
                if self._failures == 1:
                    logger.exception("floor analysis failed")
                else:
                    logger.debug("floor analysis failed again (%d)", self._failures)
                continue
            inside = view.labels[view.trapezoid]
            free_pct = PCT * float((inside == PIXEL_FREE).mean()) if inside.size else 0.0
            obstacle_pct = PCT * float((inside == PIXEL_OBSTACLE).mean()) if inside.size else 0.0
            clear = clear_ahead_m(view, intrinsics, self._mount, self._floor_cfg.far_m)
            result = FloorResult(view, free_pct, obstacle_pct, clear, time.monotonic(), frame.seq,
                                 capture_wall(frame))
            with self._cond:
                self._result = result
            self._write_log(result, boxes)

    def _write_log(self, result, boxes):
        if self._log is None:
            return
        try:
            self._log.write(result, boxes)
        except (OSError, ValueError):     # ValueError: the file was closed under a thread close() gave up on
            self._log_failures += 1
            if self._log_failures == 1:
                logger.exception("floor log failed")


class VideoSink:
    """The run's output files. write() paces the video to wall time; the first frame opens it."""

    def __init__(self, out_dir, ready_file):
        self.out_dir = out_dir
        self.ready_file = ready_file
        self.video_path = None
        self.frames = 0                  # video frames written, repeats included
        self.records = 0                 # processed frames logged
        self._writer = None
        self._t0 = None
        self._jsonl = open(os.path.join(out_dir, JSONL_NAME), "w", encoding="utf-8")

    def elapsed_s(self):
        return 0.0 if self._t0 is None else time.monotonic() - self._t0

    def write(self, canvas, record):
        """Adds record["video"] = {first, count}: the video frames showing this canvas."""
        now = time.monotonic()
        if self._writer is None:
            self._open(canvas.shape[1], canvas.shape[0])
            self._t0 = now
        due = int((now - self._t0) * VIDEO_FPS) + 1
        first = self.frames
        while self.frames < due:         # 0 iterations: ahead of wall time, this frame is dropped
            self._writer.write(canvas)
            self.frames += 1
        record["video"] = {"first": first, "count": self.frames - first}
        self._jsonl.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.records += 1
        if self.records == 1:
            with open(self.ready_file, "w", encoding="utf-8") as f:
                f.write(self.video_path + "\n")

    def close(self):
        if self._writer is not None:
            self._writer.release()       # writes the mp4 index; without it the file does not play
        self._jsonl.close()
        logger.info("recorded %d frames (%.1f s) to %s, %d detection rows", self.frames,
                    self.frames / VIDEO_FPS, self.video_path, self.records)

    def _open(self, width, height):
        for fourcc, ext in VIDEO_CODECS:
            path = os.path.join(self.out_dir, VIDEO_STEM + ext)
            writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), VIDEO_FPS, (width, height))
            if writer.isOpened():
                self._writer, self.video_path = writer, path
                logger.info("video %s (%s, %.0f fps)", path, fourcc, VIDEO_FPS)
                return
            writer.release()
            logger.warning("codec %s unavailable", fourcc)
        raise RuntimeError("no video codec opened: " + ", ".join(c for c, _ in VIDEO_CODECS))


@dataclass
class Pipeline:
    """The sources and models one recording runs on. depth_src / floor_worker / raw_writer None: that
    part is off. raw_writer is color_src's listener."""
    detector: object
    device: str
    color_src: object
    depth_src: object
    floor_worker: object
    raw_writer: object
    mount: CameraMount
    intrinsics: object                   # Intrinsics; fallback_intrinsics until something better is known
    intrinsics_fixed: bool               # from --intrinsics: never replaced by the Astra's FOV

    def close(self):
        """Stops every thread. False when a camera thread is stuck in its driver."""
        if self.floor_worker is not None:
            self.floor_worker.close()
        clean = self.color_src.close()
        if self.depth_src is not None:
            clean = self.depth_src.close() and clean
        if self.raw_writer is not None:
            self.color_src.listener = None   # a stuck grabber that wakes up later offers nothing more
            self.raw_writer.close()
        return clean


class DetectionRecorder:
    """Pipeline + sink; run() loops until stop is set or max_seconds pass."""

    def __init__(self, pipeline, sink, status_file):
        self.pipeline = pipeline
        self.sink = sink
        self.status_file = status_file
        self.light = LightCheck()
        self._motion = ""
        self._last_state = None

    def run(self, stop, max_seconds):
        period = 1.0 / VIDEO_FPS
        started = time.monotonic()
        last_log_t = 0.0
        while not stop.is_set():
            t0 = time.monotonic()
            if t0 - started > max_seconds:
                logger.warning("max %.0f s reached, stopping", max_seconds)
                return
            record = self._cycle(t0)
            if record is not None and t0 - last_log_t >= LOG_INTERVAL_S:
                logger.info("%s | infer %.0f ms | %d objects | %s", record["status"], record["infer_ms"],
                            len(record["detections"]), record["motion"] or "-")
                last_log_t = t0
            stop.wait(max(period - (time.monotonic() - t0), 0.0))

    def _cycle(self, now):
        """One frame: detect, measure, draw, write. None when there is no fresh color frame."""
        p = self.pipeline
        self._update_intrinsics()
        color = p.color_src.latest()
        if color is None or now - color.t > MAX_AGE_S:
            self._log_state(STATUS_NO_COLOR)
            return None
        depth = None
        if p.depth_src is None:
            status = STATUS_RGB_ONLY
        else:
            pair, status = pair_frames(color, p.depth_src.frames(), now, MAX_SKEW_S, MAX_AGE_S)
            if pair is not None:
                color, depth = pair
        self._log_state(status)
        if p.raw_writer is not None:
            p.raw_writer.ensure(color)   # this frame's outputs are about to be recorded: its image must exist
        self.light.add(color.data, color.seq)

        t_infer = time.monotonic()
        detections = p.detector.detect(color.data)
        infer_ms = (time.monotonic() - t_infer) * MS_PER_S
        results = [(d, self._range(d.box, depth)) for d in detections]
        floor = self._floor(color, [d.box for d in detections], depth)

        self._motion = self._read_motion()
        base = color if floor is None else color._replace(
            data=blend_floor(color.data, floor.view.labels, floor.view.trapezoid))
        canvas = render_view(base, depth, results, status, f"infer {infer_ms:3.0f} ms | {p.device}", False)
        # Not `now`: the floor result may have landed while YOLO ran, after the cycle began.
        drawn_at = time.monotonic()
        self._draw_bars(canvas, floor, drawn_at)
        record = {"seq": color.seq, "t_capture": round(capture_wall(color), CAPTURE_TIME_DIGITS),
                  "depth_seq": None if depth is None else depth.seq,
                  "t": round(time.time(), TIME_DIGITS), "video_s": round(self.sink.elapsed_s(), TIME_DIGITS),
                  "status": status, "motion": self._motion, "infer_ms": round(infer_ms, MS_DIGITS),
                  "detections": result_records(results), "floor": self._floor_record(floor, drawn_at)}
        self.sink.write(canvas, record)
        return record

    def _range(self, box, depth):
        """The Astra's measurement when it has one, else the floor-geometry estimate (or None)."""
        if depth is not None:
            measured = measure_object(depth.data, box, self.pipeline.intrinsics)
            if measured is not None:
                return measured
        return box_floor_range(box, FRAME_H, self.pipeline.intrinsics, self.pipeline.mount)

    def _floor(self, color, boxes, depth):
        """Hands this frame to the floor worker; returns its newest result if still fresh enough."""
        if self.pipeline.floor_worker is None:
            return None
        self.pipeline.floor_worker.submit(color, boxes, depth.data if depth is not None else None,
                                          self.pipeline.intrinsics)
        floor = self.pipeline.floor_worker.latest()
        if floor is None or time.monotonic() - floor.t > FLOOR_MAX_AGE_S:
            return None
        return floor

    def _draw_bars(self, canvas, floor, now):
        """Bottom: the floor summary above the motion step."""
        top = FRAME_H - BOTTOM_BARS * MOTION_BAR_H
        cv2.rectangle(canvas, (0, top), (FRAME_W, FRAME_H), BAR_COLOR, -1)
        if self.pipeline.floor_worker is None:
            floor_text = FLOOR_OFF_TEXT
        elif floor is None:
            floor_text = FLOOR_WAITING_TEXT
        else:
            clear = FLOOR_NOT_IN_VIEW_TEXT if floor.clear_m is None else f"clear ahead {floor.clear_m:.2f} m"
            floor_text = (f"floor (DA): free {floor.free_pct:.0f}%  obstacle {floor.obstacle_pct:.0f}%  |  "
                          f"{clear}  |  DA {now - floor.t:.1f}s ago")
        put_text(canvas, floor_text, (TEXT_X, top + MOTION_BAR_H - TEXT_BASELINE), FLOOR_TEXT_COLOR)
        put_text(canvas, f"t+{self.sink.elapsed_s():5.1f}s  {self._motion}", (TEXT_X, FRAME_H - TEXT_BASELINE),
                 MOTION_TEXT_COLOR)

    @staticmethod
    def _floor_record(floor, now):
        """The floor summary as a jsonl field, naming the frame it was computed on (floor/<seq>.png);
        None when no fresh analysis was drawn."""
        if floor is None:
            return None
        return {"seq": floor.seq, "t_capture": round(floor.t_capture, CAPTURE_TIME_DIGITS),
                "free_pct": round(floor.free_pct, PCT_DIGITS), "obstacle_pct": round(floor.obstacle_pct, PCT_DIGITS),
                "clear_ahead_m": None if floor.clear_m is None else round(floor.clear_m, METRE_DIGITS),
                "age_s": round(now - floor.t, TIME_DIGITS)}

    def _update_intrinsics(self):
        p = self.pipeline
        if not p.intrinsics_fixed and p.depth_src is not None and p.depth_src.fov is not None:
            p.intrinsics = intrinsics_from_fov(FRAME_W, FRAME_H, *p.depth_src.fov)
            p.intrinsics_fixed = True
            logger.warning("no --intrinsics file: using the OpenNI2 FOV (fx=%.1f fy=%.1f)",
                           p.intrinsics.fx, p.intrinsics.fy)

    def _read_motion(self):
        """The orchestrator replaces the file atomically; a missing file keeps the last line."""
        try:
            with open(self.status_file, encoding="utf-8") as f:
                return f.read().strip()
        except OSError:
            return self._motion

    def _log_state(self, state):
        if state != self._last_state:
            (logger.info if state in (STATUS_OK, STATUS_RGB_ONLY) else logger.warning)("state: %s", state)
            self._last_state = state


def _open_depth(enabled):
    """A started DepthSource, or None for RGB only (disabled, or no binding / redist)."""
    if not enabled:
        return None
    try:
        from depth_source import DepthSource, resolve_redist_path   # imports the openni binding
        source = DepthSource(resolve_redist_path())
    except (ImportError, RuntimeError) as exc:
        logger.warning("depth unavailable (%s): recording RGB only", exc)
        return None
    source.start()
    return source


def _open_floor(enabled, device, da_size, mount, floor_cfg, out_dir):
    """A started FloorWorker logging to out_dir/FLOOR_DIR, or None (disabled, or transformers / the DA
    weights unavailable)."""
    if not enabled:
        return None
    try:
        from mono_depth import MonoDepth
        mono = MonoDepth(device, da_size)        # imports transformers here: seconds, and optional
    except (ImportError, OSError, RuntimeError) as exc:   # no package; no weights offline; CUDA
        logger.warning("Depth Anything unavailable (%s): recording without the floor overlay", exc)
        return None
    worker = FloorWorker(mono, mount, floor_cfg, FloorLog(out_dir))
    worker.start()
    return worker


def _write_recording(out_dir, args, pipeline, floor_cfg, sink=None, light=None):
    """recording.json: what the streams were recorded with; with sink (at the end) also how much of each,
    and light (LightCheck.result()) how well the scene was lit."""
    p = pipeline
    meta = {"created": datetime.now().isoformat(timespec="seconds"),
            "frame_size": [FRAME_W, FRAME_H], "color_index": args.color_index,
            "intrinsics": p.intrinsics._asdict(),
            "intrinsics_source": "file" if args.intrinsics else ("astra_fov" if p.intrinsics_fixed else "fallback_fov"),
            "mount": p.mount._asdict(),
            "depth": p.depth_src is not None,
            "floor": None if p.floor_worker is None else dict(floor_cfg._asdict(), da_input_size=args.da_size),
            "raw": None if p.raw_writer is None else {"dir": RAW_DIR, "index": INDEX_NAME,
                                                       "jpeg_quality": p.raw_writer.jpeg_quality},
            "seq": "frame_grabber.Frame.seq of the color camera: raw/<seq>.jpg, detections.jsonl seq, "
                   "floor/<seq>.png and floor/index.jsonl seq all name the same frame"}
    if sink is not None:
        meta["counts"] = {"yolo_frames": sink.records, "video_frames": sink.frames,
                          "floor_analyses": 0 if p.floor_worker is None else p.floor_worker.analyses,
                          "raw_written": None if p.raw_writer is None else p.raw_writer.written,
                          "raw_dropped": None if p.raw_writer is None else p.raw_writer.dropped}
        meta["light"] = light            # None: the run ended before the check had its frames
    path = os.path.join(out_dir, RECORDING_NAME)
    with open(path + ".tmp", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    os.replace(path + ".tmp", path)


def _parse_args():
    mount = default_mount()              # vision/camera_mount.json when calib_floor.py has written it
    parser = argparse.ArgumentParser(description="Record YOLO detections, distances and free floor to a video.")
    parser.add_argument("--out", required=True, help="output folder, created if missing")
    parser.add_argument("--status-file", help=f"text drawn on every frame (default: OUT/{STATUS_NAME})")
    parser.add_argument("--ready-file", help=f"created after the first frame (default: OUT/{READY_NAME})")
    parser.add_argument("--no-depth", action="store_true", help="RGB only, do not open OpenNI2")
    parser.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS)
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"))
    parser.add_argument("--model", default=DEFAULT_MODEL, help="YOLO weights (default: %(default)s)")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF, help="min detection confidence")
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ, help="YOLO input size")
    parser.add_argument("--color-index", type=int, default=0, help="OpenCV index of the Astra RGB camera")
    parser.add_argument("--intrinsics", default=default_intrinsics_file(),
                        help="JSON {fx, fy, cx, cy} of the RGB camera at 640x480 (default: %(default)s)")
    parser.add_argument("--no-floor", action="store_true", help="no Depth Anything free-floor overlay")
    parser.add_argument("--da-size", type=int, default=DEFAULT_INPUT_SIZE,
                        help="Depth Anything input short side, multiple of 14 (default: %(default)s)")
    parser.add_argument("--floor-far", type=float, default=DEFAULT_FAR_M, help="how far to look for floor, m")
    parser.add_argument("--cam-height", type=float, default=mount.height_m, help="camera height, m (default: %(default)s)")
    parser.add_argument("--cam-pitch", type=float, default=mount.pitch_deg,
                        help="camera pitch, deg, + = tilted down (default: %(default)s)")
    parser.add_argument("--cam-forward", type=float, default=mount.forward_m,
                        help="camera ahead of the robot center, m (default: %(default)s)")
    parser.add_argument("--cam-left", type=float, default=mount.left_m,
                        help="camera left of the robot center, m (default: %(default)s)")
    parser.add_argument("--no-raw", action="store_true", help=f"do not save the undrawn frames to OUT/{RAW_DIR}/")
    parser.add_argument("--raw-quality", type=int, default=DEFAULT_JPEG_QUALITY,
                        help="JPEG quality of the raw frames (default: %(default)s)")
    return parser.parse_args()


def main():
    args = _parse_args()
    os.makedirs(args.out, exist_ok=True)
    status_file = args.status_file or os.path.join(args.out, STATUS_NAME)
    ready_file = args.ready_file or os.path.join(args.out, READY_NAME)
    if os.path.exists(ready_file):
        os.remove(ready_file)            # a stale one would release the orchestrator too early

    # Installed before the slow model load: a stop that arrives during it ends the run right after.
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    device = select_device(args.device)
    if args.intrinsics:
        intrinsics = load_intrinsics(args.intrinsics)
    else:
        intrinsics = fallback_intrinsics(FRAME_W, FRAME_H)
    mount = CameraMount(args.cam_height, args.cam_pitch, args.cam_forward, args.cam_left)
    floor_cfg = FloorConfig(far_m=args.floor_far, half_width_m=None)
    detector = Detector(args.model, device, args.conf, args.imgsz)
    floor_worker = _open_floor(not args.no_floor, device, args.da_size, mount, floor_cfg, args.out)
    raw_writer = None
    color_src = ColorSource(args.color_index)
    if not args.no_raw:
        raw_writer = RawFrameWriter(args.out, args.raw_quality)
        raw_writer.start()
        color_src.listener = raw_writer.offer   # before start(): no frame may arrive unnumbered on disk
    color_src.start()
    pipeline = Pipeline(detector, device, color_src, _open_depth(not args.no_depth), floor_worker, raw_writer,
                        mount, intrinsics, bool(args.intrinsics))
    _write_recording(args.out, args, pipeline, floor_cfg)
    sink = VideoSink(args.out, ready_file)
    recorder = DetectionRecorder(pipeline, sink, status_file)
    clean = True
    try:
        recorder.run(stop, args.max_seconds)
    finally:
        sink.close()
        clean = pipeline.close()
        _write_recording(args.out, args, pipeline, floor_cfg, sink, recorder.light.result())
    exit_code = 0 if sink.frames else EXIT_NO_FRAME
    if not clean:
        # A grabber is stuck inside a driver call; a normal interpreter exit could wait on it.
        logger.warning("exiting hard: a camera thread did not stop")
        logging.shutdown()
        os._exit(exit_code)
    sys.exit(exit_code)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    main()
