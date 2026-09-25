"""Unit tests that need neither the camera nor the model. Run: python test_vision.py

Covers object_distance (the numbers), pair_frames (the sync gate), select_device (the CUDA
option), LatestFrameGrabber against fake sources that stall, fail and hang - the failure
modes seen when the camera is moved fast - and the frame numbering the recorder's streams share
(seq, the raw writer, the floor log).
"""
import json
import math
import os
import sys
import tempfile
import threading
import time

import cv2
import numpy as np

import frame_grabber
from detector import Detection, select_device
from drawing import format_range
from floor_geometry import CameraMount
from floor_segment import FloorView, box_floor_range
from frame_grabber import Frame, LatestFrameGrabber, capture_wall, pair_frames
from object_distance import Intrinsics, intrinsics_from_fov, measure_object, result_records
from raw_recorder import FILE_PATTERN, INDEX_NAME, RawFrameWriter

H, W = 480, 640
INTR = Intrinsics(fx=570.0, fy=570.0, cx=319.5, cy=239.5)
BOX = (220.0, 140.0, 420.0, 340.0)   # 200x200 box centered on the principal point
CORE = (slice(190, 290), slice(270, 370))   # its central 50%
TOL = 1e-6


def _depth(value_mm=0):
    return np.full((H, W), value_mm, dtype=np.uint16)


def _fill_core(depth, fractions_values):
    """Fills the core of BOX row-wise: [(fraction, mm), ...]."""
    core = depth[CORE]
    flat = core.reshape(-1)
    start = 0
    for fraction, mm in fractions_values:
        n = int(round(fraction * flat.size))
        flat[start:start + n] = mm
        start += n
    depth[CORE] = flat.reshape(core.shape)
    return depth


def check_object_distance():
    errors = []
    r = measure_object(_depth(1500), BOX, INTR)
    half_px_m = 0.5 * 1.5 / INTR.fx   # BOX is centered on pixel 320/240, the principal point is 319.5/239.5
    if (r is None or abs(r.z_m - 1.5) > TOL
            or abs(r.xyz[0] - half_px_m) > TOL or abs(r.xyz[1] - half_px_m) > TOL):
        errors.append(f"uniform 1500 mm near the axis: {r}")

    r = measure_object(_fill_core(_depth(), [(0.4, 1200), (0.6, 3000)]), BOX, INTR)
    if r is None or abs(r.z_m - 1.2) > TOL:
        errors.append(f"chair-like 40% object @1.2 m, 60% wall @3 m behind -> expected 1.2 m, got {r}")

    r = measure_object(_fill_core(_depth(), [(0.02, 300), (0.98, 2000)]), BOX, INTR)
    if r is None or abs(r.z_m - 2.0) > TOL:
        errors.append(f"2% speckle @0.3 m in front of 2 m surface -> expected 2.0 m, got {r}")

    r = measure_object(_fill_core(_depth(), [(0.2, 1500)]), BOX, INTR)
    if r is not None:
        errors.append(f"only 20% valid core pixels should give no reading, got {r}")

    six = [(1 / 6, 1000 + 500 * i) for i in range(6)]
    r = measure_object(_fill_core(_depth(), six), BOX, INTR)
    if r is not None:
        errors.append(f"six clusters of ~17% (none reaches 20%) should give no reading, got {r}")

    tilted = _depth()
    tilted[CORE] = np.linspace(1000, 2500, CORE[1].stop - CORE[1].start).astype(np.uint16)[None, :]
    r = measure_object(tilted, BOX, INTR)
    if r is None or abs(r.z_m - 1.75) > 0.02:
        errors.append(f"continuous 1.0-2.5 m slope (table at a grazing angle) -> one surface ~1.75 m, got {r}")

    z = 2.0
    u = INTR.cx + INTR.fx * 0.5   # x = 0.5 * z
    side = _depth(2000)
    r = measure_object(side, (u - 20, INTR.cy - 20, u + 20, INTR.cy + 20), INTR)
    if r is None or abs(r.xyz[0] - 1.0) > 1e-3 or abs(r.range_m - math.sqrt(1.0 + z * z)) > 1e-3:
        errors.append(f"off-axis object: expected x=1.0 m range={math.sqrt(5):.3f} m, got {r}")

    r = measure_object(_depth(1500), (600.0, 100.0, 700.0, 200.0), INTR)
    if r is None or abs(r.z_m - 1.5) > TOL:
        errors.append(f"box half outside the image should still measure its visible part, got {r}")
    if measure_object(_depth(1500), (700.0, 100.0, 800.0, 200.0), INTR) is not None:
        errors.append("box entirely outside the image should give no reading")
    r = measure_object(_depth(1500), (300.2, 200.2, 300.6, 200.6), INTR)
    if r is None:
        errors.append("sub-pixel box inside the image should still sample one pixel")

    fx = intrinsics_from_fov(640, 480, math.radians(90), math.radians(90)).fx
    if abs(fx - 320.0) > 1e-6:
        errors.append(f"intrinsics_from_fov 90 deg @640 px: expected fx=320, got {fx}")
    return errors


def check_pair_frames():
    errors = []
    now = 100.0
    color = Frame("c", now - 0.010)
    depths = [Frame("d0", now - 0.080), Frame("d1", now - 0.045), Frame("d2", now - 0.012), Frame("d3", now - 0.001)]
    pair, status = pair_frames(color, depths, now, 0.040, 0.5)
    if status != "OK" or pair[1].data != "d2":
        errors.append(f"should pick the depth nearest in time (d2), not the newest: {status} {pair}")
    cases = [
        ((None, depths), "NO COLOR"),
        ((Frame("c", now - 1.0), depths), "NO COLOR"),
        ((color, []), "NO DEPTH"),
        ((color, [Frame("d", now - 1.0)]), "NO DEPTH"),
        ((Frame("c", now - 0.100), [Frame("d", now - 0.010)]), "OUT OF SYNC"),
    ]
    for (c, d), expected in cases:
        pair, status = pair_frames(c, d, now, 0.040, 0.5)
        if status != expected or pair is not None:
            errors.append(f"expected {expected}, got {status} {pair}")
    return errors


def check_select_device():
    errors = []
    expectations = [(("auto", True), "cuda:0"), (("auto", False), "cpu"), (("cuda", True), "cuda:0"),
                    (("cpu", True), "cpu")]
    for args, expected in expectations:
        got = select_device(*args)
        if got != expected:
            errors.append(f"select_device{args}: expected {expected}, got {got}")
    for args, exc_type in ((("cuda", False), RuntimeError), (("gpu", True), ValueError)):
        try:
            select_device(*args)
            errors.append(f"select_device{args} should raise {exc_type.__name__}")
        except exc_type:
            pass
    return errors


class _FakeSource(LatestFrameGrabber):
    """Scripted source. behaviour(read_count) -> 'frame' | 'none' | 'raise' | 'hang'."""

    name = "fake"

    def __init__(self, behaviour, open_failures=0):
        super().__init__()
        self._behaviour = behaviour
        self._open_failures = open_failures
        self.opens = 0
        self.closes = 0
        self.reads = 0
        self._hang = threading.Event()

    def _open(self):
        self.opens += 1
        if self.opens <= self._open_failures:
            raise RuntimeError("scripted open failure")

    def _read(self):
        self.reads += 1
        action = self._behaviour(self.reads)
        if action == "frame":
            time.sleep(0.005)
            return self.reads
        if action == "raise":
            raise RuntimeError("scripted read failure")
        if action == "hang":
            self._hang.wait()   # like a driver call with no timeout
        time.sleep(0.01)
        return None

    def _close(self):
        self.closes += 1


def _run_fake(source, seconds):
    source.start()
    time.sleep(seconds)
    t = time.monotonic()
    clean = source.close()
    return clean, time.monotonic() - t


def check_grabber():
    errors = []
    saved = (frame_grabber.STALL_RESTART_S, frame_grabber.RESTART_BACKOFF_S, frame_grabber.CLOSE_JOIN_S)
    frame_grabber.STALL_RESTART_S, frame_grabber.RESTART_BACKOFF_S, frame_grabber.CLOSE_JOIN_S = 0.2, 0.05, 0.3
    try:
        healthy = _FakeSource(lambda n: "frame")
        clean, _ = _run_fake(healthy, 0.2)
        if not clean or healthy.latest() is None or healthy.closes != 1 or len(healthy.frames()) > frame_grabber.RING_SIZE:
            errors.append(f"healthy: clean={clean} latest={healthy.latest()} closes={healthy.closes}")

        stall = _FakeSource(lambda n: "frame" if n <= 3 else "none")
        _run_fake(stall, 0.8)
        if stall.restarts < 1 or stall.opens < 2 or stall.closes < 2:
            errors.append(f"stall must close + reopen: restarts={stall.restarts} opens={stall.opens} closes={stall.closes}")

        flaky_open = _FakeSource(lambda n: "frame", open_failures=2)
        _run_fake(flaky_open, 0.5)
        if flaky_open.latest() is None or flaky_open.closes < 3:
            errors.append(f"failed opens must be cleaned up and retried: opens={flaky_open.opens} "
                          f"closes={flaky_open.closes} latest={flaky_open.latest()}")

        raising = _FakeSource(lambda n: "frame" if n <= 3 else "raise")
        _run_fake(raising, 0.8)
        if raising.restarts < 1:
            errors.append(f"read exceptions must count as a stall: restarts={raising.restarts}")

        hang = _FakeSource(lambda n: "frame" if n <= 3 else "hang")
        hang.start()
        time.sleep(0.2)
        t = time.monotonic()
        latest = hang.latest()   # must not block while the thread is stuck in "driver" code
        poll_s = time.monotonic() - t
        t = time.monotonic()
        clean = hang.close()
        close_s = time.monotonic() - t
        if latest is None or poll_s > 0.05:
            errors.append(f"latest() during a hang: {latest} after {poll_s:.3f}s")
        if clean or close_s > frame_grabber.CLOSE_JOIN_S + 0.2:
            errors.append(f"close() on a hung thread must give up (False) in ~{frame_grabber.CLOSE_JOIN_S}s: "
                          f"clean={clean} took {close_s:.2f}s")
        hang._hang.set()
    finally:
        frame_grabber.STALL_RESTART_S, frame_grabber.RESTART_BACKOFF_S, frame_grabber.CLOSE_JOIN_S = saved
    return errors


def check_frame_numbering():
    """Every frame gets the next seq and a wall time; the listener sees every one of them, before
    the ring does, and a listener that raises does not stop the camera."""
    errors = []
    source = _FakeSource(lambda n: "frame")
    seen, early = [], []

    def listener(frame):
        latest = source.latest()
        early.append(latest is None or latest.seq != frame.seq)
        seen.append(frame)
        if len(seen) == 3:
            raise RuntimeError("scripted listener failure")

    source.listener = listener
    t_before = time.time()
    _run_fake(source, 0.2)
    ring = source.frames()
    seqs = [f.seq for f in seen]
    if len(seen) < 10 or seqs != list(range(len(seen))):
        errors.append(f"listener must get every frame, seq 0,1,2,... without gaps: {seqs[:12]}")
    if not all(early):
        errors.append("the listener must see a frame before latest() can return it")
    if [f.seq for f in ring] != seqs[-len(ring):]:
        errors.append(f"ring seqs {[f.seq for f in ring]} must be the newest the listener saw")
    if not all(t_before <= f.wall <= time.time() for f in seen):
        errors.append("wall must be time.time() at arrival")
    if Frame("c", 1.0).seq is not None:
        errors.append("a Frame built by hand has no seq")
    if any(f.t_sensor is not None for f in seen) or capture_wall(seen[0]) != seen[0].wall:
        errors.append("a source without driver stamps: t_sensor None, capture time = arrival")

    for offset_s, trusted in ((0.034, True), (-0.5, False), (5.0, False)):
        stamped = _FakeSource(lambda n: "frame")
        stamped._sensor_time = lambda: time.monotonic() - offset_s
        _run_fake(stamped, 0.05)
        frame = stamped.latest()
        if (frame.t_sensor is not None) != trusted:
            errors.append(f"driver stamp {offset_s:+.3f} s from arrival: trusted={frame.t_sensor is not None}")
        elif trusted and abs(frame.wall - capture_wall(frame) - (frame.t - frame.t_sensor)) > 1e-6:   # ~2e-7 s: a double at 1.8e9 s
            errors.append("capture_wall must move the driver stamp onto the wall clock")
    return errors


def check_raw_writer():
    """A full queue drops frames and counts them; ensure() writes a dropped frame on the spot; the
    index holds exactly the frames on disk, with the grabber's timestamps."""
    errors = []
    # Arrival 34 ms after the driver stamp, as measured on the Astra RGB; frame 1 has no stamp.
    lag = {k: 0.034 for k in range(5)}
    lag[1] = None
    frames = [Frame(np.full((H, W, 3), 40 * k, np.uint8), 100.0 + k / 30.0, k, 1790000000.0 + k / 30.0,
                    None if lag[k] is None else 100.0 + k / 30.0 - lag[k]) for k in range(5)]
    with tempfile.TemporaryDirectory() as out:
        writer = RawFrameWriter(out, queue_frames=2)
        for frame in frames:                 # not started: the queue takes 0 and 1, drops 2, 3, 4
            writer.offer(frame)
        writer.ensure(frames[3])             # a frame YOLO used: written now
        writer.ensure(frames[0])             # still queued: left to the thread
        writer.start()
        writer.close()
        with open(os.path.join(writer.dir, INDEX_NAME), encoding="utf-8") as f:
            index = sorted((json.loads(line) for line in f), key=lambda r: r["seq"])
        if [r["seq"] for r in index] != [0, 1, 3]:
            errors.append(f"index seqs {[r['seq'] for r in index]}, expected [0, 1, 3]")
        if (writer.written, writer.dropped, writer.rescued) != (3, 2, 1):
            errors.append(f"written/dropped/rescued {writer.written}/{writer.dropped}/{writer.rescued}, expected 3/2/1")
        for r in index:
            frame = frames[r["seq"]]
            image = cv2.imread(os.path.join(writer.dir, r["file"]))
            if r["file"] != FILE_PATTERN.format(seq=r["seq"]) or image is None or image.shape != frame.data.shape:
                errors.append(f"seq {r['seq']}: file {r['file']} missing or wrong")
            elif abs(float(image.mean()) - float(frame.data.mean())) > 2.0:
                errors.append(f"seq {r['seq']}: image content changed on disk")
            stamped = lag[r["seq"]] is not None
            if (abs(r["t_capture"] - (frame.wall - (lag[r["seq"]] or 0.0))) > 1e-4
                    or (r["t_sensor"] is not None) != stamped
                    or (stamped and abs(r["t_sensor"] - frame.t_sensor) > 1e-4)
                    or abs(r["t_arrival"] - frame.t) > 1e-4):
                errors.append(f"seq {r['seq']}: timestamps {r['t_capture']}/{r['t_sensor']}/{r['t_arrival']} must be "
                              f"the driver stamp on the wall clock / on monotonic / the arrival")
    return errors


def check_floor_log():
    """floor/<seq>.png round-trips the labels; the index line names the analysed frame."""
    import record                            # the recorder's own module: YOLO / DA load only in main()
    errors = []
    labels = np.zeros((H, W), np.uint8)
    labels[300:, :] = 1
    labels[350:360, 100:200] = 2
    view = FloorView(labels, labels > 0, np.array([[150, 359]]), np.array([True]), None, (0.1, 0.0, -20.0))
    result = record.FloorResult(view, 70.0, 5.0, 1.234, 0.0, 42, 1790000000.12345)
    with tempfile.TemporaryDirectory() as out:
        log = record.FloorLog(out)
        log.write(result, [(100.0, 200.0, 200.0, 360.0)])
        log.close()
        with open(os.path.join(log.dir, INDEX_NAME), encoding="utf-8") as f:
            line = json.loads(f.readline())
        back = cv2.imread(os.path.join(log.dir, line["labels"]), cv2.IMREAD_UNCHANGED)
        if line["seq"] != 42 or line["labels"] != record.FLOOR_LABELS_PATTERN.format(seq=42):
            errors.append(f"index line must name seq 42: {line['seq']} {line['labels']}")
        if back is None or not np.array_equal(back, labels):
            errors.append("labels PNG must round-trip exactly")
        if abs(line["t_capture"] - 1790000000.1235) > 1e-6 or line["contacts"] != [[150, 359]]:
            errors.append(f"t_capture / contacts: {line['t_capture']} {line['contacts']}")
    return errors


def check_box_floor_range():
    """A box standing on the floor, straight ahead, level camera: its bottom row v sees the floor at
    z = h * fy / (v - cy). Cut-off boxes and boxes above the horizon give no estimate."""
    errors = []
    mount = CameraMount(height_m=0.24, pitch_deg=0.0, forward_m=0.0)
    v = INTR.cy + 114.0                                       # 0.24 * 570 / 114 = 1.2 m ahead
    r = box_floor_range((INTR.cx - 20, 100.0, INTR.cx + 20, v), H, INTR, mount)
    expected = math.hypot(1.2, 0.24)                          # camera -> contact point, Euclidean
    if r is None or abs(r.z_m - 1.2) > TOL or abs(r.range_m - expected) > TOL or r.valid_fraction is not None:
        errors.append(f"box bottom 114 px below the axis -> z 1.2 m, range {expected:.4f}; got {r}")
    if format_range(r) != f"~{expected:.2f} m":
        errors.append(f"an estimate must print with '~': {format_range(r)!r}")
    if box_floor_range((100.0, 100.0, 200.0, H - 1.0), H, INTR, mount) is not None:
        errors.append("a box cut off by the bottom edge has no visible foot")
    if box_floor_range((100.0, 50.0, 200.0, INTR.cy - 10), H, INTR, mount) is not None:
        errors.append("a box ending above the horizon cannot touch the floor")
    rows = result_records([(Detection(56, "chair", 0.9, (0.0, 0.0, 1.0, 1.0)), r)])
    if rows[0]["range_source"] != "floor" or rows[0]["valid_fraction"] is not None:
        errors.append(f"record of an estimate: {rows[0]}")
    return errors


def main():
    failed = 0
    for check in (check_object_distance, check_box_floor_range, check_pair_frames, check_select_device,
                  check_grabber, check_frame_numbering, check_raw_writer, check_floor_log):
        errors = check()
        print(f"{'PASS' if not errors else 'FAIL'} {check.__name__}")
        for e in errors:
            print("   ", e)
        failed += bool(errors)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
