"""Background frame grabbing shared by the depth and color sources, plus RGB/depth pairing.

Each camera is read on its own daemon thread that owns the driver handle end to end (open, read,
close) and keeps only the last few frames. The processing loop never calls into a driver: it
polls the grabbers. A driver call that blocks (USB hiccup while the camera is moved fast, cable
pulled) can therefore stall a grabber thread, never the loop, the window or the quit key.

Every frame is numbered where it arrives (seq), so the streams that consume it - the raw writer,
YOLO, Depth Anything, the video - can say exactly which frame each of their outputs came from.
"""
import collections
import logging
import threading
import time

# t = host time.monotonic() when the frame was received. Both cameras are stamped on the same
# host clock; the devices share no clock we could use instead (UVC gives none through OpenCV).
# seq = this source's count of frames received, from 0, never reused across a reopen. wall =
# time.time() at the same instant as t, the clock the drive orchestrator's run.json uses.
# t_sensor = when the DRIVER stamped the frame, on the time.monotonic() clock (V4L2 buffer time),
# None where the source has none. Arrival (t) jitters by up to ~250 ms while YOLO / Depth
# Anything hold the CPU - frames wait in the driver's buffers and then come out 2 ms apart - the
# driver stamp does not: measured 2026-09-25 on the Astra RGB, 32-36 ms apart, ~34 ms before arrival.
# seq / wall / t_sensor are None on a frame not made by a grabber (a drawn canvas, a test).
Frame = collections.namedtuple("Frame", "data t seq wall t_sensor", defaults=(None, None, None))

RING_SIZE = 6              # frames kept per source for nearest-timestamp pairing (~200 ms @30 fps)
SENSOR_MAX_LAG_S = 1.0     # a driver stamp further than this before arrival (or after it) is not trusted
STALL_RESTART_S = 2.0      # no frame for this long -> close and reopen the source
RESTART_BACKOFF_S = 1.0    # pause between reopen attempts
READ_ERROR_PAUSE_S = 0.05  # keeps a persistently failing read from spinning the CPU
CLOSE_JOIN_S = 2.0         # how long close() waits for the thread before abandoning it

logger = logging.getLogger(__name__)


def capture_wall(frame):
    """Wall-clock time (time.time()) the frame was captured: the driver's stamp moved onto the wall
    clock where the source has one, its arrival otherwise."""
    if frame.t_sensor is None:
        return frame.wall
    return frame.wall - (frame.t - frame.t_sensor)


class LatestFrameGrabber:
    """Runs a source on a daemon thread and keeps its newest frames.

    Subclasses implement _open(), _read() and _close(), all called on the grabber thread only.
    _read() returns an array, or None when nothing arrived within its own short timeout; where
    the driver offers a timeout it must use it. An exception from any of the three is logged and
    handled like a stall: close, back off, reopen. A subclass whose driver stamps frames may
    override _sensor_time(), called right after a successful _read(); a stamp more than
    SENSOR_MAX_LAG_S from arrival is dropped (t_sensor None).

    listener, if set before start(), is called with EVERY frame on the grabber thread - also the
    ones the ring drops before anyone polls them. It must return at once (hand the frame to a
    queue); an exception from it is logged and the frame still reaches the ring.
    """

    name = "source"

    def __init__(self):
        self._lock = threading.Lock()
        self._frames = collections.deque(maxlen=RING_SIZE)
        self._running = False
        self._thread = None
        self._seq = 0
        self._listener_failures = 0
        self.listener = None
        self.restarts = 0

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, name=self.name, daemon=True)
        self._thread.start()

    def latest(self):
        with self._lock:
            return self._frames[-1] if self._frames else None

    def frames(self):
        with self._lock:
            return list(self._frames)

    def close(self):
        """Stops the thread. False when it is stuck in a driver call and had to be abandoned -
        the caller must then not tear down anything that thread may still be using."""
        self._running = False
        if self._thread is None:
            return True
        self._thread.join(timeout=CLOSE_JOIN_S)
        if self._thread.is_alive():
            logger.warning("%s: thread stuck in a driver call after %.0fs, abandoning it",
                           self.name, CLOSE_JOIN_S)
            return False
        return True

    def _open(self):
        raise NotImplementedError

    def _read(self):
        raise NotImplementedError

    def _close(self):
        raise NotImplementedError

    def _sensor_time(self):
        """The driver's time.monotonic() stamp of the frame _read() just returned, or None."""
        return None

    def _run(self):
        while self._running:
            try:
                self._open()
            except Exception as exc:
                logger.warning("%s: open failed: %s - retrying in %.0fs", self.name, exc, RESTART_BACKOFF_S)
                self._safe_close()   # a half-finished open still holds handles the retry would trip on
                time.sleep(RESTART_BACKOFF_S)
                continue
            stalled = self._pump()
            self._safe_close()
            if stalled and self._running:
                self.restarts += 1
                logger.warning("%s: no frame for %.0fs - reopening (restart #%d)",
                               self.name, STALL_RESTART_S, self.restarts)
                time.sleep(RESTART_BACKOFF_S)

    def _safe_close(self):
        try:
            self._close()
        except Exception as exc:
            logger.warning("%s: close failed: %s", self.name, exc)

    def _pump(self):
        """Reads until stopped (returns False) or stalled (returns True)."""
        last_frame_t = time.monotonic()
        while self._running:
            try:
                data = self._read()
            except Exception as exc:
                logger.warning("%s: read failed: %s", self.name, exc)
                data = None
                time.sleep(READ_ERROR_PAUSE_S)
            now = time.monotonic()
            if data is not None:
                t_sensor = self._sensor_time()
                if t_sensor is not None and not 0.0 <= now - t_sensor <= SENSOR_MAX_LAG_S:
                    t_sensor = None
                frame = Frame(data, now, self._seq, time.time(), t_sensor)
                self._seq += 1
                self._notify(frame)   # before the ring: a listener sees a frame before any consumer can
                with self._lock:
                    self._frames.append(frame)
                last_frame_t = now
            elif now - last_frame_t > STALL_RESTART_S:
                return True
        return False

    def _notify(self, frame):
        if self.listener is None:
            return
        try:
            self.listener(frame)
        except Exception:
            # The camera goes on without whatever listens. One traceback, not one per frame.
            self._listener_failures += 1
            if self._listener_failures == 1:
                logger.exception("%s: frame listener failed", self.name)


def pair_frames(color, depth_frames, now, max_skew_s, max_age_s):
    """Pairs the newest color frame with the depth frame nearest to it in time.

    Returns ((color, depth), "OK") or (None, reason). Refusing a pair is the safe answer: a depth
    frame a few tens of ms off its color frame puts every bbox onto the wrong pixels as soon as the
    camera moves, which reads as a confident but wrong distance.
    """
    if color is None or now - color.t > max_age_s:
        return None, "NO COLOR"
    if not depth_frames or now - depth_frames[-1].t > max_age_s:
        return None, "NO DEPTH"
    depth = min(depth_frames, key=lambda f: abs(f.t - color.t))
    if abs(depth.t - color.t) > max_skew_s:
        return None, "OUT OF SYNC"
    return (color, depth), "OK"
