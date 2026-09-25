"""Background frame grabbing shared by the depth and color sources, plus RGB/depth pairing.

Each camera is read on its own daemon thread that owns the driver handle end to end (open, read,
close) and keeps only the last few frames. The processing loop never calls into a driver: it
polls the grabbers. A driver call that blocks (USB hiccup while the camera is moved fast, cable
pulled) can therefore stall a grabber thread, never the loop, the window or the quit key.
"""
import collections
import logging
import threading
import time

# t = host time.monotonic() when the frame was received. Both cameras are stamped on the same
# host clock; the devices share no clock we could use instead (UVC gives none through OpenCV).
Frame = collections.namedtuple("Frame", "data t")

RING_SIZE = 6              # frames kept per source for nearest-timestamp pairing (~200 ms @30 fps)
STALL_RESTART_S = 2.0      # no frame for this long -> close and reopen the source
RESTART_BACKOFF_S = 1.0    # pause between reopen attempts
READ_ERROR_PAUSE_S = 0.05  # keeps a persistently failing read from spinning the CPU
CLOSE_JOIN_S = 2.0         # how long close() waits for the thread before abandoning it

logger = logging.getLogger(__name__)


class LatestFrameGrabber:
    """Runs a source on a daemon thread and keeps its newest frames.

    Subclasses implement _open(), _read() and _close(), all called on the grabber thread only.
    _read() returns an array, or None when nothing arrived within its own short timeout; where
    the driver offers a timeout it must use it. An exception from any of the three is logged and
    handled like a stall: close, back off, reopen.
    """

    name = "source"

    def __init__(self):
        self._lock = threading.Lock()
        self._frames = collections.deque(maxlen=RING_SIZE)
        self._running = False
        self._thread = None
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
                with self._lock:
                    self._frames.append(Frame(data, now))
                last_frame_t = now
            elif now - last_frame_t > STALL_RESTART_S:
                return True
        return False


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
