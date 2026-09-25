"""Every raw camera frame to disk, on its own thread: the stream every other output of a recording
points back into by frame_grabber.Frame.seq.

The grabber hands each frame to offer() before any consumer can poll it, so YOLO, Depth Anything and
the video only ever see frames that are already queued here or were counted as dropped. A consumer
that records an output for a frame calls ensure(frame): a dropped frame is then written on the
spot, so every seq named in detections.jsonl or floor/index.jsonl has its image on disk.

Written to <out>/raw/:
  <seq>.jpg      the frame exactly as the camera delivered it: no overlay, no resize (FILE_PATTERN)
  index.jsonl    one line per written frame: seq; t_capture, the wall clock (time.time(), s) of the
                 capture - the driver's stamp where there is one (frame_grabber.capture_wall), the
                 clock run.json uses; t_sensor, that stamp on time.monotonic() (null: none); t_arrival
                 (time.monotonic() when the grabber got it); file. Lines follow the writing order,
                 which is seq order except for a frame ensure() wrote late; readers sort by seq.
A seq absent from the index was dropped because the queue was full; close() logs how many.
"""
import json
import logging
import os
import queue
import threading

import cv2

from frame_grabber import capture_wall

RAW_DIR = "raw"
INDEX_NAME = "index.jsonl"
FILE_PATTERN = "{seq:06d}.jpg"
DEFAULT_JPEG_QUALITY = 90          # ~60 KB a 640x480 frame; artifacts well below what DA / ORB react to
QUEUE_FRAMES = 60                  # 2 s at 30 fps, ~55 MB of 640x480 BGR
CLOSE_TIMEOUT_S = 5.0              # close() waits this long for the queue to drain
CAPTURE_TIME_DIGITS = 4            # 0.1 ms: frames are 33 ms apart, and the map derives speeds from their gaps

logger = logging.getLogger(__name__)


class RawFrameWriter:
    """start() once; offer() from the grabber thread; ensure() from any consumer; close() once the
    grabber has stopped."""

    def __init__(self, out_dir, jpeg_quality=DEFAULT_JPEG_QUALITY, queue_frames=QUEUE_FRAMES):
        self.dir = os.path.join(out_dir, RAW_DIR)
        os.makedirs(self.dir, exist_ok=True)
        self.jpeg_quality = int(jpeg_quality)
        self._params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        self._queue = queue.Queue(maxsize=queue_frames)
        self._lock = threading.Lock()      # the index file, the dropped set and the counters
        # Line-buffered: a recorder that exits hard (camera thread stuck in its driver) keeps its index.
        self._index = open(os.path.join(self.dir, INDEX_NAME), "w", buffering=1, encoding="utf-8")
        self._dropped = set()              # seqs refused by a full queue and not written since
        self._thread = None
        self.written = 0
        self.rescued = 0                   # dropped frames ensure() wrote after all
        self.failed = 0                    # cv2.imwrite returned False

    @property
    def dropped(self):
        """Frames missing from disk: dropped and never ensured."""
        with self._lock:
            return len(self._dropped)

    def start(self):
        self._thread = threading.Thread(target=self._run, name="raw-writer", daemon=True)
        self._thread.start()

    def offer(self, frame):
        """Grabber thread: queue the frame, never block. A full queue drops it (counted)."""
        try:
            self._queue.put_nowait(frame)
        except queue.Full:
            with self._lock:
                self._dropped.add(frame.seq)

    def ensure(self, frame):
        """A consumer is about to record an output for this frame: make sure its image exists. A
        frame still queued is left to the writer thread; a dropped one is written now, on the
        caller's thread (~5 ms)."""
        with self._lock:
            if frame.seq not in self._dropped:
                return
            self._dropped.discard(frame.seq)
        if self._write(frame):
            with self._lock:
                self.rescued += 1

    def close(self):
        """Writes what is still queued (up to CLOSE_TIMEOUT_S), then closes the index. The grabber
        must be stopped first: a frame offered after this is never written."""
        if self._thread is not None:
            try:
                self._queue.put(None, timeout=CLOSE_TIMEOUT_S)   # after every queued frame
                self._thread.join(timeout=CLOSE_TIMEOUT_S)
            except queue.Full:
                pass
            if self._thread.is_alive():
                logger.warning("raw writer did not drain in %.0f s: ~%d frames not written", CLOSE_TIMEOUT_S,
                               self._queue.qsize())
        with self._lock:
            self._index.close()
            missing = len(self._dropped)
        logger.info("raw frames: %d written to %s, %d dropped (queue full), %d rescued for YOLO / DA, "
                    "%d failed", self.written, self.dir, missing, self.rescued, self.failed)

    def _run(self):
        while True:
            frame = self._queue.get()
            if frame is None:
                return
            self._write(frame)

    def _write(self, frame):
        name = FILE_PATTERN.format(seq=frame.seq)
        ok = cv2.imwrite(os.path.join(self.dir, name), frame.data, self._params)
        with self._lock:
            if not ok:
                self.failed += 1
                if self.failed == 1:
                    logger.error("raw writer: cannot write %s (disk full?)", os.path.join(self.dir, name))
                return False
            self._index.write(json.dumps({
                "seq": frame.seq, "t_capture": round(capture_wall(frame), CAPTURE_TIME_DIGITS),
                "t_sensor": None if frame.t_sensor is None else round(frame.t_sensor, CAPTURE_TIME_DIGITS),
                "t_arrival": round(frame.t, CAPTURE_TIME_DIGITS), "file": name}) + "\n")
            self.written += 1
        return True
