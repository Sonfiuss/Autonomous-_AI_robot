"""Astra Pro RGB via OpenCV.

On the Astra Pro the color camera is its own UVC device ("Astra Pro HD Camera", USB PID 0x0501,
separate from the depth PID 0x0403), so it is read like a webcam, not through OpenNI2.
"""
import sys

import cv2

from frame_grabber import LatestFrameGrabber

WIDTH, HEIGHT = 640, 480   # must match depth_source: D2C registers depth onto this exact grid
MS_PER_S = 1000.0


class ColorSource(LatestFrameGrabber):
    """BGR frames, HEIGHT x WIDTH x 3."""

    name = "color"

    def __init__(self, index):
        super().__init__()
        self._index = index
        self._cap = None

    def _open(self):
        backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_V4L2
        self._cap = cv2.VideoCapture(self._index, backend)
        if not self._cap.isOpened():
            raise RuntimeError(f"cannot open camera index {self._index}")
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        got = (int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if got != (WIDTH, HEIGHT):
            raise RuntimeError(f"camera index {self._index} gives {got[0]}x{got[1]}, "
                               f"registration needs {WIDTH}x{HEIGHT} - wrong device index?")

    def _read(self):
        ok, image = self._cap.read()
        return image if ok else None

    def _sensor_time(self):
        """V4L2's buffer timestamp, which OpenCV reports as the position in ms. It is on
        CLOCK_MONOTONIC, the clock of time.monotonic(). DSHOW reports something else: none."""
        if sys.platform == "win32":
            return None
        ms = self._cap.get(cv2.CAP_PROP_POS_MSEC)
        return ms / MS_PER_S if ms > 0 else None

    def _close(self):
        cap, self._cap = self._cap, None
        if cap is not None:
            cap.release()
