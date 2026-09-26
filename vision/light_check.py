"""Is the scene lit well enough for the vision to work? Median brightness and ORB keypoint count over
a few frames at the start of a recording.

A dark room (mean 4/255, 222 ORB keypoints on 2026-09-26) starves both Depth Anything and the
frame-to-frame motion estimate, and the output then looks like an algorithm bug. This puts the reason
in recording.json and the log instead. It never blocks a recording.
"""
import logging

import cv2
import numpy as np

LIGHT_SKIP_FRAMES = 60           # camera frames (seq) left to auto-exposure first: ~2 s at 30 fps
LIGHT_CHECK_FRAMES = 10          # frames measured: ~1 s of YOLO frames at record.VIDEO_FPS
MIN_MEAN_BRIGHTNESS = 40.0       # of 255; lit room 105, dark room 4 (2026-09-26)
MIN_ORB = 500                    # keypoints per frame; lit room 1978, dark room 222
ORB_MAX_FEATURES = 2000          # ORB's default cap of 500 would hide everything above MIN_ORB
MEAN_DIGITS = 1

logger = logging.getLogger(__name__)


class LightCheck:
    """add() frames until done; result() then holds the verdict. The warning is logged once."""

    def __init__(self, frames=LIGHT_CHECK_FRAMES, skip=LIGHT_SKIP_FRAMES):
        self._want = frames
        self._skip = skip
        self._means = []
        self._orbs = []
        self._orb = cv2.ORB_create(nfeatures=ORB_MAX_FEATURES)

    @property
    def done(self):
        return len(self._means) >= self._want

    def add(self, image_bgr, seq):
        """Measures one frame (seq: frame_grabber.Frame.seq); frames before `skip` and after the check
        is done are ignored."""
        if self.done or seq < self._skip:
            return
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        self._means.append(float(gray.mean()))
        self._orbs.append(len(self._orb.detect(gray, None)))
        if not self.done:
            return
        r = self.result()
        if not r["ok"]:
            logger.warning("scene too dark for vision: brightness %.0f/255 (min %.0f), %d ORB keypoints "
                           "(min %d) - turn the lights on", r["mean"], MIN_MEAN_BRIGHTNESS, r["orb"], MIN_ORB)

    def result(self):
        """{mean, orb, frames, ok, min_mean, min_orb}; None before the first measured frame. Medians,
        so one frame caught mid auto-exposure does not decide it."""
        if not self._means:
            return None
        mean = float(np.median(self._means))
        orb = int(np.median(self._orbs))
        return {"mean": round(mean, MEAN_DIGITS), "orb": orb, "frames": len(self._means),
                "ok": mean >= MIN_MEAN_BRIGHTNESS and orb >= MIN_ORB,
                "min_mean": MIN_MEAN_BRIGHTNESS, "min_orb": MIN_ORB}
