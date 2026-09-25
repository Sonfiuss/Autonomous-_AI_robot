"""Depth Anything V2 Small: relative inverse depth from one RGB frame.

Used for one job only: finding free floor where the Astra returns no depth. On glossy tiles seen from
0.24 m the IR dot pattern hits the floor at 10-20 deg and the depth chip cannot match it - 0% depth
from 0.6 to 1.0 m on the dev room's floor - while the RGB image shows that floor plainly.

The output is DISPARITY-LIKE (bigger = closer) and affine-invariant: unknown scale and shift, both
changing from frame to frame. It is not metric and is never turned into metres here; floor_segment.py
only compares it with the floor plane fitted to the same frame.

Small is Apache-2.0. Base and Large are CC-BY-NC-4.0 - do not swap them in for anything commercial.
"""
import collections
import logging
import os
import threading
import time

import cv2
import numpy as np

# transformers imports TensorFlow too when it is installed: seconds of startup and CUDA warnings, for
# a model this module never uses.
os.environ.setdefault("USE_TF", "0")

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_ID = "depth-anything/Depth-Anything-V2-Small-hf"
DEFAULT_MODEL_DIR = os.path.join(BASE_DIR, "models", "depth-anything-v2-small")   # git-ignored
DEFAULT_INPUT_SIZE = 518     # short side, multiple of 14. 308 is ~4.5x faster on CPU but on the real
                             # floor put phantom obstacles along the far edge and on a floor cable
CLOSE_JOIN_S = 5.0           # one CPU inference at 518 is ~3.5 s on the dev laptop

# seq: the number submit() returned for the frame this came from. t: time.monotonic() when done.
DepthResult = collections.namedtuple("DepthResult", "disparity seq t")

logger = logging.getLogger(__name__)


class MonoDepth:
    def __init__(self, device, input_size=DEFAULT_INPUT_SIZE, model_dir=DEFAULT_MODEL_DIR):
        import torch   # deferred with transformers: replay with saved disparity needs neither
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation

        local = os.path.isdir(model_dir)
        source = model_dir if local else MODEL_ID
        self._processor = AutoImageProcessor.from_pretrained(source, size={"height": input_size, "width": input_size})
        model = AutoModelForDepthEstimation.from_pretrained(source)
        if not local:
            # One download, then offline: the robot will not always have a network.
            model.save_pretrained(model_dir)
            self._processor.save_pretrained(model_dir)
            logger.info("Depth Anything weights saved to %s", model_dir)
        self._torch = torch
        self._fp16 = device.startswith("cuda")
        self._model = (model.half() if self._fp16 else model).to(device).eval()
        self._device = device
        self.infer(np.zeros((480, 640, 3), np.uint8))   # lazy init / CUDA context, out of the first capture
        logger.info("Depth Anything V2 Small on %s (fp16=%s, input %d)", device, self._fp16, input_size)

    def infer(self, bgr):
        """(H, W) float32 relative inverse depth at the frame's own resolution; bigger = closer."""
        inputs = self._processor(images=cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), return_tensors="pt")
        pixels = inputs["pixel_values"].to(self._device)
        with self._torch.no_grad():
            out = self._model(pixel_values=pixels.half() if self._fp16 else pixels).predicted_depth
            out = self._torch.nn.functional.interpolate(out[:, None].float(), size=bgr.shape[:2],
                                                        mode="bilinear", align_corners=False)
        return out[0, 0].cpu().numpy()


class MonoDepthWorker:
    """Runs a MonoDepth on its own daemon thread, which is the only thread touching the model.

    One frame waits at most: submit() replaces a frame not yet started, so the live overlay lags by
    one inference, never by a queue. A capture calls submit() then wait(), with the main loop blocked
    in between - nothing can replace its frame.
    """

    def __init__(self, mono):
        self._mono = mono
        self._cond = threading.Condition()
        self._pending = None
        self._result = None      # last SUCCESSFUL DepthResult
        self._done_seq = 0       # last submission finished, failed or not
        self._seq = 0
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, name="mono-depth", daemon=True)
        self._thread.start()

    def submit(self, bgr):
        with self._cond:
            self._seq += 1
            self._pending = (self._seq, bgr)
            self._cond.notify_all()
            return self._seq

    def latest(self):
        with self._cond:
            return self._result

    def wait(self, seq, timeout_s):
        """The disparity of submission `seq`, or None on timeout / a failed inference."""
        with self._cond:
            self._cond.wait_for(lambda: not self._running or self._done_seq >= seq, timeout=timeout_s)
            return self._result.disparity if self._result is not None and self._result.seq == seq else None

    def close(self):
        with self._cond:
            self._running = False
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=CLOSE_JOIN_S)

    def _run(self):
        while True:
            with self._cond:
                self._cond.wait_for(lambda: self._pending is not None or not self._running)
                if not self._running:
                    return
                seq, bgr = self._pending
                self._pending = None
            try:
                disparity = self._mono.infer(bgr)
            except Exception:
                logger.exception("Depth Anything inference failed")
                disparity = None
            with self._cond:
                if disparity is not None:
                    self._result = DepthResult(disparity, seq, time.monotonic())
                self._done_seq = seq   # a failure still ends the wait of a capture blocked on it
                self._cond.notify_all()
