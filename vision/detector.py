"""YOLO object detector (80 COCO classes) with CUDA-first device selection."""
import collections
import logging
import os

import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(BASE_DIR, "models", "yolo11n.pt")   # weights are git-ignored
DEFAULT_CONF = 0.5      # a bare wall gave cat 0.44 / tv 0.34 on the dev laptop; keep those out
DEFAULT_IMGSZ = 640     # laptop CPU: ~175 ms @640, ~117 ms @480; small far objects need the 640

Detection = collections.namedtuple("Detection", "class_id name conf box")   # box = (x1, y1, x2, y2) px

logger = logging.getLogger(__name__)


def select_device(requested, cuda_available=None):
    """'auto' -> CUDA when torch sees a GPU, else CPU. An explicit 'cuda' with no GPU is an error,
    not a silent CPU fallback: on the Jetson that would hide a broken CUDA install behind a ~10x
    slowdown that only shows up as a sluggish robot."""
    if cuda_available is None:
        import torch
        cuda_available = torch.cuda.is_available()
    if requested == "auto":
        return "cuda:0" if cuda_available else "cpu"
    if requested == "cuda":
        if not cuda_available:
            raise RuntimeError("--device cuda requested but torch.cuda.is_available() is False "
                               "(CPU-only torch build, or the CUDA driver is not visible)")
        return "cuda:0"
    if requested == "cpu":
        return "cpu"
    raise ValueError(f"unknown device '{requested}' (auto | cpu | cuda)")


def _precision_args(fp16):
    """ultralytics >= 8.4 takes quantize=16 and warns on every call given half=True; older
    releases (what a Jetson image may ship) only know half. FP32 is the default in both."""
    if not fp16:
        return {}
    from ultralytics.cfg import DEFAULT_CFG_DICT
    return {"quantize": 16} if "quantize" in DEFAULT_CFG_DICT else {"half": True}


class Detector:
    def __init__(self, model_path, device, conf=DEFAULT_CONF, imgsz=DEFAULT_IMGSZ):
        from ultralytics import YOLO   # deferred: importing ultralytics costs seconds
        self._model = YOLO(model_path)
        self._device = device
        fp16 = device.startswith("cuda")   # fp16 roughly doubles throughput on the Jetson GPU
        self._predict_args = dict(device=device, conf=conf, imgsz=imgsz, verbose=False, **_precision_args(fp16))
        self.names = self._model.names
        # First call pays for lazy init / CUDA context; keep that out of the first real cycle.
        self.detect(np.zeros((480, 640, 3), dtype=np.uint8))
        logger.info("%s on %s (fp16=%s, conf=%.2f, imgsz=%d)",
                    os.path.basename(model_path), device, fp16, conf, imgsz)

    def detect(self, bgr):
        result = self._model.predict(bgr, **self._predict_args)[0]
        boxes = result.boxes
        return [Detection(int(c), self.names[int(c)], float(s), tuple(float(v) for v in b))
                for c, s, b in zip(boxes.cls.tolist(), boxes.conf.tolist(), boxes.xyxy.tolist())]
