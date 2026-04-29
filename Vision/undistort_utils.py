"""Utilities to undistort/dewarp lens distortion (pinhole or fisheye) with OpenCV.

Use-case: wide/fisheye cameras often show curved edges. Depth models (e.g. Depth-Anything-V2)
are typically trained on rectilinear images, so undistorting first usually improves results.

Intrinsics JSON format (example):
{
  "model": "pinhole" | "fisheye",
  "camera_matrix": [[fx,0,cx],[0,fy,cy],[0,0,1]],
  "dist_coeffs": [k1,k2,p1,p2,k3],
  "image_width": 1920,
  "image_height": 1080,
  "alpha": 0.0,         # pinhole only (0=crop, 1=keep FOV)
  "balance": 0.0,       # fisheye only (0=crop, 1=keep FOV)
  "crop": true
}
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Tuple

import cv2
import numpy as np


@dataclass
class Intrinsics:
    model: str  # "pinhole" or "fisheye"
    camera_matrix: np.ndarray  # 3x3
    dist_coeffs: np.ndarray  # (N,)
    image_width: Optional[int] = None
    image_height: Optional[int] = None
    alpha: float = 0.0  # pinhole: 0..1
    balance: float = 0.0  # fisheye: 0..1
    crop: bool = True


def load_intrinsics_json(path: str | Path) -> Intrinsics:
    p = Path(path)
    data = json.loads(p.read_text(encoding="utf-8"))

    model = str(data.get("model", "pinhole")).lower().strip()
    if model not in {"pinhole", "fisheye"}:
        raise ValueError(f"Unsupported intrinsics model: {model}. Use 'pinhole' or 'fisheye'.")

    camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
    dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64).reshape(-1)

    iw = data.get("image_width")
    ih = data.get("image_height")

    alpha = float(data.get("alpha", 0.0))
    balance = float(data.get("balance", 0.0))
    crop = bool(data.get("crop", True))

    return Intrinsics(
        model=model,
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_width=int(iw) if iw is not None else None,
        image_height=int(ih) if ih is not None else None,
        alpha=alpha,
        balance=balance,
        crop=crop,
    )


class Undistorter:
    """Frame undistorter with cached maps per (w,h).

    Notes:
      - For pinhole: uses getOptimalNewCameraMatrix + initUndistortRectifyMap.
      - For fisheye: uses estimateNewCameraMatrixForUndistortRectify + fisheye.initUndistortRectifyMap.
    """

    def __init__(self, intrinsics: Intrinsics):
        self.intr = intrinsics
        self._cache: dict[Tuple[int, int], tuple[np.ndarray, np.ndarray, tuple[int, int, int, int]]] = {}

    def _build_maps(self, w: int, h: int) -> tuple[np.ndarray, np.ndarray, tuple[int, int, int, int]]:
        key = (w, h)
        if key in self._cache:
            return self._cache[key]

        K = self.intr.camera_matrix
        D = self.intr.dist_coeffs

        if self.intr.model == "fisheye":
            R = np.eye(3, dtype=np.float64)
            newK = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D, (w, h), R, balance=float(self.intr.balance)
            )
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D, R, newK, (w, h), cv2.CV_16SC2
            )
            # Fisheye path doesn't provide ROI; use full frame.
            roi = (0, 0, w, h)
        else:
            newK, roi = cv2.getOptimalNewCameraMatrix(
                K, D, (w, h), alpha=float(self.intr.alpha), newImgSize=(w, h)
            )
            map1, map2 = cv2.initUndistortRectifyMap(
                K, D, None, newK, (w, h), cv2.CV_16SC2
            )
            if roi is None:
                roi = (0, 0, w, h)

        self._cache[key] = (map1, map2, roi)
        return map1, map2, roi

    def __call__(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        map1, map2, roi = self._build_maps(w, h)
        undist = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

        if self.intr.crop:
            x, y, rw, rh = roi
            if rw > 0 and rh > 0:
                undist = undist[y : y + rh, x : x + rw]
        return undist


def maybe_make_undistorter(intrinsics_path: Optional[str]) -> Optional[Undistorter]:
    if not intrinsics_path:
        return None
    intr = load_intrinsics_json(intrinsics_path)
    return Undistorter(intr)
