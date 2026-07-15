#!/usr/bin/env python3
"""calib_io — shared calibration loading / rectification / detection helpers
for the stereo accuracy tools (epipolar_check.py, depth_eval.py, sgbm_probe.py).

Supports BOTH yml schemas:
  old (2026-07-02): single K + dist shared by both cameras
  new (Stage B):    K_left / K_right / dist_left / dist_right per camera
plus the optional empirical scale keys fb_measured / fb_offset_px (Stage C).

Pure cv2 + numpy — runs identically on the Windows analysis PC and the Jetson.
"""
import csv
import os
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np


# ------------------------------------------------------------------ calib ----
def load_calib(path):
    """Read a stereo_rectify yml (old or new schema) → namespace with rectify
    maps and rectified projection intrinsics.

    Returns SimpleNamespace(map_l, map_r, fb, fb_used, fb_measured, fb_offset_px,
    fx, fy, cx, cy, size, schema, path). `fb_used` prefers fb_measured when
    present (the tape-measure-corrected value beats the calib-derived one).
    """
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"cannot open calib yml: {path}")

    def mat(name):
        n = fs.getNode(name)
        return None if n.empty() else n.mat()

    def real(name):
        n = fs.getNode(name)
        return None if n.empty() else float(n.real())

    w, h = real("image_width"), real("image_height")
    if not w or not h:
        raise ValueError(f"{path}: missing image_width/image_height")
    size = (int(w), int(h))

    K_l, K_r = mat("K_left"), mat("K_right")
    if K_l is not None and K_r is not None:            # new per-camera schema
        d_l, d_r, schema = mat("dist_left"), mat("dist_right"), "per-camera"
    else:                                              # old shared-K schema
        K_l = K_r = mat("K")
        d_l = d_r = mat("dist")
        schema = "single-K"
    R1, R2, P1, P2 = mat("R1"), mat("R2"), mat("P1"), mat("P2")
    fb = real("fb")
    fb_measured = real("fb_measured")
    fb_offset_px = real("fb_offset_px") or 0.0
    fs.release()
    if any(m is None for m in (K_l, d_l, R1, R2, P1, P2)):
        raise ValueError(f"{path}: missing K/dist/R1/R2/P1/P2 nodes")

    map_l = cv2.initUndistortRectifyMap(K_l, d_l, R1, P1, size, cv2.CV_32FC1)
    map_r = cv2.initUndistortRectifyMap(K_r, d_r, R2, P2, size, cv2.CV_32FC1)
    return SimpleNamespace(
        map_l=map_l, map_r=map_r, fb=fb, fb_measured=fb_measured,
        fb_offset_px=fb_offset_px,
        fb_used=fb_measured if fb_measured else fb,
        fx=float(P2[0, 0]), fy=float(P2[1, 1]),
        cx=float(P2[0, 2]), cy=float(P2[1, 2]),
        size=size, schema=schema, path=str(path))


def rectify_pair(calib, left_bgr, right_bgr):
    l = cv2.remap(left_bgr, calib.map_l[0], calib.map_l[1], cv2.INTER_LINEAR)
    r = cv2.remap(right_bgr, calib.map_r[0], calib.map_r[1], cv2.INTER_LINEAR)
    return l, r


# --------------------------------------------------------------- detection ---
def find_board(gray, pattern=(9, 6)):
    """Full-board chessboard corners, subpixel. Returns (N,2) float32 in
    row-major board order, or None.

    SB detector for robust ordered detection, then cornerSubPix refinement:
    SB's own subpixel output pixel-locks up to ±0.23 px at quarter-pixel
    phases (measured in depth_eval --selftest); the gradient-based refine
    brings that to ±0.03 px."""
    found, corners = cv2.findChessboardCornersSB(
        gray, pattern, flags=cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found or corners is None or len(corners) != pattern[0] * pattern[1]:
        # SB can be flaky (observed: fails on a large board with slight blur
        # that the classic detector handles) — classic finder as fallback
        found, corners = cv2.findChessboardCorners(
            gray, pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found or corners is None \
                or len(corners) != pattern[0] * pattern[1]:
            return None
    corners = corners.reshape(-1, 1, 2).astype(np.float32)
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-4)
    cv2.cornerSubPix(gray, corners, (8, 8), (-1, -1), crit)
    corners = corners.reshape(-1, 2)
    # canonical order regardless of detector/board orientation: first corner
    # above-left of the last — otherwise L/R (or L/L across views) rows would
    # not correspond when one detection starts from the other end
    if (corners[0, 1], corners[0, 0]) > (corners[-1, 1], corners[-1, 0]):
        corners = corners[::-1].copy()
    return corners


def brightness(gray):
    return float(gray.mean())


def sharpness(gray):
    """Laplacian variance — low = blurry / out of focus."""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


# ------------------------------------------------------------------ pairs ----
def _right_of(left_path):
    """Guess the right-image path for a left one: 'left'→'right', with the
    legacy 'righ.jpg' typo tried as fallback."""
    for a, b in (("left", "right"), ("left", "righ"), ("_l.", "_r.")):
        cand = Path(str(left_path).replace(a, b))
        if cand != Path(left_path) and cand.is_file():
            return cand
    return None


def iter_pairs(session=None, left=None, right=None):
    """Yield (tag, left_path, right_path, z_true_or_None).

    session dir with manifest.csv (tag, z_true_m, left, right) → manifest rows;
    session dir without manifest → every '*left*' image with a matching right;
    explicit --left/--right → single pair.
    """
    if left and right:
        yield Path(left).stem, Path(left), Path(right), None
        return
    session = Path(session)
    manifest = session / "manifest.csv"
    if manifest.is_file():
        with open(manifest, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                z = row.get("z_true_m", "").strip()
                yield (row["tag"], session / row["left"], session / row["right"],
                       float(z) if z and float(z) > 0 else None)
        return
    for lp in sorted(session.glob("*.jpg")) + sorted(session.glob("*.png")):
        if "left" not in lp.name:
            continue
        rp = _right_of(lp)
        if rp is not None:
            yield lp.stem.replace("_left", "").replace("left", "") or lp.stem, \
                lp, rp, None


def read_pair(left_path, right_path):
    l, r = cv2.imread(str(left_path)), cv2.imread(str(right_path))
    if l is None or r is None:
        raise FileNotFoundError(f"cannot read pair: {left_path} / {right_path}")
    return l, r


def check_size(img, calib, tag):
    h, w = img.shape[:2]
    if (w, h) != calib.size:
        raise ValueError(f"{tag}: image {w}x{h} but calib expects "
                         f"{calib.size[0]}x{calib.size[1]}")


def write_csv(path, header, rows):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        wtr = csv.writer(f)
        wtr.writerow(header)
        wtr.writerows(rows)
    print(f"[calib_io] report -> {path}")
