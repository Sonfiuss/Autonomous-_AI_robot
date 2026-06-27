#!/usr/bin/env python3
"""
freespace.py — Navigable-ground mask from a live stereo camera pair.

Pipeline per frame:
  1. Capture left + right from USB cameras  (or load from file with --left-img / --right-img)
  2. Rectify: uncalibrated (SIFT feature match + fundamental matrix)
              or calibrated (--calib stereo.yml)
  3. SGBM disparity — same params as stereo-camera/vision/StereoCamera.cpp
  4. V-disparity ground-plane fit:
       for each image row  →  75th-percentile disparity of valid pixels
       fit  d_ground(v) = a·v + b  (higher v = lower in image = closer ground = higher disparity)
  5. Per-pixel classify:
       GROUND   : |d - d_ground(v)| < --thresh   AND  d > 1
       OBSTACLE : |d - d_ground(v)| >= --thresh  AND  d > 1
       UNKNOWN  : d <= 1  (no valid disparity)
  6. Flood-fill from every bottom-row GROUND pixel  →  keep only reachable free space
  7. Morphological CLOSE to fill holes in the free region
  8. Overlay:  green (α=0.45) = free,  red (α=0.35) = obstacle,  blended onto left frame
  9. Write --out  (and show window if --display)

Usage (from stereo-camera/):
    python3 tools/freespace.py                                       # live cameras
    python3 tools/freespace.py --display                             # + live window
    python3 tools/freespace.py --left-img captures/left.jpg \\
                               --right-img captures/right.jpg        # offline test
    python3 tools/freespace.py --calib calib/stereo.yml              # calibrated mode
    python3 tools/freespace.py --thresh 8 --out captures/free.jpg    # tuning
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np


# ── Defaults (matching StereoCamera::Config) ─────────────────────────────────
_LEFT_IDX    = 0
_RIGHT_IDX   = 2   # Jetson UVC: each camera occupies 2 nodes (capture+metadata)
_WIDTH       = 640
_HEIGHT      = 480
_FPS         = 30

# SGBM (matching StereoCamera.cpp)
_NDISP       = 128
_BLOCK       = 5

# Freespace tuning
_THRESH      = 6.0    # disparity-px tolerance for ground classification
_MIN_AREA    = 400    # min connected free-space blob (px) to keep
_ALPHA_FREE  = 0.45   # green overlay opacity
_ALPHA_OBS   = 0.35   # red overlay opacity
_WARMUP      = 5      # frames to discard on camera open (auto-exposure settle)


# ── Rectification ─────────────────────────────────────────────────────────────

# Cached homographies from the first successful uncalibrated rectification.
# Reused every frame so ORB matching only runs ONCE (first frame).
# Call reset_uncalib_cache() to force a new ORB match (e.g. camera moved).
_uncalib_H = None   # (H1, H2)

def reset_uncalib_cache():
    global _uncalib_H
    _uncalib_H = None


def _rectify_uncalib(gray_l, gray_r, min_inliers=20):
    """
    First call: ORB feature match → fundamental matrix → stereoRectifyUncalibrated.
    Subsequent calls: reuse cached H1/H2 (stable, fast — no re-matching).
    Returns (gray_l_rect, gray_r_rect, H1, ok).
    """
    global _uncalib_H
    h, w = gray_l.shape

    if _uncalib_H is not None:
        H1, H2 = _uncalib_H
        l_r = cv2.warpPerspective(gray_l, H1, (w, h))
        r_r = cv2.warpPerspective(gray_r, H2, (w, h))
        return l_r, r_r, H1, True

    print("[freespace] Computing rectification (ORB, first frame only)...", flush=True)
    orb  = cv2.ORB_create(nfeatures=2000)
    kp1, d1 = orb.detectAndCompute(gray_l, None)
    kp2, d2 = orb.detectAndCompute(gray_r, None)
    if d1 is None or d2 is None or len(kp1) < 8 or len(kp2) < 8:
        return gray_l, gray_r, None, False

    bf   = cv2.BFMatcher(cv2.NORM_HAMMING)
    raw  = bf.knnMatch(d1, d2, k=2)
    good = [m for m, n in raw if m.distance < 0.85 * n.distance]
    if len(good) < min_inliers:
        return gray_l, gray_r, None, False

    pts1 = np.float32([kp1[m.queryIdx].pt for m in good])
    pts2 = np.float32([kp2[m.trainIdx].pt for m in good])
    F, mask = cv2.findFundamentalMat(pts1, pts2, cv2.FM_RANSAC, 1.0, 0.99)
    if F is None or mask is None:
        return gray_l, gray_r, None, False

    sel = mask.ravel().astype(bool)
    if sel.sum() < min_inliers:
        return gray_l, gray_r, None, False

    _, H1, H2 = cv2.stereoRectifyUncalibrated(pts1[sel], pts2[sel], F, (w, h))
    _uncalib_H = (H1, H2)
    print(f"[freespace] Rectification cached ({int(sel.sum())} inliers).", flush=True)

    l_r = cv2.warpPerspective(gray_l, H1, (w, h))
    r_r = cv2.warpPerspective(gray_r, H2, (w, h))
    return l_r, r_r, H1, True


class _CalibMaps:
    """Loaded once; caches per calib file."""
    _cache: dict = {}

    @classmethod
    def get(cls, calib_path, img_size):
        key = (calib_path, img_size)
        if key in cls._cache:
            return cls._cache[key]

        fs = cv2.FileStorage(calib_path, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            return None

        M1 = fs.getNode("M1").mat(); D1 = fs.getNode("D1").mat()
        M2 = fs.getNode("M2").mat(); D2 = fs.getNode("D2").mat()
        R  = fs.getNode("R").mat();  T  = fs.getNode("T").mat()
        R1 = fs.getNode("R1").mat(); R2 = fs.getNode("R2").mat()
        P1 = fs.getNode("P1").mat(); P2 = fs.getNode("P2").mat()
        fs.release()

        if R1 is None or (isinstance(R1, np.ndarray) and R1.size == 0):
            Q = np.zeros((4, 4))
            R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
                M1, D1, M2, D2, img_size, R, T)

        mLx, mLy = cv2.initUndistortRectifyMap(M1, D1, R1, P1, img_size, cv2.CV_32FC1)
        mRx, mRy = cv2.initUndistortRectifyMap(M2, D2, R2, P2, img_size, cv2.CV_32FC1)
        maps = (mLx, mLy, mRx, mRy)
        cls._cache[key] = maps
        return maps


def rectify(left_color, right_gray, calib_path=None, do_rectify=True):
    """
    Rectify the stereo pair.
    Returns (left_color_rect, left_gray_rect, right_gray_rect).
    left_color_rect is the rectified left frame used for the overlay background.
    Pass do_rectify=False to skip (cameras already horizontally aligned).
    """
    gray_l = cv2.cvtColor(left_color, cv2.COLOR_BGR2GRAY)
    h, w   = gray_l.shape

    if not do_rectify:
        return left_color, gray_l, right_gray

    if calib_path:
        maps = _CalibMaps.get(calib_path, (w, h))
        if maps is not None:
            mLx, mLy, mRx, mRy = maps
            lc_r = cv2.remap(left_color, mLx, mLy, cv2.INTER_LINEAR)
            lg_r = cv2.remap(gray_l,     mLx, mLy, cv2.INTER_LINEAR)
            rg_r = cv2.remap(right_gray, mRx, mRy, cv2.INTER_LINEAR)
            return lc_r, lg_r, rg_r
        else:
            print("[freespace] WARNING: calibration file not found, using uncalibrated.")

    lg_r, rg_r, H1, ok = _rectify_uncalib(gray_l, right_gray)
    if not ok:
        print("[freespace] WARNING: rectification failed — disparity may be noisy.")
        return left_color, gray_l, right_gray

    # Apply the same H1 to the color frame so the overlay aligns with disparity.
    lc_r = cv2.warpPerspective(left_color, H1, (w, h))
    return lc_r, lg_r, rg_r


# ── Disparity ─────────────────────────────────────────────────────────────────

_matcher = None

def _get_matcher():
    global _matcher
    if _matcher is None:
        bs = _BLOCK
        nd = _NDISP
        _matcher = cv2.StereoSGBM_create(
            minDisparity      = 0,
            numDisparities    = nd,
            blockSize         = bs,
            P1                = 8  * 3 * bs * bs,
            P2                = 32 * 3 * bs * bs,
            disp12MaxDiff     = 1,
            preFilterCap      = 63,
            uniquenessRatio   = 10,
            speckleWindowSize = 100,
            speckleRange      = 32,
            mode              = cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
    return _matcher


def compute_disparity(gray_l, gray_r):
    """Returns float32 disparity map; invalid pixels = NaN."""
    disp16 = _get_matcher().compute(gray_l, gray_r)
    disp   = disp16.astype(np.float32) / 16.0
    # Use errstate to suppress NaN comparison warnings from the assignment itself.
    with np.errstate(invalid='ignore'):
        disp[disp <= 0] = np.nan
    return disp


# ── Ground-plane fit (V-disparity) ────────────────────────────────────────────

def fit_ground_plane(disp):
    """
    Build a row→dominant-disparity table, fit d_ground(v) = a·v + b.
    Returns (a, b) or None if coverage is too sparse.

    Convention: v=0 is the TOP of the image; higher v = closer ground = higher disparity.
    A robust positive slope 'a' is expected for a forward-looking camera.
    """
    h, w  = disp.shape
    vs, ds = [], []

    for v in range(h):
        row   = disp[v]
        with np.errstate(invalid='ignore'):
            valid = row[np.isfinite(row) & (row > 1.0)]
        # Require at least 15% valid pixels per row to trust the estimate.
        if len(valid) >= w * 0.15:
            vs.append(v)
            # 75th percentile: robust against obstacles that inflate the median.
            ds.append(float(np.percentile(valid, 75)))

    if len(vs) < max(10, h // 5):
        return None

    vs = np.array(vs, dtype=np.float64)
    ds = np.array(ds, dtype=np.float64)
    A  = np.column_stack([vs, np.ones_like(vs)])
    coef, *_ = np.linalg.lstsq(A, ds, rcond=None)
    return float(coef[0]), float(coef[1])


# ── Freespace mask ────────────────────────────────────────────────────────────

def compute_freespace(disp, ground, thresh=_THRESH):
    """
    Returns uint8 mask:  255 = free ground reachable from bottom row,  0 = blocked/unknown.
    """
    if ground is None:
        return np.zeros(disp.shape, dtype=np.uint8)

    a, b  = ground
    h, w  = disp.shape
    rows  = np.arange(h, dtype=np.float32)
    d_ref = (a * rows + b)[:, None]          # (h, 1) broadcast-ready

    with np.errstate(invalid='ignore'):
        ground_px = (
            np.isfinite(disp) &
            (disp > 1.0) &
            (np.abs(disp - d_ref) < thresh)
        ).astype(np.uint8) * 255

    # Keep only ground pixels connected to the bottom row (= directly reachable ground).
    # Connected-components approach: label all ground blobs, then keep those whose
    # bounding box touches the last image row.
    n, labels, stats, _ = cv2.connectedComponentsWithStats(ground_px)
    out = np.zeros_like(ground_px)
    for lbl in range(1, n):
        area = stats[lbl, cv2.CC_STAT_AREA]
        if area < _MIN_AREA:
            continue
        # A blob is "bottom-connected" if any pixel of that label sits in the last row.
        if np.any(labels[h - 1, :] == lbl):
            out[labels == lbl] = 255

    # Close holes in the free region.
    k   = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, k)
    return out


# ── Obstacle mask ─────────────────────────────────────────────────────────────

def compute_obstacles(disp, ground, thresh=_THRESH):
    """Pixels with valid disparity that do NOT belong to the ground plane."""
    if ground is None:
        return np.zeros(disp.shape, dtype=np.uint8)

    a, b  = ground
    h, w  = disp.shape
    rows  = np.arange(h, dtype=np.float32)
    d_ref = (a * rows + b)[:, None]

    with np.errstate(invalid='ignore'):
        obs = (
            np.isfinite(disp) &
            (disp > 1.0) &
            (np.abs(disp - d_ref) > thresh)
        ).astype(np.uint8) * 255

    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    return cv2.dilate(obs, k, iterations=1)


# ── Overlay ───────────────────────────────────────────────────────────────────

def make_overlay(left_color, free_mask, obs_mask):
    """Alpha-blend green (free) and red (obstacle) onto the left frame."""
    overlay = left_color.copy().astype(np.float32)

    green         = np.zeros_like(overlay)
    green[:, :, 1] = free_mask.astype(np.float32)   # B=0 G=mask R=0
    overlay = overlay * (1 - _ALPHA_FREE) + green * _ALPHA_FREE

    red           = np.zeros_like(overlay)
    red[:, :, 2]  = obs_mask.astype(np.float32)     # B=0 G=0 R=mask
    overlay = overlay * (1 - _ALPHA_OBS) + red * _ALPHA_OBS

    return np.clip(overlay, 0, 255).astype(np.uint8)


# ── Stats HUD ────────────────────────────────────────────────────────────────

def draw_hud(img, ground, free_mask, dt_ms):
    """Burn minimal stats into the top-left corner of the image."""
    h, w = img.shape[:2]
    free_pct = 100.0 * int(free_mask.sum() // 255) / (h * w)

    if ground:
        line1 = f"ground: d={ground[0]:.3f}v + {ground[1]:.1f}"
    else:
        line1 = "ground: NOT DETECTED"
    line2 = f"free: {free_pct:.1f}%   dt: {dt_ms:.0f} ms"

    for i, txt in enumerate([line1, line2]):
        cv2.putText(img, txt, (6, 18 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(img, txt, (6, 18 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 40), 1, cv2.LINE_AA)

    # Draw the freespace boundary contour.
    contours, _ = cv2.findContours(free_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 255, 128), 1)


# ── Full single-frame pipeline ────────────────────────────────────────────────

def process_pair(left_color, right_color, calib_path=None, thresh=_THRESH,
                 do_rectify=True):
    """
    Run the full freespace pipeline on one stereo pair.
    Returns (overlay_bgr, free_mask, ground_or_None, dt_ms).
    """
    t0 = time.time()

    right_gray             = cv2.cvtColor(right_color, cv2.COLOR_BGR2GRAY)
    left_rect, lg_r, rg_r = rectify(left_color, right_gray, calib_path, do_rectify)

    disp   = compute_disparity(lg_r, rg_r)
    ground = fit_ground_plane(disp)
    free   = compute_freespace(disp, ground, thresh)
    obs    = compute_obstacles(disp, ground, thresh)

    overlay = make_overlay(left_rect, free, obs)
    dt_ms   = (time.time() - t0) * 1000.0
    draw_hud(overlay, ground, free, dt_ms)

    return overlay, free, ground, dt_ms


# ── Camera wrapper (mirrors StereoCamera.cpp grab+retrieve pattern) ───────────

class StereoCap:
    def __init__(self, left_idx, right_idx, width, height, fps):
        self._cl = cv2.VideoCapture(left_idx)
        self._cr = cv2.VideoCapture(right_idx)
        for cap in (self._cl, self._cr):
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            cap.set(cv2.CAP_PROP_FPS,          fps)

    def ok(self):
        return self._cl.isOpened() and self._cr.isOpened()

    def warmup(self, n=_WARMUP):
        for _ in range(n):
            self._cl.grab()
            self._cr.grab()

    def grab_pair(self):
        """Synchronized grab → retrieve (minimizes inter-camera skew)."""
        if not self._cl.grab() or not self._cr.grab():
            return None, None
        ok_l, fl = self._cl.retrieve()
        ok_r, fr = self._cr.retrieve()
        return (fl, fr) if (ok_l and ok_r) else (None, None)

    def release(self):
        self._cl.release()
        self._cr.release()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Navigable-ground mask from a live stereo camera pair.")
    ap.add_argument("--left",      type=int,   default=_LEFT_IDX,  help="Left camera index")
    ap.add_argument("--right",     type=int,   default=_RIGHT_IDX, help="Right camera index")
    ap.add_argument("--width",     type=int,   default=_WIDTH)
    ap.add_argument("--height",    type=int,   default=_HEIGHT)
    ap.add_argument("--fps",       type=int,   default=_FPS)
    ap.add_argument("--calib",     type=str,   default="",
                    help="stereo.yml calibration file (optional)")
    ap.add_argument("--left-img",  type=str,   default="",
                    help="Left image file — offline mode (skips camera)")
    ap.add_argument("--right-img", type=str,   default="",
                    help="Right image file — offline mode")
    ap.add_argument("--out",       type=str,   default="captures/freespace_out.jpg",
                    help="Output overlay image path")
    ap.add_argument("--thresh",      type=float, default=_THRESH,
                    help="Disparity tolerance for ground classification (px, default 6)")
    ap.add_argument("--no-rectify", action="store_true",
                    help="Skip rectification (use when cameras are already well-aligned)")
    ap.add_argument("--display",    action="store_true",
                    help="Show live OpenCV window (requires display)")
    args = ap.parse_args()

    calib      = args.calib or None
    do_rectify = not args.no_rectify
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)

    # ── Offline mode ─────────────────────────────────────────────────────────
    if args.left_img and args.right_img:
        left  = cv2.imread(args.left_img,  cv2.IMREAD_COLOR)
        right = cv2.imread(args.right_img, cv2.IMREAD_COLOR)
        if left is None or right is None:
            print("ERROR: could not read input images.", file=sys.stderr)
            sys.exit(1)

        overlay, free, ground, dt = process_pair(
            left, right, calib, args.thresh, do_rectify)
        free_pct = 100.0 * int(free.sum() // 255) / free.size
        g_str    = f"ground: {ground[0]:.3f}v+{ground[1]:.1f}" if ground else "NO GROUND"
        print(f"dt={dt:.0f}ms  free={free_pct:.1f}%  {g_str}")

        cv2.imwrite(args.out, overlay)
        print(f"Saved: {args.out}")

        if args.display:
            cv2.imshow("Freespace", overlay)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return

    # ── Live camera mode ──────────────────────────────────────────────────────
    cam = StereoCap(args.left, args.right, args.width, args.height, args.fps)
    if not cam.ok():
        print(f"ERROR: cannot open cameras {args.left}/{args.right}.", file=sys.stderr)
        sys.exit(1)

    print(f"Cameras {args.left}/{args.right} opened ({args.width}x{args.height} @{args.fps}fps)")
    print("Warming up...", end=" ", flush=True)
    cam.warmup()
    print("ready.")
    print("Running.  Ctrl+C to stop." +
          ("  Window key 'q' to quit." if args.display else ""))

    frame_n = 0
    try:
        while True:
            left, right = cam.grab_pair()
            if left is None:
                print("[ERROR] Camera read failed.", file=sys.stderr)
                break

            overlay, free, ground, dt = process_pair(
                left, right, calib, args.thresh, do_rectify)
            frame_n += 1
            free_pct = 100.0 * int(free.sum() // 255) / free.size
            g_str    = f"ground: {ground[0]:.3f}v+{ground[1]:.1f}" if ground else "NO GROUND"
            print(f"frame={frame_n:4d}  dt={dt:5.0f}ms  free={free_pct:5.1f}%  {g_str}")

            cv2.imwrite(args.out, overlay)

            if args.display:
                cv2.imshow("Freespace", overlay)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        cam.release()
        if args.display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
