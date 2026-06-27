#!/usr/bin/env python3
"""
freespace.py — Navigable-ground (free space) mask powered by AD-Census disparity.

Same freespace logic as stereo-camera/tools/freespace.py, but the disparity comes
from the AD-Census binary (higher quality than SGBM) instead of cv2.StereoSGBM.

Pipeline per frame:
  1. Run adcensus_depth → <prefix>_disp_raw.png  (16-bit, disparity×16, 0=invalid)
  2. Load raw disparity → float32, invalid = NaN
  3. V-disparity ground-plane fit:
       for each row → 75th-percentile disparity of valid pixels
       fit  d_ground(v) = a·v + b
  4. Per-pixel classify:
       GROUND   : |d - d_ground(v)| <  thresh  AND d > 1
       OBSTACLE : |d - d_ground(v)| >= thresh  AND d > 1
       UNKNOWN  : d <= 1
  5. Flood from bottom-row GROUND pixels → keep only reachable free space
  6. Morphological CLOSE to fill holes
  7. Overlay: green = free space, red = obstacle, white box = obstacle blob

Usage (from stereo-camera-AD-Census/):
    # Offline test
    python3 tools/freespace.py --left-img L.png --right-img R.png

    # Single live capture
    python3 tools/freespace.py --camera

    # Continuous live loop with window
    python3 tools/freespace.py --camera --loop --display

    # Tune ground tolerance
    python3 tools/freespace.py --camera --thresh 8
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# ── Defaults ─────────────────────────────────────────────────────────────────

_BINARY      = str((Path(__file__).resolve().parent.parent / "adcensus_depth"))
_THRESH      = 6.0    # disparity-px tolerance for ground classification
_MIN_AREA    = 400    # min connected free-space blob (px) to keep
_MIN_OBS     = 300    # min obstacle blob (px) to draw a box
_ALPHA_FREE  = 0.45   # green overlay opacity
_ALPHA_OBS   = 0.35   # red overlay opacity

# Metric scale (matching stereo-camera/tools/depth_grid.py — no calib file yet).
# depth_mm = baseline_mm * focal_px / disparity_px.  Tune --focal if distances are off.
_BASELINE_MM = 54.0   # physical distance between the two cameras
_FOCAL_PX    = 554.0  # estimated focal length (px) for ~60° H-FOV at 640 px


# ── Raw disparity I/O ─────────────────────────────────────────────────────────

def load_raw_disparity(path: str) -> np.ndarray:
    """
    Read 16-bit raw disparity PNG saved by adcensus_depth.
    Returns float32 array; value 0 → NaN (invalid).
    """
    raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise FileNotFoundError(f"Cannot read disparity: {path}")
    disp = raw.astype(np.float32) / 16.0
    with np.errstate(invalid='ignore'):
        disp[disp <= 0] = np.nan
    return disp


# ── Ground-plane fit (V-disparity) ───────────────────────────────────────────

def fit_ground_plane(disp: np.ndarray):
    """
    Fit d_ground(v) = a·v + b from row-wise 75th-percentile disparity.
    Returns (a, b) or None if coverage is too sparse.
    v=0 is top of image; higher v = lower in image = closer ground = higher disparity.
    """
    h, w = disp.shape
    vs, ds = [], []

    for v in range(h):
        row = disp[v]
        with np.errstate(invalid='ignore'):
            valid = row[np.isfinite(row) & (row > 1.0)]
        if len(valid) >= w * 0.15:
            vs.append(v)
            ds.append(float(np.percentile(valid, 75)))

    if len(vs) < max(10, h // 5):
        return None

    vs = np.array(vs, dtype=np.float64)
    ds = np.array(ds, dtype=np.float64)
    A  = np.column_stack([vs, np.ones_like(vs)])
    coef, *_ = np.linalg.lstsq(A, ds, rcond=None)
    return float(coef[0]), float(coef[1])


# ── Freespace mask ────────────────────────────────────────────────────────────

def compute_freespace(disp: np.ndarray, ground, thresh: float) -> np.ndarray:
    """
    Returns uint8 mask: 255 = free ground reachable from bottom row, else 0.
    """
    if ground is None:
        return np.zeros(disp.shape, dtype=np.uint8)

    a, b  = ground
    h, w  = disp.shape
    rows  = np.arange(h, dtype=np.float32)
    d_ref = (a * rows + b)[:, None]

    with np.errstate(invalid='ignore'):
        ground_px = (
            np.isfinite(disp) &
            (disp > 1.0) &
            (np.abs(disp - d_ref) < thresh)
        ).astype(np.uint8) * 255

    # Keep only ground blobs connected to the bottom row (= reachable free space).
    n, labels, stats, _ = cv2.connectedComponentsWithStats(ground_px)
    out = np.zeros_like(ground_px)
    for lbl in range(1, n):
        if stats[lbl, cv2.CC_STAT_AREA] < _MIN_AREA:
            continue
        if np.any(labels[h - 1, :] == lbl):
            out[labels == lbl] = 255

    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    return cv2.morphologyEx(out, cv2.MORPH_CLOSE, k)


# ── Obstacle mask ─────────────────────────────────────────────────────────────

def compute_obstacles(disp: np.ndarray, ground, thresh: float) -> np.ndarray:
    """Valid-disparity pixels that do NOT belong to the ground plane."""
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
            (np.abs(disp - d_ref) >= thresh)
        ).astype(np.uint8) * 255

    k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    return cv2.dilate(obs, k, iterations=1)


def disparity_to_distance(disp_px: float, focal_px: float, baseline_mm: float) -> float:
    """Convert a disparity value (px) to distance in metres. Returns inf if invalid."""
    if not np.isfinite(disp_px) or disp_px <= 1.0:
        return float('inf')
    return (baseline_mm * focal_px / disp_px) / 1000.0


def find_obstacle_boxes(obs_mask: np.ndarray, disp: np.ndarray,
                        focal_px: float, baseline_mm: float):
    """
    Bounding boxes of obstacle blobs above the min-area threshold, each tagged
    with the distance to the nearest part of the blob.
    Returns list of (x, y, w, h, dist_m).
    """
    n, labels, stats, _ = cv2.connectedComponentsWithStats(obs_mask)
    boxes = []
    for lbl in range(1, n):
        if stats[lbl, cv2.CC_STAT_AREA] < _MIN_OBS:
            continue
        x  = stats[lbl, cv2.CC_STAT_LEFT]
        y  = stats[lbl, cv2.CC_STAT_TOP]
        bw = stats[lbl, cv2.CC_STAT_WIDTH]
        bh = stats[lbl, cv2.CC_STAT_HEIGHT]

        # Distance = nearest point of the blob → use the 90th-percentile disparity
        # (closest = highest disparity), robust against a few noisy max pixels.
        with np.errstate(invalid='ignore'):
            blob_d = disp[(labels == lbl) & np.isfinite(disp) & (disp > 1.0)]
        if blob_d.size:
            d_near = float(np.percentile(blob_d, 90))
            dist_m = disparity_to_distance(d_near, focal_px, baseline_mm)
        else:
            dist_m = float('inf')
        boxes.append((x, y, bw, bh, dist_m))
    return boxes


# ── Overlay renderer ─────────────────────────────────────────────────────────

def make_overlay(left_color: np.ndarray, free_mask: np.ndarray,
                 obs_mask: np.ndarray, boxes: list) -> np.ndarray:
    overlay = left_color.copy().astype(np.float32)

    green = np.zeros_like(overlay)
    green[:, :, 1] = free_mask.astype(np.float32)
    overlay = overlay * (1 - _ALPHA_FREE) + green * _ALPHA_FREE

    red = np.zeros_like(overlay)
    red[:, :, 2] = obs_mask.astype(np.float32)
    overlay = overlay * (1 - _ALPHA_OBS) + red * _ALPHA_OBS

    out = np.clip(overlay, 0, 255).astype(np.uint8)
    for (x, y, bw, bh, dist_m) in boxes:
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (255, 255, 255), 2)
        label = f"{dist_m:.2f}m" if np.isfinite(dist_m) else "?"
        ly = max(y - 6, 12)
        cv2.putText(out, label, (x, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, label, (x, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
    return out


def draw_hud(img: np.ndarray, ground, free_mask: np.ndarray,
             boxes: list, dt_ms: float):
    h, w = img.shape[:2]
    free_pct = 100.0 * int(free_mask.sum() // 255) / (h * w)

    g_str = (f"ground: d={ground[0]:.3f}v+{ground[1]:.1f}"
             if ground else "ground: NOT DETECTED")
    nearest = min((b[4] for b in boxes), default=float('inf'))
    n_str   = f"nearest: {nearest:.2f}m" if np.isfinite(nearest) else "nearest: --"
    lines = [g_str,
             f"free: {free_pct:.1f}%  obstacles: {len(boxes)}  {n_str}",
             f"dt: {dt_ms:.0f}ms"]
    for i, txt in enumerate(lines):
        cv2.putText(img, txt, (6, 18 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(img, txt, (6, 18 + i * 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 40), 1, cv2.LINE_AA)

    contours, _ = cv2.findContours(free_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(img, contours, -1, (0, 255, 128), 1)


# ── AD-Census runner ──────────────────────────────────────────────────────────

def run_adcensus_files(left_img: str, right_img: str, out_prefix: str,
                       min_disp: int, max_disp: int) -> str:
    """Call adcensus_depth in file mode. Returns path to raw disparity PNG."""
    cmd = [_BINARY, left_img, right_img, out_prefix, str(min_disp), str(max_disp)]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"adcensus_depth failed:\n{result.stderr}")
    return out_prefix + "_disp_raw.png"


def capture_pair(left_idx: int, right_idx: int, width: int, height: int,
                 fps: int) -> tuple:
    """Capture one synchronized stereo pair. Returns (left_bgr, right_bgr)."""
    cap_l = cv2.VideoCapture(left_idx)
    cap_r = cv2.VideoCapture(right_idx)
    for cap in (cap_l, cap_r):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS,          fps)

    for _ in range(5):           # warm-up (auto-exposure settle)
        cap_l.grab(); cap_r.grab()

    cap_l.grab(); cap_r.grab()
    ok_l, frame_l = cap_l.retrieve()
    ok_r, frame_r = cap_r.retrieve()
    cap_l.release(); cap_r.release()

    if not ok_l or not ok_r:
        raise RuntimeError("Camera capture failed.")
    return frame_l, frame_r


# ── Single-frame pipeline ─────────────────────────────────────────────────────

def process_frame(left_color: np.ndarray, raw_disp_path: str, thresh: float,
                  focal_px: float = _FOCAL_PX, baseline_mm: float = _BASELINE_MM):
    """
    Full freespace pipeline on one frame.
    Returns (overlay, free_mask, obs_mask, boxes, ground, dt_ms).
    boxes: list of (x, y, w, h, dist_m).
    """
    t0   = time.time()
    disp = load_raw_disparity(raw_disp_path)

    ground = fit_ground_plane(disp)
    free   = compute_freespace(disp, ground, thresh)
    obs    = compute_obstacles(disp, ground, thresh)
    boxes  = find_obstacle_boxes(obs, disp, focal_px, baseline_mm)

    overlay = make_overlay(left_color, free, obs, boxes)
    dt_ms   = (time.time() - t0) * 1000.0
    draw_hud(overlay, ground, free, boxes, dt_ms)

    return overlay, free, obs, boxes, ground, dt_ms


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(
        description="Free-space (navigable ground) mask via AD-Census disparity.")
    ap.add_argument("--left-img",  type=str, default="", help="Left image (file mode)")
    ap.add_argument("--right-img", type=str, default="", help="Right image (file mode)")
    ap.add_argument("--camera",    action="store_true", help="Live camera mode")
    ap.add_argument("--left",      type=int, default=0)
    ap.add_argument("--right",     type=int, default=2)
    ap.add_argument("--width",     type=int, default=640)
    ap.add_argument("--height",    type=int, default=480)
    ap.add_argument("--fps",       type=int, default=30)
    ap.add_argument("--min-disp",  type=int, default=0)
    ap.add_argument("--max-disp",  type=int, default=128)
    ap.add_argument("--thresh",    type=float, default=_THRESH,
                    help="Disparity tolerance for ground classification (px, default 6)")
    ap.add_argument("--focal",     type=float, default=_FOCAL_PX,
                    help="Focal length in px for distance (default 554)")
    ap.add_argument("--baseline",  type=float, default=_BASELINE_MM,
                    help="Camera baseline in mm for distance (default 54)")
    ap.add_argument("--out",       type=str, default="captures/freespace",
                    help="Output prefix (saves _overlay.jpg, _free.png, _obstacles.png)")
    ap.add_argument("--display",   action="store_true", help="Show OpenCV window")
    ap.add_argument("--loop",      action="store_true",
                    help="Repeat capture in a loop (camera mode only)")
    args = ap.parse_args()

    Path(args.out).parent.mkdir(parents=True, exist_ok=True)

    if not os.path.isfile(_BINARY):
        print(f"ERROR: adcensus_depth binary not found at {_BINARY}", file=sys.stderr)
        print("Run build.sh first.", file=sys.stderr)
        sys.exit(1)

    def run_once():
        if args.left_img and args.right_img:
            raw_path   = run_adcensus_files(args.left_img, args.right_img, args.out,
                                            args.min_disp, args.max_disp)
            left_color = cv2.imread(args.left_img, cv2.IMREAD_COLOR)
        elif args.camera:
            frame_l, frame_r = capture_pair(args.left, args.right,
                                            args.width, args.height, args.fps)
            tmp_l = args.out + "_tmp_left.png"
            tmp_r = args.out + "_tmp_right.png"
            cv2.imwrite(tmp_l, frame_l)
            cv2.imwrite(tmp_r, frame_r)
            raw_path   = run_adcensus_files(tmp_l, tmp_r, args.out,
                                            args.min_disp, args.max_disp)
            left_color = frame_l
        else:
            ap.print_help()
            sys.exit(0)

        overlay, free, obs, boxes, ground, dt_ms = process_frame(
            left_color, raw_path, args.thresh, args.focal, args.baseline)

        free_pct = 100.0 * int(free.sum() // 255) / free.size
        nearest  = min((b[4] for b in boxes), default=float('inf'))
        n_str    = f"nearest={nearest:.2f}m" if np.isfinite(nearest) else "nearest=--"
        g_str    = (f"ground: {ground[0]:.3f}v+{ground[1]:.1f}"
                    if ground else "NO GROUND DETECTED")
        print(f"dt={dt_ms:.0f}ms  free={free_pct:.1f}%  "
              f"obstacles={len(boxes)}  {n_str}  {g_str}")
        for i, (x, y, bw, bh, dist_m) in enumerate(boxes):
            d = f"{dist_m:.2f}m" if np.isfinite(dist_m) else "?"
            print(f"  obj[{i}] x={x} y={y} w={bw} h={bh} dist={d}")

        cv2.imwrite(args.out + "_overlay.jpg",   overlay)
        cv2.imwrite(args.out + "_free.png",      free)
        cv2.imwrite(args.out + "_obstacles.png", obs)
        print(f"Saved: {args.out}_overlay.jpg")

        if args.display:
            cv2.imshow("Freespace (AD-Census)", overlay)

    if args.loop and args.camera:
        print("Running loop. Press 'q' in window or Ctrl+C to stop.")
        try:
            while True:
                run_once()
                if args.display and (cv2.waitKey(1) & 0xFF == ord('q')):
                    break
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            if args.display:
                cv2.destroyAllWindows()
    else:
        run_once()
        if args.display:
            cv2.waitKey(0)
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
