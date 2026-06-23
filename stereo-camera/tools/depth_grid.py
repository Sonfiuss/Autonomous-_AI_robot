#!/usr/bin/env python3
"""
depth_grid.py – Single-frame stereo depth visualiser

Loads left + right images, optionally rectifies them (uncalibrated, via feature
matching + fundamental matrix), computes a disparity map, then overlays a
72×36 grid where each cell shows the estimated distance in cm.

Usage (from stereo-camera/):
    python3 tools/depth_grid.py \
        --left  captures/left_20260610_001156.jpg \
        --right captures/right_20260610_001156.jpg \
        --out   captures/depth_grid_out.jpg \
        --baseline 54 --ndisp 128

Assumptions (no calibration file):
    Baseline   = 54 mm  (measured physical distance between cameras)
    Focal len  = 554 px (ESTIMATED for ~60° H-FOV at 640 px width — see note below)
    depth (mm) = baseline_mm * focal_px / disparity_px

Focal-length calibration note (no checkerboard needed):
    The absolute cm values scale linearly with --focal. To calibrate quickly:
    place an object at a KNOWN distance, read the cm printed in that cell, then
    set  --focal = current_focal * (true_cm / shown_cm).  Repeat once.

Limitations:
    Uncalibrated rectification fixes epipolar geometry / row alignment but NOT
    lens distortion or absolute metric scale. True accuracy needs a real stereo
    calibration (stereo_calibrate.py -> calib/stereo.yml) and reprojectImageTo3D
    with the Q matrix.
"""

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

# ── Constants ────────────────────────────────────────────────────────────────
BASELINE_MM  = 54.0        # physical baseline
FOCAL_PX     = 554.0       # estimated focal length in pixels (no calib file)
GRID_COLS    = 72
GRID_ROWS    = 36
MIN_DISP     = 1           # ignore pixels with disparity below this (far / invalid)
MAX_DEPTH_MM = 5000        # clip display at 5 m
NDISP        = 128         # disparity search range (must be divisible by 16)


# ── Uncalibrated rectification ─────────────────────────────────────────────────
def rectify_uncalibrated(left, right, min_inliers=20):
    """
    Rectify a stereo pair without calibration: detect + match features, estimate
    the fundamental matrix (RANSAC), then stereoRectifyUncalibrated.

    Returns (left_rect, right_rect, info) where info is a dict with keys:
        ok            – True if rectification applied, False if fell back
        n_matches     – good matches after ratio test
        n_inliers     – RANSAC inliers used for F
        v_residual    – mean |y_l - y_r| of inliers AFTER rectification (px)
    On <min_inliers inliers, warns and returns the original pair (ok=False).
    """
    h, w = left.shape[:2]
    gray_l = cv2.cvtColor(left,  cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

    # Feature detection — prefer SIFT, fall back to ORB
    try:
        detector = cv2.SIFT_create()
        norm = cv2.NORM_L2
    except AttributeError:
        detector = cv2.ORB_create(4000)
        norm = cv2.NORM_HAMMING

    kp_l, des_l = detector.detectAndCompute(gray_l, None)
    kp_r, des_r = detector.detectAndCompute(gray_r, None)

    info = {"ok": False, "n_matches": 0, "n_inliers": 0, "v_residual": float("nan")}
    if des_l is None or des_r is None or len(kp_l) < min_inliers or len(kp_r) < min_inliers:
        print("[rectify] WARN: too few features detected — using original pair.")
        return left, right, info

    bf = cv2.BFMatcher(norm)
    raw = bf.knnMatch(des_l, des_r, k=2)
    good = [m for m, n in (p for p in raw if len(p) == 2) if m.distance < 0.75 * n.distance]
    info["n_matches"] = len(good)
    if len(good) < min_inliers:
        print(f"[rectify] WARN: only {len(good)} good matches — using original pair.")
        return left, right, info

    pts_l = np.float32([kp_l[m.queryIdx].pt for m in good])
    pts_r = np.float32([kp_r[m.trainIdx].pt for m in good])

    F, mask = cv2.findFundamentalMat(pts_l, pts_r, cv2.FM_RANSAC, 1.0, 0.999)
    if F is None or mask is None:
        print("[rectify] WARN: fundamental matrix failed — using original pair.")
        return left, right, info

    mask = mask.ravel().astype(bool)
    in_l, in_r = pts_l[mask], pts_r[mask]
    info["n_inliers"] = int(mask.sum())
    if info["n_inliers"] < min_inliers:
        print(f"[rectify] WARN: only {info['n_inliers']} inliers — using original pair.")
        return left, right, info

    # Vertical residual of inliers BEFORE rectification. If the cameras are
    # already roughly row-aligned this is small, and forcing a homography can
    # only make things worse (stereoRectifyUncalibrated is numerically fragile).
    v_before = float(np.mean(np.abs(in_l[:, 1] - in_r[:, 1])))

    ok, H1, H2 = cv2.stereoRectifyUncalibrated(
        in_l.reshape(-1, 1, 2), in_r.reshape(-1, 1, 2), F, (w, h))
    if not ok:
        print("[rectify] WARN: stereoRectifyUncalibrated failed — using original pair.")
        return left, right, info

    # Mean vertical residual of inliers after applying the homographies
    in_l_h = cv2.perspectiveTransform(in_l.reshape(-1, 1, 2), H1).reshape(-1, 2)
    in_r_h = cv2.perspectiveTransform(in_r.reshape(-1, 1, 2), H2).reshape(-1, 2)
    v_after = float(np.mean(np.abs(in_l_h[:, 1] - in_r_h[:, 1])))
    info["v_residual"] = v_after

    print(f"[rectify] matches={info['n_matches']} inliers={info['n_inliers']} "
          f"v_residual before={v_before:.2f}px after={v_after:.2f}px")

    # Reject the rectification if it does not actually improve row alignment
    # (degenerate homography). Keep whichever pair has lower vertical residual.
    if v_after > v_before + 0.5:
        print("[rectify] Rectification did not improve alignment — using original pair "
              f"(already aligned to {v_before:.2f}px).")
        info["v_residual"] = v_before
        return left, right, info

    left_rect  = cv2.warpPerspective(left,  H1, (w, h))
    right_rect = cv2.warpPerspective(right, H2, (w, h))
    info["ok"] = True
    return left_rect, right_rect, info


def save_rectified_preview(left_rect, right_rect, path, step=40):
    """hstack the rectified pair with horizontal reference lines for eyeballing."""
    pair = np.hstack([left_rect, right_rect])
    h, w = pair.shape[:2]
    for y in range(0, h, step):
        cv2.line(pair, (0, y), (w, y), (0, 255, 0), 1)
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path), pair)
    print(f"[rectify] Saved preview -> {Path(path).resolve()}")


# ── SGBM stereo matcher ───────────────────────────────────────────────────────
def make_matcher(ndisp=NDISP):
    win  = 5
    ndisp = (ndisp // 16) * 16 or 16   # must be divisible by 16
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=ndisp,
        blockSize=win,
        P1=8  * 3 * win ** 2,
        P2=32 * 3 * win ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=32,
        preFilterCap=63,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    return sgbm


def compute_disparity(gray_l, gray_r, ndisp=NDISP):
    """
    Compute a Q16 disparity map. Uses ximgproc WLS filter (with right matcher)
    if available for a denser, smoother result; otherwise plain left matcher.
    """
    left_matcher = make_matcher(ndisp)
    has_wls = hasattr(cv2, "ximgproc")
    if has_wls:
        try:
            right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
            disp_l = left_matcher.compute(gray_l, gray_r)
            disp_r = right_matcher.compute(gray_r, gray_l)
            wls = cv2.ximgproc.createDisparityWLSFilter(left_matcher)
            wls.setLambda(8000.0)
            wls.setSigmaColor(1.5)
            disp = wls.filter(disp_l, gray_l, disparity_map_right=disp_r)
            print("[depth_grid] Disparity: SGBM + WLS filter")
            return disp
        except Exception as e:  # noqa: BLE001
            print(f"[depth_grid] WLS unavailable ({e}); plain SGBM.")
    print("[depth_grid] Disparity: plain SGBM (no ximgproc)")
    return left_matcher.compute(gray_l, gray_r)


# ── Depth from disparity ──────────────────────────────────────────────────────
def disparity_to_depth(disp_raw, baseline_mm=BASELINE_MM, focal_px=FOCAL_PX):
    """Return depth map in mm (float32). Invalid → 0."""
    disp = disp_raw.astype(np.float32) / 16.0   # SGBM stores Q16
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = np.where(disp >= MIN_DISP,
                         baseline_mm * focal_px / disp,
                         0.0)
    depth = np.where(depth > MAX_DEPTH_MM, 0.0, depth)
    return depth


# ── Grid depth sampling ───────────────────────────────────────────────────────
def sample_grid(depth_map, rows, cols):
    """Return (rows, cols) array of median depth per cell."""
    h, w = depth_map.shape
    cy = np.linspace(0, h, rows + 1, dtype=int)
    cx = np.linspace(0, w, cols + 1, dtype=int)
    grid = np.zeros((rows, cols), dtype=np.float32)
    for r in range(rows):
        for c in range(cols):
            patch = depth_map[cy[r]:cy[r+1], cx[c]:cx[c+1]]
            valid = patch[patch > 0]
            if valid.size > 0:
                grid[r, c] = float(np.median(valid))
    return grid


# ── Visualisation ─────────────────────────────────────────────────────────────
CELL_MIN_PX = 28   # upscale so each cell is at least this many pixels wide


def draw_depth_grid(left_img, depth_grid, rows, cols, baseline_mm, focal_px):
    """
    Draw a colour-coded depth grid overlay on the left image.
    Each cell is filled with a heat colour and labelled with depth in cm.
    """
    h0, w0 = left_img.shape[:2]
    scale = max(1, int(np.ceil(CELL_MIN_PX / (w0 / cols))))
    if scale > 1:
        vis = cv2.resize(left_img, (w0 * scale, h0 * scale),
                         interpolation=cv2.INTER_LINEAR)
    else:
        vis = left_img.copy()
    h, w = vis.shape[:2]

    # Cell boundaries
    xs = np.linspace(0, w, cols + 1, dtype=int)
    ys = np.linspace(0, h, rows + 1, dtype=int)

    valid = depth_grid[depth_grid > 0]
    d_min = float(valid.min()) if valid.size else 0
    d_max = float(np.clip(valid.max(), 1, MAX_DEPTH_MM)) if valid.size else MAX_DEPTH_MM

    font       = cv2.FONT_HERSHEY_SIMPLEX
    cell_h     = ys[1] - ys[0]
    cell_w     = xs[1] - xs[0]
    font_scale = max(0.18, min(0.35, cell_h / 55))
    thickness  = 1

    for r in range(rows):
        for c in range(cols):
            d = depth_grid[r, c]
            x0, y0, x1, y1 = xs[c], ys[r], xs[c+1], ys[r+1]

            # Colour from depth: near=red, far=blue
            if d > 0:
                ratio = np.clip((d - d_min) / max(d_max - d_min, 1), 0, 1)
                # BGR: near=red(0,0,255) → far=blue(255,0,0)
                b = int(255 * ratio)
                g = int(255 * (1 - abs(2 * ratio - 1)))
                rv = int(255 * (1 - ratio))
                color = (b, g, rv)
                alpha = 0.30
                overlay = vis[y0:y1, x0:x1].copy()
                overlay[:] = color
                cv2.addWeighted(overlay, alpha, vis[y0:y1, x0:x1], 1 - alpha, 0,
                                vis[y0:y1, x0:x1])
                label = f"{d/10:.0f}"   # mm → cm, no decimal
            else:
                label = "?"

            # Grid line
            cv2.rectangle(vis, (x0, y0), (x1 - 1, y1 - 1), (80, 80, 80), 1)

            # Label centred in cell
            (tw, th), _ = cv2.getTextSize(label, font, font_scale, thickness)
            tx = x0 + (cell_w - tw) // 2
            ty = y0 + (cell_h + th) // 2
            # Shadow then text for readability
            cv2.putText(vis, label, (tx+1, ty+1), font, font_scale,
                        (0, 0, 0), thickness + 1, cv2.LINE_AA)
            cv2.putText(vis, label, (tx, ty), font, font_scale,
                        (255, 255, 255), thickness, cv2.LINE_AA)

    # Legend bar (right 10 px strip)
    bar_h = h
    for py in range(bar_h):
        ratio = 1.0 - py / bar_h
        b = int(255 * (1 - ratio))
        g = int(255 * (1 - abs(2 * ratio - 1)))
        rv = int(255 * ratio)
        cv2.line(vis, (w - 10, py), (w - 1, py), (b, g, rv), 1)
    cv2.putText(vis, f"{d_min/10:.0f}cm", (w - 48, h - 4),
                font, 0.35, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(vis, f"{d_max/10:.0f}cm", (w - 48, 12),
                font, 0.35, (255, 255, 255), 1, cv2.LINE_AA)

    # Title
    cv2.putText(vis, f"Depth grid {cols}x{rows}  baseline={baseline_mm:.0f}mm  f={focal_px:.0f}px",
                (4, 14), font, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

    return vis


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Single-frame stereo depth grid",
        epilog="Focal calibration: place an object at a known distance, then set "
               "--focal = old_focal * (true_cm / shown_cm) and re-run.")
    parser.add_argument("--left",  required=True, help="Left image path")
    parser.add_argument("--right", required=True, help="Right image path")
    parser.add_argument("--out",   default="depth_grid_out.jpg",
                        help="Output image path (default: depth_grid_out.jpg)")
    parser.add_argument("--baseline", type=float, default=BASELINE_MM,
                        help=f"Camera baseline in mm (default {BASELINE_MM})")
    parser.add_argument("--focal",    type=float, default=FOCAL_PX,
                        help=f"Focal length in pixels (default {FOCAL_PX})")
    parser.add_argument("--ndisp", type=int, default=NDISP,
                        help=f"Disparity search range, mult of 16 (default {NDISP})")
    parser.add_argument("--rectify", dest="rectify", action="store_true", default=True,
                        help="Enable uncalibrated rectification (default on)")
    parser.add_argument("--no-rectify", dest="rectify", action="store_false",
                        help="Disable rectification")
    parser.add_argument("--show", action="store_true",
                        help="Display result in a window (requires display)")
    args = parser.parse_args()

    baseline_mm = args.baseline
    focal_px    = args.focal

    # Load images
    left  = cv2.imread(args.left)
    right = cv2.imread(args.right)
    if left is None or right is None:
        sys.exit(f"[ERROR] Could not load images: {args.left}  {args.right}")

    print(f"[depth_grid] Image size: {left.shape[1]}x{left.shape[0]}")
    print(f"[depth_grid] Baseline={baseline_mm}mm  focal={focal_px}px  ndisp={args.ndisp}")
    print(f"[depth_grid] Grid: {GRID_COLS} cols × {GRID_ROWS} rows")

    out_path = Path(args.out)

    # Rectify (uncalibrated) before matching
    if args.rectify:
        left_r, right_r, info = rectify_uncalibrated(left, right)
        if info["ok"]:
            preview = out_path.parent / "depth_grid_rectified.jpg"
            save_rectified_preview(left_r, right_r, preview)
    else:
        left_r, right_r = left, right
        print("[depth_grid] Rectification disabled (--no-rectify)")

    # Convert to grayscale for disparity
    gray_l = cv2.cvtColor(left_r,  cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_r, cv2.COLOR_BGR2GRAY)

    # Equalise histogram to improve matching in low-contrast areas
    gray_l = cv2.equalizeHist(gray_l)
    gray_r = cv2.equalizeHist(gray_r)

    # Compute disparity on the rectified images
    print("[depth_grid] Computing SGBM disparity...")
    disp_raw = compute_disparity(gray_l, gray_r, args.ndisp)

    depth_map = disparity_to_depth(disp_raw, baseline_mm, focal_px)

    valid_pct = 100.0 * np.sum(depth_map > 0) / depth_map.size
    print(f"[depth_grid] Valid depth pixels: {valid_pct:.1f}%")
    if np.any(depth_map > 0):
        d_valid = depth_map[depth_map > 0]
        print(f"[depth_grid] Depth range: {d_valid.min():.0f} – {d_valid.max():.0f} mm")

    # Sample grid
    grid = sample_grid(depth_map, GRID_ROWS, GRID_COLS)
    grid_valid_pct = 100.0 * np.sum(grid > 0) / grid.size
    print(f"[depth_grid] Valid grid cells: {grid_valid_pct:.1f}%")

    # Draw on the rectified left image so grid aligns with the depth map
    vis = draw_depth_grid(left_r, grid, GRID_ROWS, GRID_COLS, baseline_mm, focal_px)

    # Save
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), vis)
    print(f"[depth_grid] Saved -> {out_path.resolve()}")

    if args.show:
        cv2.imshow("Depth Grid", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
