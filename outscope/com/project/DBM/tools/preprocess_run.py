#!/usr/bin/env python3
"""
preprocess_run.py — preprocessing + cleanup wrapper around the AD-Census binary.

Pipeline (all steps optional via flags):
    1. denoise + CLAHE   improve texture / suppress sensor noise before matching
    2. rectify           uncalibrated (feature match + fundamental matrix) so the
                         left/right rows line up — the real fix for scattered disp
    3. adcensus_depth    run the compiled C++ matcher on the prepared pair
    4. speckle + guided  remove small isolated disparity blobs and edge-aware smooth
                         the result (stand-in for WLS on an external disparity map)
    5. zoning            segment the cleaned disparity into depth bands + outline the
                         largest object regions

The C++ tool (adcensus_depth) is NOT modified — this only prepares its input and
cleans its output. Uses the same uncalibrated rectification proven in
../stereo-camera/tools/depth_grid.py.

Usage (from stereo-camera-AD-Census/):
    python tools/preprocess_run.py \
        --left  captures/left_20260610_001156.jpg \
        --right captures/right_20260610_001156.jpg \
        --out-prefix captures/adcensus_pre \
        --ndisp 128

On Windows the MinGW OpenCV DLLs (C:\\msys64\\mingw64\\bin) are added to PATH for
the subprocess automatically.
"""

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np

MODULE_DIR = Path(__file__).resolve().parent.parent
MINGW_BIN = r"C:\msys64\mingw64\bin"   # for the C++ tool's OpenCV DLLs on Windows

# Metric-depth constants (uncalibrated estimate, shared with
# ../stereo-camera/tools/depth_grid.py). depth_mm = baseline_mm * focal_px / disp_px
BASELINE_MM = 54.0
FOCAL_PX    = 554.0
GRID_COLS   = 72
GRID_ROWS   = 36


# ── 1. denoise + CLAHE ────────────────────────────────────────────────────────
def denoise_clahe(bgr):
    """Edge-preserving denoise (bilateral) + CLAHE on the luma channel."""
    den = cv2.bilateralFilter(bgr, d=7, sigmaColor=50, sigmaSpace=50)
    lab = cv2.cvtColor(den, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


# ── 2. uncalibrated rectification (ported from depth_grid.py) ──────────────────
def rectify_uncalibrated(left, right, min_inliers=20):
    h, w = left.shape[:2]
    gray_l = cv2.cvtColor(left,  cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
    try:
        detector, norm = cv2.SIFT_create(), cv2.NORM_L2
    except AttributeError:
        detector, norm = cv2.ORB_create(4000), cv2.NORM_HAMMING

    kp_l, des_l = detector.detectAndCompute(gray_l, None)
    kp_r, des_r = detector.detectAndCompute(gray_r, None)
    info = {"ok": False, "n_matches": 0, "n_inliers": 0, "v_residual": float("nan")}
    if des_l is None or des_r is None or len(kp_l) < min_inliers or len(kp_r) < min_inliers:
        print("[rectify] WARN: too few features — using original pair.")
        return left, right, info

    raw = cv2.BFMatcher(norm).knnMatch(des_l, des_r, k=2)
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

    v_before = float(np.mean(np.abs(in_l[:, 1] - in_r[:, 1])))
    ok, H1, H2 = cv2.stereoRectifyUncalibrated(
        in_l.reshape(-1, 1, 2), in_r.reshape(-1, 1, 2), F, (w, h))
    if not ok:
        print("[rectify] WARN: stereoRectifyUncalibrated failed — using original pair.")
        return left, right, info

    in_l_h = cv2.perspectiveTransform(in_l.reshape(-1, 1, 2), H1).reshape(-1, 2)
    in_r_h = cv2.perspectiveTransform(in_r.reshape(-1, 1, 2), H2).reshape(-1, 2)
    v_after = float(np.mean(np.abs(in_l_h[:, 1] - in_r_h[:, 1])))
    info["v_residual"] = v_after
    print(f"[rectify] matches={info['n_matches']} inliers={info['n_inliers']} "
          f"v_residual before={v_before:.2f}px after={v_after:.2f}px")

    if v_after > v_before + 0.5:
        print("[rectify] Rectification did not improve alignment — using original pair.")
        info["v_residual"] = v_before
        return left, right, info

    info["ok"] = True
    return (cv2.warpPerspective(left, H1, (w, h)),
            cv2.warpPerspective(right, H2, (w, h)), info)


# ── 3. run the compiled AD-Census binary ──────────────────────────────────────
def find_binary():
    for name in ("adcensus_depth.exe", "adcensus_depth"):
        for d in (MODULE_DIR, MODULE_DIR / "build"):
            p = d / name
            if p.exists():
                return p
    return None


def run_adcensus(binary, left_path, right_path, out_prefix, mind, maxd):
    env = os.environ.copy()
    if os.name == "nt" and Path(MINGW_BIN).exists():
        env["PATH"] = MINGW_BIN + os.pathsep + env.get("PATH", "")
    cmd = [str(binary), str(left_path), str(right_path), str(out_prefix),
           str(mind), str(maxd)]
    print(f"[adcensus] {' '.join(cmd)}")
    res = subprocess.run(cmd, check=True, env=env, capture_output=True, text=True)
    print(res.stdout, end="")
    if res.stderr:
        print(res.stderr, end="", file=sys.stderr)

    # The binary min-max normalizes disparity to 0..255 for the PNG and prints the
    # true pixel range. We recover px (hence metric depth) by de-normalizing.
    dmin_px = dmax_px = None
    m = re.search(r"Disparity range:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]", res.stdout)
    if m:
        dmin_px, dmax_px = float(m.group(1)), float(m.group(2))
        print(f"[adcensus] recovered disparity range: [{dmin_px:.2f}, {dmax_px:.2f}] px")
    else:
        print("[adcensus] WARN: could not parse disparity range — metric depth unavailable.")

    disp_png = Path(f"{out_prefix}_disp.png")
    if not disp_png.exists():
        sys.exit(f"[ERROR] adcensus produced no output: {disp_png}")
    disp8 = cv2.imread(str(disp_png), cv2.IMREAD_GRAYSCALE)
    return disp8, dmin_px, dmax_px


# ── metric depth from the normalized 8-bit disparity ───────────────────────────
def disp8_to_cm(disp8, dmin_px, dmax_px, baseline_mm=BASELINE_MM, focal_px=FOCAL_PX):
    """De-normalize disp8 (0..255) back to px using the binary's reported range,
    then convert to depth in cm. 0 (invalid / farthest) stays 0."""
    cm = np.zeros(disp8.shape, np.float32)
    valid = disp8 > 0
    if not valid.any():
        return cm
    if dmin_px is not None and dmax_px is not None and dmax_px > dmin_px:
        px = dmin_px + disp8[valid].astype(np.float32) / 255.0 * (dmax_px - dmin_px)
    else:
        px = disp8[valid].astype(np.float32)   # unknown scale: relative only
    px = np.maximum(px, 0.5)
    cm[valid] = baseline_mm * focal_px / px / 10.0
    return cm


# ── 4. speckle removal + edge-aware (guided) smoothing ────────────────────────
def clean_disparity(disp8, guide_bgr, speckle_area=80, max_diff=4):
    """disp8: 8-bit normalized disparity. Returns cleaned 8-bit disparity."""
    # filterSpeckles works on int16; 0 stays invalid.
    d16 = disp8.astype(np.int16)
    cv2.filterSpeckles(d16, 0, speckle_area, max_diff)
    cleaned = np.clip(d16, 0, 255).astype(np.uint8)

    # Edge-aware smoothing guided by the left image (WLS stand-in).
    if hasattr(cv2, "ximgproc"):
        try:
            cleaned = cv2.ximgproc.guidedFilter(
                guide=cv2.cvtColor(guide_bgr, cv2.COLOR_BGR2GRAY),
                src=cleaned, radius=8, eps=500)
        except Exception as e:  # noqa: BLE001
            print(f"[clean] guidedFilter unavailable ({e}); bilateral fallback.")
            cleaned = cv2.bilateralFilter(cleaned, 7, 40, 40)
    else:
        cleaned = cv2.bilateralFilter(cleaned, 7, 40, 40)

    # Keep holes (0) as invalid; close tiny gaps in valid regions.
    valid = (cleaned > 0).astype(np.uint8) * 255
    valid = cv2.morphologyEx(valid, cv2.MORPH_OPEN,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    cleaned[valid == 0] = 0
    return cleaned


# ── 5. depth-band zoning + object outlines ────────────────────────────────────
def zone_objects(disp8, left_bgr, n_bands=5, min_area_frac=0.01):
    """Segment disparity into depth bands and outline large object regions."""
    h, w = disp8.shape
    overlay = left_bgr.copy()
    valid = disp8 > 0

    # Quantize valid disparity into n_bands -> colored zones.
    bands = np.zeros_like(disp8)
    if valid.any():
        vmin, vmax = int(disp8[valid].min()), int(disp8[valid].max())
        edges = np.linspace(vmin, vmax + 1, n_bands + 1)
        bands[valid] = np.digitize(disp8[valid], edges[1:-1]) + 1  # 1..n_bands
    band_color = cv2.applyColorMap((bands * (255 // max(n_bands, 1))).astype(np.uint8),
                                   cv2.COLORMAP_JET)
    band_color[~valid] = 0
    zoned = cv2.addWeighted(left_bgr, 0.5, band_color, 0.5, 0)

    # Outline large connected objects (nearest band = closest objects first).
    min_area = int(min_area_frac * h * w)
    for b in range(n_bands, 0, -1):
        bandmask = (bands == b).astype(np.uint8)
        bandmask = cv2.morphologyEx(bandmask, cv2.MORPH_OPEN,
                                    cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
        n, labels, stats, cent = cv2.connectedComponentsWithStats(bandmask, 8)
        for i in range(1, n):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < min_area:
                continue
            x, y, ww, hh = (stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                            stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
            cv2.rectangle(zoned, (x, y), (x + ww, y + hh), (0, 255, 255), 2)
            med = int(np.median(disp8[labels == i]))
            cv2.putText(zoned, f"d{med}", (x + 2, y + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
    return zoned


# ── arm masking ────────────────────────────────────────────────────────────────
def arm_mask(disp8, left_bgr, dmin_px, dmax_px, near_cm=60.0, side="auto",
             baseline_mm=BASELINE_MM, focal_px=FOCAL_PX, min_area_frac=0.008):
    """Find the robot's own arm in the disparity map and return a uint8 mask
    (255 = arm). The arm's image position varies with joint angles, so no fixed
    polygon — instead fuse cues that fail differently:
      (a) near-disparity   the arm is the closest thing in frame,
      (b) border-touch     it is bolted to the body, so its blob reaches an image
                           edge; free-standing objects usually do not,
      (c) side prior       optional left/right hint,
      (d) wire colour seed saturated red/yellow ribbon wires on the arm.
    The decider is border-connectivity: keep the largest near+border component.
    """
    h, w = disp8.shape
    valid = disp8 > 0
    cm = disp8_to_cm(disp8, dmin_px, dmax_px, baseline_mm, focal_px)

    # (a) near gate — prefer metric, fall back to top-disparity percentile when the
    # px scale is unknown or the metric gate captures too little.
    near = valid & (cm > 0) & (cm <= near_cm)
    if near.sum() < min_area_frac * h * w and valid.any():
        thr = np.percentile(disp8[valid], 80)
        near = valid & (disp8 >= thr)
    near = (near.astype(np.uint8)) * 255

    # (d) saturated red/yellow wire seed
    hsv = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2HSV)
    hh, ss, vv = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((hh < 10) | (hh > 170)) & (ss > 120) & (vv > 80)
    yellow = (hh >= 18) & (hh <= 38) & (ss > 120) & (vv > 80)
    near = np.maximum(near, ((red | yellow).astype(np.uint8)) * 255)

    # consolidate the blob
    k9 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    near = cv2.morphologyEx(near, cv2.MORPH_CLOSE, k9)
    near = cv2.morphologyEx(near, cv2.MORPH_OPEN,
                            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))

    # (b)+(c) keep the largest border-touching component, weighted by side prior
    n, labels, stats, cent = cv2.connectedComponentsWithStats(near, 8)
    min_area = int(min_area_frac * h * w)
    best, best_score = -1, -1.0
    for i in range(1, n):
        x, y, ww, hh2, area = (stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                               stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT],
                               stats[i, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        touches = (x == 0 or y == 0 or x + ww >= w or y + hh2 >= h)
        if not touches:
            continue
        cx = cent[i][0]
        if side == "right":
            side_ok = cx >= w * 0.4
        elif side == "left":
            side_ok = cx <= w * 0.6
        else:
            side_ok = True
        score = area * (1.0 if side_ok else 0.3)
        if score > best_score:
            best, best_score = i, score

    mask = np.zeros((h, w), np.uint8)
    if best > 0:
        mask[labels == best] = 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k9)
        cov = 100.0 * (mask > 0).sum() / (h * w)
        print(f"[arm] masked {cov:.1f}% of frame (border-connected near blob).")
    else:
        print("[arm] WARN: no border-connected near blob found — nothing masked.")
    return mask


def arm_overlay(left_bgr, mask):
    """Tint the arm region red + draw its contour, for eyeballing the mask."""
    ov = left_bgr.copy()
    m = mask > 0
    if m.any():
        red = np.zeros_like(ov)
        red[..., 2] = 255
        ov[m] = (left_bgr[m] * 0.55 + red[m] * 0.45).astype(np.uint8)
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(ov, cnts, -1, (0, 255, 255), 2)
    cv2.putText(ov, "ARM MASK", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 2, cv2.LINE_AA)
    return ov


# ── free-space depth grid ──────────────────────────────────────────────────────
def free_space_grid(cm, mask, left_bgr, rows=GRID_ROWS, cols=GRID_COLS,
                    near_cm=80.0, min_valid_frac=0.05, cell_min_px=22):
    """Per-cell free-space map on arm-free metric depth.
    Cells: ARM (gray, blanked), NEAR/obstacle (red, < near_cm, cm labelled),
    CLEAR (green, >= near_cm, cm labelled), UNKNOWN (dark, too few valid px)."""
    h, w = cm.shape
    arm = mask > 0

    scale = max(1, int(np.ceil(cell_min_px / (w / cols))))
    vis = cv2.resize(left_bgr, (w * scale, h * scale), interpolation=cv2.INTER_LINEAR) \
        if scale > 1 else left_bgr.copy()
    H, W = vis.shape[:2]
    xs = np.linspace(0, w, cols + 1, dtype=int)
    ys = np.linspace(0, h, rows + 1, dtype=int)
    XS = np.linspace(0, W, cols + 1, dtype=int)
    YS = np.linspace(0, H, rows + 1, dtype=int)

    font = cv2.FONT_HERSHEY_SIMPLEX
    cell_h = YS[1] - YS[0]
    fs = max(0.18, min(0.34, cell_h / 55))
    n_clear = n_near = n_arm = 0

    for r in range(rows):
        for c in range(cols):
            patch = cm[ys[r]:ys[r+1], xs[c]:xs[c+1]]
            armp = arm[ys[r]:ys[r+1], xs[c]:xs[c+1]]
            X0, Y0, X1, Y1 = XS[c], YS[r], XS[c+1], YS[r+1]
            valid = patch[patch > 0]

            if armp.mean() > 0.5:
                color, label, alpha = (90, 90, 90), "", 0.55      # ARM
                n_arm += 1
            elif valid.size < max(1, int(min_valid_frac * patch.size)):
                color, label, alpha = (40, 40, 40), "?", 0.35     # UNKNOWN
            else:
                d = float(np.median(valid))
                if d < near_cm:
                    color, alpha = (0, 0, 255), 0.40              # NEAR / obstacle
                    n_near += 1
                else:
                    color, alpha = (0, 200, 0), 0.32             # CLEAR / free
                    n_clear += 1
                label = f"{d:.0f}"

            overlay = vis[Y0:Y1, X0:X1].copy()
            overlay[:] = color
            cv2.addWeighted(overlay, alpha, vis[Y0:Y1, X0:X1], 1 - alpha, 0,
                            vis[Y0:Y1, X0:X1])
            cv2.rectangle(vis, (X0, Y0), (X1 - 1, Y1 - 1), (70, 70, 70), 1)
            if label:
                (tw, th), _ = cv2.getTextSize(label, font, fs, 1)
                tx = X0 + ((X1 - X0) - tw) // 2
                ty = Y0 + ((Y1 - Y0) + th) // 2
                cv2.putText(vis, label, (tx+1, ty+1), font, fs, (0, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(vis, label, (tx, ty), font, fs, (255, 255, 255), 1, cv2.LINE_AA)

    cv2.putText(vis, f"free-space {cols}x{rows}  clear={n_clear} near<{near_cm:.0f}cm={n_near} "
                f"arm={n_arm}  (cm labels)", (4, 14), font, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
    print(f"[freespace] clear={n_clear} near={n_near} arm={n_arm} (of {rows*cols} cells)")
    return vis


def main():
    ap = argparse.ArgumentParser(description="Preprocess + AD-Census + cleanup")
    ap.add_argument("--left",  required=True)
    ap.add_argument("--right", required=True)
    ap.add_argument("--out-prefix", default="captures/adcensus_pre")
    ap.add_argument("--ndisp", type=int, default=128)
    ap.add_argument("--mind",  type=int, default=0)
    ap.add_argument("--bin", default=None, help="path to adcensus_depth (auto-detected)")
    ap.add_argument("--no-denoise", action="store_true")
    ap.add_argument("--no-rectify", action="store_true")
    ap.add_argument("--no-clean",   action="store_true")
    ap.add_argument("--no-zone",    action="store_true")
    ap.add_argument("--bands", type=int, default=5)
    # arm mask + free-space
    ap.add_argument("--no-arm-mask",  action="store_true", help="disable arm masking")
    ap.add_argument("--arm-near-cm",  type=float, default=60.0,
                    help="max depth treated as arm (near gate); decider is border-touch")
    ap.add_argument("--arm-side", choices=("auto", "left", "right"), default="auto",
                    help="prior for which side the arm is on (default auto)")
    ap.add_argument("--no-freespace", action="store_true", help="skip free-space grid")
    ap.add_argument("--free-near-cm", type=float, default=80.0,
                    help="cells nearer than this are obstacles (red); else free (green)")
    ap.add_argument("--baseline", type=float, default=BASELINE_MM, help="baseline mm")
    ap.add_argument("--focal",    type=float, default=FOCAL_PX, help="focal px")
    args = ap.parse_args()

    left = cv2.imread(args.left)
    right = cv2.imread(args.right)
    if left is None or right is None:
        sys.exit(f"[ERROR] cannot read images: {args.left} {args.right}")
    print(f"[pre] size {left.shape[1]}x{left.shape[0]}")

    out_prefix = Path(args.out_prefix)
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    # 1. rectify FIRST, on the sharp originals (denoising blurs the keypoints
    #    that feature matching needs, which wrecks the homography estimate).
    if not args.no_rectify:
        left, right, info = rectify_uncalibrated(left, right)
        pair = np.hstack([left, right])
        for y in range(0, pair.shape[0], 40):
            cv2.line(pair, (0, y), (pair.shape[1], y), (0, 255, 0), 1)
        cv2.imwrite(f"{out_prefix}_rectified.png", pair)

    # 2. denoise + CLAHE on the rectified pair, to feed the matcher.
    if not args.no_denoise:
        left, right = denoise_clahe(left), denoise_clahe(right)
        print("[pre] denoise + CLAHE applied")

    # write prepared pair for the C++ tool
    pre_l = Path(f"{out_prefix}_L.png")
    pre_r = Path(f"{out_prefix}_R.png")
    cv2.imwrite(str(pre_l), left)
    cv2.imwrite(str(pre_r), right)

    # 3. run AD-Census
    binary = Path(args.bin) if args.bin else find_binary()
    if not binary or not binary.exists():
        sys.exit("[ERROR] adcensus_depth not found — build it first (run_ad-census.sh).")
    disp, dmin_px, dmax_px = run_adcensus(binary, pre_l, pre_r, out_prefix,
                                          args.mind, args.ndisp)

    # 3b. arm mask — zero the robot's own arm in the disparity BEFORE clean/zone,
    #     so it is never reported as an object or obstacle.
    mask = np.zeros(disp.shape, np.uint8)
    if not args.no_arm_mask:
        mask = arm_mask(disp, left, dmin_px, dmax_px, near_cm=args.arm_near_cm,
                        side=args.arm_side, baseline_mm=args.baseline,
                        focal_px=args.focal)
        cv2.imwrite(f"{out_prefix}_armmask.png", arm_overlay(left, mask))
        print(f"[arm] -> {out_prefix}_armmask.png")
        disp[mask > 0] = 0

    # 4. clean
    if not args.no_clean:
        disp = clean_disparity(disp, left)
        disp[mask > 0] = 0   # keep the arm blanked through cleaning
        cv2.imwrite(f"{out_prefix}_clean.png", disp)
        cv2.imwrite(f"{out_prefix}_clean_color.png",
                    cv2.applyColorMap(disp, cv2.COLORMAP_JET))
        print(f"[clean] -> {out_prefix}_clean.png (+_color)")

    # 5. zoning (on arm-free disparity)
    if not args.no_zone:
        zoned = zone_objects(disp, left, n_bands=args.bands)
        cv2.imwrite(f"{out_prefix}_zones.png", zoned)
        print(f"[zone] -> {out_prefix}_zones.png")

    # 6. free-space depth grid (metric, arm cells blanked)
    if not args.no_freespace:
        cm = disp8_to_cm(disp, dmin_px, dmax_px, args.baseline, args.focal)
        fs = free_space_grid(cm, mask, left, near_cm=args.free_near_cm)
        cv2.imwrite(f"{out_prefix}_freespace.png", fs)
        print(f"[freespace] -> {out_prefix}_freespace.png")

    print("[done] outputs written with prefix:", out_prefix)


if __name__ == "__main__":
    main()
