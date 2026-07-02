#!/usr/bin/env python3
"""
stereo_ruler.py — use the stereo pair as a RULER to scale Depth Anything V2 depth.

Pipeline (task 2026-07-02_stereo-ruler-mono-scale, steps 2-6):
  1. Normalize the right image with warp_stereo from stereo-camera/calib/alignment.yml
     (removes vertical offset + rotation + scale, KEEPS horizontal disparity).
  2. Golden points: Shi-Tomasi corners on the left image (grid-bucketed), NCC
     block-match along the same row in the aligned right image, subpixel parabola,
     left-right mutual consistency + uniqueness ratio. Few dozen GOLD points beat
     thousands of noisy ones ("quy ho tinh").
  3. DA-V2 relative inverse-depth on the right image, warped to the same frame.
  4. Robust RANSAC fit  d_mono = s * disparity + t  over the golden points.
     (s, t) converts DA-V2 output to pseudo-disparity px: disp = (d_mono - t) / s.
  5. Dense output: pseudo-disparity map (px). Metric depth needs the constant
     f*B (focal px * baseline m): Z[m] = f*B / disp — pass --fb when known.

Usage:
  python3 stereo_ruler.py                     # defaults: pair 3 (left.jpg / righ.jpg)
  python3 stereo_ruler.py --left L.jpg --right R.jpg --fb 0.06
"""
import argparse
import csv
import os
import sys
from pathlib import Path

import numpy as np
import cv2

_HERE = Path(__file__).resolve().parent          # depth-anything/src
_DA_ROOT = _HERE.parent                          # depth-anything/
_REPO = _DA_ROOT.parent
_DEF_LEFT = _REPO / "stereo-camera" / "tools" / "captures" / "left.jpg"
_DEF_RIGHT = _REPO / "stereo-camera" / "tools" / "captures" / "righ.jpg"
_DEF_ALIGN = _REPO / "stereo-camera" / "calib" / "alignment.yml"
_DEF_OUT = _DA_ROOT / "output" / "stereo_ruler"

_CFGS = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
}


# ---------------------------------------------------------------- alignment --
def load_warp_stereo(path):
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    M = fs.getNode("warp_stereo").mat()
    fs.release()
    if M is None:
        sys.exit(f"[ruler] 'warp_stereo' not found in {path} — "
                 f"run stereo-camera/tools/align_from_chessboard.py first")
    return M


def load_rectify(path, size):
    """Rectification maps + f*B from stereo_calibrate_2view.py output.

    This rig is toed-in with a tilted baseline, so epipolar lines are slanted:
    only full rectification makes row-matching valid at every depth (the 2D
    warp from align_from_chessboard is exact at one depth plane only).
    """
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    K = fs.getNode("K").mat()
    dist = fs.getNode("dist").mat()
    R1 = fs.getNode("R1").mat()
    R2 = fs.getNode("R2").mat()
    P1 = fs.getNode("P1").mat()
    P2 = fs.getNode("P2").mat()
    fb = fs.getNode("fb").real()
    fs.release()
    map_l = cv2.initUndistortRectifyMap(K, dist, R1, P1, size, cv2.CV_32FC1)
    map_r = cv2.initUndistortRectifyMap(K, dist, R2, P2, size, cv2.CV_32FC1)
    return map_l, map_r, fb


# ------------------------------------------------------------- golden points --
def _bucket_corners(gray, grid=(12, 8), per_cell=2, max_corners=4000):
    """Shi-Tomasi corners spread over a grid so depths are diverse."""
    pts = cv2.goodFeaturesToTrack(gray, maxCorners=max_corners,
                                  qualityLevel=0.01, minDistance=7)
    if pts is None:
        return []
    h, w = gray.shape
    gx, gy = grid
    taken = {}
    out = []
    for x, y in pts.reshape(-1, 2):          # gFTT returns strongest first
        cell = (int(x * gx / w), int(y * gy / h))
        if taken.get(cell, 0) >= per_cell:
            continue
        taken[cell] = taken.get(cell, 0) + 1
        out.append((float(x), float(y)))
    return out


def _match_row(strip, patch):
    """NCC of patch along a row strip. Returns (offset_subpx, score, score2) or None."""
    if strip.shape[1] < patch.shape[1] or strip.shape[0] != patch.shape[0]:
        return None
    sc = cv2.matchTemplate(strip, patch, cv2.TM_CCOEFF_NORMED).ravel()
    if sc.size < 3:
        return None
    i = int(np.argmax(sc))
    best = float(sc[i])
    # uniqueness: best score outside +-2 of the peak
    masked = sc.copy()
    masked[max(0, i - 2):i + 3] = -1.0
    second = float(masked.max()) if sc.size > 5 else -1.0
    # subpixel parabola
    off = float(i)
    if 0 < i < sc.size - 1:
        denom = sc[i - 1] - 2 * sc[i] + sc[i + 1]
        if abs(denom) > 1e-9:
            off += 0.5 * (sc[i - 1] - sc[i + 1]) / denom
    return off, best, second


def golden_points(gray_l, gray_r, d_min=-4, d_max=140, block=11,
                  ncc_min=0.70, uniq_margin=0.05, lr_tol=1.0,
                  grid=(16, 10), per_cell=3):
    """Sparse high-confidence disparities left->right_aligned.

    Searches rows y-1..y+1 (tolerates ~1px residual rotation).
    Returns list of dicts {x, y, xr, disp, ncc}.
    """
    b = block // 2
    h, w = gray_l.shape
    golden = []
    for x, y in _bucket_corners(gray_l, grid, per_cell, max_corners=6000):
        xi, yi = int(round(x)), int(round(y))
        if not (b + 1 <= yi < h - b - 1 and d_max + b <= xi < w - b):
            continue
        patch = gray_l[yi - b:yi + b + 1, xi - b:xi + b + 1]
        if patch.std() < 3.0:                      # flat patch: unreliable
            continue
        x0 = xi - d_max - b                        # strip covers disp in [d_min, d_max]
        x1 = xi - d_min + b + 1
        if x0 < 0 or x1 > w:
            continue
        m = None
        for dy in (0, -1, 1):                      # residual rotation tolerance
            cand = _match_row(gray_r[yi + dy - b:yi + dy + b + 1, x0:x1], patch)
            if cand is not None and (m is None or cand[1] > m[1]):
                m = cand
        if m is None:
            continue
        off, ncc, ncc2 = m
        if ncc < ncc_min or ncc - ncc2 < uniq_margin:
            continue
        xr = x0 + b + off                          # matched center x in right
        disp = xi - xr
        # left-right consistency: match the right patch back into left
        xri = int(round(xr))
        if not (b <= xri < w - b):
            continue
        rpatch = gray_r[yi - b:yi + b + 1, xri - b:xri + b + 1]
        lx0 = xri + d_min - b
        lx1 = xri + d_max + b + 1
        if lx0 < 0 or lx1 > w:
            continue
        mb = _match_row(gray_l[yi - b:yi + b + 1, lx0:lx1], rpatch)
        if mb is None:
            continue
        x_back = lx0 + b + mb[0]
        if abs(x_back - xi) > lr_tol:
            continue
        golden.append({"x": float(xi), "y": float(yi), "xr": float(xr),
                       "disp": float(disp), "ncc": float(ncc)})
    return golden


# ------------------------------------------------------------------ mono depth --
def infer_mono(right_bgr, encoder, input_size, device_choice):
    import torch
    sys.path.insert(0, str(_HERE))
    from depth_anything_v2.dpt import DepthAnythingV2
    device = ('cuda' if torch.cuda.is_available() else 'cpu') \
        if device_choice == 'auto' else device_choice
    model = DepthAnythingV2(**_CFGS[encoder])
    ckpt = _DA_ROOT / "model" / f"depth_anything_v2_{encoder}.pth"
    model.load_state_dict(torch.load(str(ckpt), map_location='cpu'))
    model = model.to(device).eval()
    depth = model.infer_image(right_bgr, input_size)   # relative inverse depth
    return depth.astype(np.float64), device


# ---------------------------------------------------------------- scale fit --
def fit_affine_ransac(disp, mono, iters=3000, seed=0):
    """Robust d_mono = s*disp + t. Returns (s, t, inlier_mask, rms, thresh)."""
    disp = np.asarray(disp, np.float64)
    mono = np.asarray(mono, np.float64)
    n = len(disp)
    if n < 10:
        sys.exit(f"[ruler] only {n} golden points — need >= 10. Relax thresholds "
                 f"or capture a scene with more texture.")
    thresh = max(0.03 * np.ptp(mono), 1e-6)
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        i, j = rng.choice(n, 2, replace=False)
        dd = disp[i] - disp[j]
        if abs(dd) < 0.5:
            continue
        s = (mono[i] - mono[j]) / dd
        if s <= 0:                     # both are inverse-depth-like: s must be > 0
            continue
        t = mono[i] - s * disp[i]
        inl = np.abs(mono - (s * disp + t)) < thresh
        if best is None or inl.sum() > best.sum():
            best = inl
    if best is None or best.sum() < 5:
        sys.exit("[ruler] RANSAC failed — golden points may span too little depth "
                 "range (all at similar distance).")
    for _ in range(2):                 # least-squares refine on inliers
        A = np.stack([disp[best], np.ones(best.sum())], axis=1)
        s, t = np.linalg.lstsq(A, mono[best], rcond=None)[0]
        r = np.abs(mono - (s * disp + t))
        best = r < thresh
    rms = float(np.sqrt(np.mean(r[best] ** 2)))
    return float(s), float(t), best, rms, thresh


# -------------------------------------------------------------------- output --
def _point_distance(disp, fb):
    """Distance at a golden point. Metric metres if fb given, else relative
    units (proportional to 1/disparity, normalized so nearest ~ 1.0)."""
    d = max(disp, 0.5)                 # avoid div-by-zero on far/negative disp
    return (fb / d) if fb else (1.0 / d)


def _overlay(left_bgr, golden, inliers, fb=None):
    out = left_bgr.copy()
    disps = np.array([g["disp"] for g in golden])
    lo, hi = disps.min(), max(disps.max(), disps.min() + 1e-3)
    unit = "m" if fb else "rel"
    # relative units: scale so the nearest inlier reads ~1.0 (easier to read)
    norm = 1.0
    if not fb:
        inl_disp = [g["disp"] for g, ok in zip(golden, inliers) if ok]
        if inl_disp:
            norm = 1.0 / _point_distance(max(inl_disp), None)
    font = cv2.FONT_HERSHEY_SIMPLEX
    for g, ok in zip(golden, inliers):
        u = (g["disp"] - lo) / (hi - lo)
        color = tuple(int(c) for c in cv2.applyColorMap(
            np.array([[int(u * 255)]], np.uint8), cv2.COLORMAP_JET)[0, 0])
        x, y = int(g["x"]), int(g["y"])
        cv2.circle(out, (x, y), 6, color, 2)
        if not ok:
            cv2.line(out, (x - 8, y - 8), (x + 8, y + 8), (0, 0, 255), 2)
            continue                               # label only inlier points
        if fb and g["disp"] < 2.0:
            label = "far"              # <2px disparity: distance unresolvable
        else:
            dist = _point_distance(g["disp"], fb) * norm
            label = f"{dist:.2f}{unit}" if fb else f"{dist:.2f}"
        # draw text with a dark outline so it reads over any background
        org = (x + 8, y + 4)
        cv2.putText(out, label, org, font, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, label, org, font, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    header = (f"golden={len(golden)} inliers={int(np.sum(inliers))} "
              f"disp {lo:.1f}..{hi:.1f}px | dist unit={unit}"
              + ("" if fb else " (relative, 1/disp; nearest~1.0)"))
    cv2.putText(out, header, (10, 30), font, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(out, header, (10, 30), font, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
    return out


def _scatter(disp, mono, inl, s, t, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(disp[~inl], mono[~inl], c="red", s=18, label="outlier")
    ax.scatter(disp[inl], mono[inl], c="green", s=18, label="inlier (golden)")
    xs = np.linspace(disp.min(), disp.max(), 50)
    ax.plot(xs, s * xs + t, "b-", label=f"d_mono = {s:.4f}*disp + {t:.3f}")
    ax.set_xlabel("stereo disparity (px)  [the ruler]")
    ax.set_ylabel("DA-V2 relative inverse depth")
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(
        description="Scale DA-V2 mono depth with stereo golden points.")
    p.add_argument("--left", default=str(_DEF_LEFT))
    p.add_argument("--right", default=str(_DEF_RIGHT))
    p.add_argument("--align", default=str(_DEF_ALIGN),
                   help="alignment.yml fallback (single-depth warp)")
    p.add_argument("--rectify",
                   default=str(_REPO / "stereo-camera" / "calib"
                               / "stereo_rectify.yml"),
                   help="stereo_rectify.yml from stereo_calibrate_2view.py; "
                        "if present, full metric rectification is used")
    p.add_argument("--encoder", default="vits", choices=list(_CFGS))
    p.add_argument("--input-size", type=int, default=518)
    p.add_argument("--device", default="auto", choices=["auto", "cuda", "cpu"])
    p.add_argument("--d-max", type=int, default=140, help="max disparity px")
    p.add_argument("--block", type=int, default=11, help="NCC block size (odd)")
    p.add_argument("--ncc-min", type=float, default=0.70)
    p.add_argument("--fb", type=float, default=None,
                   help="f*B override (focal px * baseline m). Default: taken "
                        "from stereo_rectify.yml (fx 1124px * B 0.054m = "
                        "60.69). Pass 0 for relative units.")
    p.add_argument("--out-dir", default=str(_DEF_OUT))
    args = p.parse_args()

    left = cv2.imread(args.left)
    right = cv2.imread(args.right)
    if left is None or right is None:
        sys.exit(f"[ruler] cannot read {args.left} / {args.right}")
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1 — bring both images into a row-aligned frame
    h, w = left.shape[:2]
    fb = args.fb
    if Path(args.rectify).exists():
        map_l, map_r, fb_file = load_rectify(args.rectify, (w, h))
        left = cv2.remap(left, map_l[0], map_l[1], cv2.INTER_LINEAR)
        right_al = cv2.remap(right, map_r[0], map_r[1], cv2.INTER_LINEAR)
        warp_mono = lambda m: cv2.remap(m, map_r[0], map_r[1], cv2.INTER_LINEAR)
        if fb is None:
            fb = fb_file
        print(f"[ruler] rectified pair (epipolar horizontal) via "
              f"{Path(args.rectify).name}, f*B={fb:.2f}")
    else:
        # fallback: single-depth 2D warp — disparity has a constant offset on
        # this toed-in rig, so metric Z from fb is NOT valid here
        M = load_warp_stereo(args.align)
        right_al = cv2.warpAffine(right, M, (w, h))
        warp_mono = lambda m: cv2.warpAffine(m, M, (w, h))
        if fb:
            print("[ruler] WARNING --fb with warp mode: toed-in cameras add a "
                  "constant disparity offset; distances will be wrong. "
                  "Run stereo_calibrate_2view.py for metric output.")
        print(f"[ruler] right normalized with warp_stereo from "
              f"{Path(args.align).name}")
    cv2.imwrite(str(out_dir / "right_aligned.jpg"), right_al)
    cv2.imwrite(str(out_dir / "left_aligned.jpg"), left)

    # 2 — golden points
    gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_al, cv2.COLOR_BGR2GRAY)
    golden = golden_points(gray_l, gray_r, d_max=args.d_max,
                           block=args.block, ncc_min=args.ncc_min)
    print(f"[ruler] {len(golden)} golden points "
          f"(NCC>={args.ncc_min}, uniqueness, LR-consistent)")

    # 3 — DA-V2 on the ORIGINAL right image, then warp its map to the same frame
    mono_raw, device = infer_mono(right, args.encoder, args.input_size, args.device)
    print(f"[ruler] DA-V2 {args.encoder} on {Path(args.right).name} (device={device})")
    mono = warp_mono(mono_raw)
    valid = warp_mono(np.ones_like(mono_raw)) > 0.99

    disp, dmono, kept = [], [], []
    for g in golden:
        xi, yi = int(round(g["xr"])), int(round(g["y"]))
        if 0 <= xi < w and 0 <= yi < h and valid[yi, xi]:
            g["d_mono"] = float(mono[yi, xi])
            disp.append(g["disp"])
            dmono.append(g["d_mono"])
            kept.append(g)
    golden = kept
    disp = np.array(disp)
    dmono = np.array(dmono)

    # 4 — robust affine fit: the ruler
    s, t, inl, rms, thr = fit_affine_ransac(disp, dmono)
    print(f"[ruler] d_mono = s*disp + t : s={s:.5f}  t={t:.4f}  "
          f"inliers={int(inl.sum())}/{len(golden)}  rms={rms:.4f} (thr={thr:.4f})")
    print(f"[ruler] pseudo-disparity of ANY right pixel: disp_px = (d_mono - t) / s")
    if fb:
        print(f"[ruler] metric: Z[m] = {fb:.4f} / disp_px")
    else:
        print(f"[ruler] no f*B -> depth stays in disparity units "
              f"(Z proportional to 1/disp_px).")

    # 5 — outputs
    with open(out_dir / "golden_points.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=["x", "y", "xr", "disp", "ncc",
                                           "d_mono", "inlier"])
        wr.writeheader()
        for g, ok in zip(golden, inl):
            wr.writerow({**{k: f"{v:.3f}" for k, v in g.items()}, "inlier": int(ok)})

    cv2.imwrite(str(out_dir / "golden_overlay.jpg"),
                _overlay(left, golden, inl, fb=fb))
    _scatter(disp, dmono, inl, s, t, out_dir / "fit_scatter.png")

    disp_dense = (mono - t) / s
    disp_dense[~valid] = 0.0
    disp_dense = np.clip(disp_dense, 0.0, None)
    np.save(out_dir / "pseudo_disparity.npy", disp_dense.astype(np.float32))
    dv = np.clip(disp_dense / max(args.d_max, 1e-6), 0, 1)
    vis = cv2.applyColorMap((dv * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    vis[~valid] = 0
    cv2.imwrite(str(out_dir / "pseudo_disparity_vis.jpg"), vis)
    if fb:
        z = np.where(disp_dense > 0.5, fb / np.maximum(disp_dense, 0.5), 0.0)
        np.save(out_dir / "depth_metric_m.npy", z.astype(np.float32))
        print(f"[ruler] metric depth map -> depth_metric_m.npy")

    fs = cv2.FileStorage(str(out_dir / "scale.yml"), cv2.FILE_STORAGE_WRITE)
    fs.write("model", f"d_mono = s*disp_px + t ; disp_px = (d_mono - t)/s")
    fs.write("s", s)
    fs.write("t", t)
    fs.write("inliers", int(inl.sum()))
    fs.write("golden_total", len(golden))
    fs.write("rms", rms)
    fs.write("encoder", args.encoder)
    fs.write("fb", fb if fb else 0.0)
    fs.release()
    print(f"[ruler] outputs -> {out_dir}")


if __name__ == "__main__":
    main()
