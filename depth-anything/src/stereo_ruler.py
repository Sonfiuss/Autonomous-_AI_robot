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

Accuracy correction (anchors): auto golden points tend to cluster near-field,
so the RANSAC affine fit extrapolates poorly to far distances. Measure a few
real distances (near AND far) with a tape, note their pixel (x, y) in
golden_overlay.jpg, put them in a CSV (x,y,z_m) and pass --anchors:
  python3 stereo_ruler.py --anchors my_anchors.csv
Anchors are shown as yellow squares in golden_overlay.jpg with both the
fitted and measured distance, so you can check the correction worked.
"""
import argparse
import csv
import glob
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import cv2

_HERE = Path(__file__).resolve().parent          # depth-anything/src
_DA_ROOT = _HERE.parent                          # depth-anything/
_REPO = _DA_ROOT.parent
_DEF_LEFT = _REPO / "stereo-camera" / "tools" / "captures" / "left.jpg"
_DEF_RIGHT = _REPO / "stereo-camera" / "tools" / "captures" / "righ.jpg"
_DEF_ALIGN = _REPO / "stereo-camera" / "calib" / "alignment.yml"
_DEF_OUT = _DA_ROOT / "output" / "stereo_ruler"

# Live-camera defaults (match area-detection/capture_chessboard.py): left=video0,
# right=video2, 640x480. Used when --camera is passed instead of --left/--right.
_LEFT_IDX = 0
_RIGHT_IDX = 2
_CAM_W = 640
_CAM_H = 480
_CAM_WARMUP = 5

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

    Returns a SimpleNamespace(map_l, map_r, fb, fx, fy, cx, cy, size).
    fx/fy/cx/cy come from the rectified projection P2 (the frame the dense
    depth lives in), so callers can back-project Z to 3D directly:
      X = (u - cx) * Z / fx ;  Y = (v - cy) * Z / fy
    """
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)

    def _mat(name):
        n = fs.getNode(name)
        return None if n.empty() else n.mat()

    def _real(name):
        n = fs.getNode(name)
        return None if n.empty() else float(n.real())

    # per-camera schema (stereo_calibrate_2view printed-board path, 2026-07-11)
    # with fallback to the legacy shared-K schema
    K_l, K_r = _mat("K_left"), _mat("K_right")
    if K_l is not None and K_r is not None:
        d_l, d_r = _mat("dist_left"), _mat("dist_right")
    else:
        K_l = K_r = _mat("K")
        d_l = d_r = _mat("dist")
    R1, R2, P1, P2 = _mat("R1"), _mat("R2"), _mat("P1"), _mat("P2")
    fb = _real("fb")
    # tape-measure-corrected scale (depth_eval.py --fit-fb --write-fb) beats
    # the calibration-derived fb when present — it absorbs printer scale,
    # residual focal error and baseline error in one measured number
    fb_measured = _real("fb_measured")
    fs.release()
    map_l = cv2.initUndistortRectifyMap(K_l, d_l, R1, P1, size, cv2.CV_32FC1)
    map_r = cv2.initUndistortRectifyMap(K_r, d_r, R2, P2, size, cv2.CV_32FC1)
    return SimpleNamespace(map_l=map_l, map_r=map_r,
                           fb=fb_measured if fb_measured else fb,
                           fb_calib=fb, fb_measured=fb_measured,
                           fx=float(P2[0, 0]), fy=float(P2[1, 1]),
                           cx=float(P2[0, 2]), cy=float(P2[1, 2]), size=size)


# --------------------------------------------------------------- live camera --
def capture_pair(dev_left, dev_right, width, height, warmup=_CAM_WARMUP):
    """Grab one synchronized frame from each stereo camera.

    Left/right default to /dev/video0 and /dev/video2 (same wiring as
    area-detection/capture_chessboard.py). A few frames are read first so the
    sensors settle on exposure/white-balance before the kept frame.
    Returns (left_bgr, right_bgr).
    """
    cap_l = cv2.VideoCapture(dev_left)
    cap_r = cv2.VideoCapture(dev_right)
    for cap in (cap_l, cap_r):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    try:
        if not (cap_l.isOpened() and cap_r.isOpened()):
            which = []
            if not cap_l.isOpened():
                which.append(f"left=/dev/video{dev_left}")
            if not cap_r.isOpened():
                which.append(f"right=/dev/video{dev_right}")
            present = ", ".join(sorted(glob.glob("/dev/video*"))) or "(none)"
            sys.exit(f"[ruler] could not open {' and '.join(which)}. This rig "
                     f"needs TWO cameras. Present now: {present}. Plug in the "
                     f"missing camera, then check 'v4l2-ctl --list-devices' and "
                     f"pass its index via --device-left/--device-right.")
        for _ in range(max(warmup, 1)):
            cap_l.read()
            cap_r.read()
        ok_l, left = cap_l.read()
        ok_r, right = cap_r.read()
        if not (ok_l and ok_r):
            sys.exit(f"[ruler] frame grab failed on {dev_left}/{dev_right}")
        return left, right
    finally:
        cap_l.release()
        cap_r.release()


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
                  grid=(16, 10), per_cell=3, dy_search=(0, -1, 1)):
    """Sparse high-confidence disparities left->right_aligned.

    dy_search: rows searched around y (default y-1..y+1 tolerates ~1px
    residual rotation; with a sub-0.5px-rectified calib pass (0,) — the extra
    rows then only admit false matches).
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
        for dy in dy_search:                       # residual rotation tolerance
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
def load_model(encoder='vits', device_choice='auto'):
    """Load DA-V2 once — callers doing several frames reuse the same model.
    Returns (model, device)."""
    import torch
    sys.path.insert(0, str(_HERE))
    from depth_anything_v2.dpt import DepthAnythingV2
    device = ('cuda' if torch.cuda.is_available() else 'cpu') \
        if device_choice == 'auto' else device_choice
    model = DepthAnythingV2(**_CFGS[encoder])
    ckpt = _DA_ROOT / "model" / f"depth_anything_v2_{encoder}.pth"
    model.load_state_dict(torch.load(str(ckpt), map_location='cpu'))
    return model.to(device).eval(), device


def infer_mono(right_bgr, encoder, input_size, device_choice):
    model, device = load_model(encoder, device_choice)
    depth = model.infer_image(right_bgr, input_size)   # relative inverse depth
    return depth.astype(np.float64), device


# ------------------------------------------------------- one-call metric depth --
# Split into three stage functions so the stereo_walk_map pipeline can run
# them on separate threads (StereoWorker / DepthWorker / FusionMapper) while
# compute_metric_depth keeps the one-call sequential behaviour.
def stereo_half(left_bgr, right_bgr, calib, d_max=140, block=11, ncc_min=0.70):
    """Stereo stage: rectify the pair + golden-point disparities.
    Returns (left_rect, right_rect, golden list)."""
    left_r = cv2.remap(left_bgr, calib.map_l[0], calib.map_l[1], cv2.INTER_LINEAR)
    right_r = cv2.remap(right_bgr, calib.map_r[0], calib.map_r[1], cv2.INTER_LINEAR)
    gray_l = cv2.cvtColor(left_r, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_r, cv2.COLOR_BGR2GRAY)
    golden = golden_points(gray_l, gray_r, d_max=d_max, block=block,
                           ncc_min=ncc_min)
    return left_r, right_r, golden


def depth_half(right_bgr, calib, model, input_size=518):
    """Mono stage: DA-V2 on the RAW right image, then warp the depth map AND
    its validity mask into the rectified frame (fusion samples mono at
    rectified golden coords, so the remap lives here, with the model).
    Returns (mono float64, valid bool)."""
    mono_raw = model.infer_image(right_bgr, input_size).astype(np.float64)
    mono = cv2.remap(mono_raw, calib.map_r[0], calib.map_r[1], cv2.INTER_LINEAR)
    valid = cv2.remap(np.ones_like(mono_raw), calib.map_r[0], calib.map_r[1],
                      cv2.INTER_LINEAR) > 0.99
    return mono, valid


def floor_anchors(mono, valid, calib, cam_h, tilt_deg,
                  z_near=0.3, z_far=2.5, n_rows=12, col_band=0.6, pctl=25,
                  golden=None):
    """Synthetic anchor correspondences (disp, mono) from the visible floor.

    The camera sits cam_h metres above the floor at a fixed downward mount
    tilt. A floor pixel at image row v then has a KNOWN metric depth
    Z(v) = cam_h / (dy·cos(tilt) + sin(tilt)),  dy = (v - cy) / fy — so rows
    from the image bottom up toward the horizon give real distances spanning
    z_near..z_far, exactly the far range the golden corners miss when the
    only texture is the near floor.

    Floor extraction per sampled row: a LOW percentile of valid mono inside
    the central col_band (mono is inverse-depth-like, so the floor is the
    smallest value on a row shared with closer obstacles), then a
    monotonicity filter (true floor gets closer, i.e. mono grows, as v grows;
    rows that break the running trend are wall/obstacle, dropped).

    Both extraction filters assume SOME floor is visible on the row — an
    obstacle filling the band defeats them, and a dark cavity that DA-V2
    hallucinates as far defeats them silently (the low percentile then picks
    the cavity, not the floor). The stereo golden points are metric truth, so
    when `golden` is given they veto: a golden point in the band whose stereo
    depth is much closer than the floor depth predicted for its row is an
    OBSTACLE — the floor behind it is occluded, i.e. every row above the
    obstacle's deepest evidence cannot be floor and is cut.

    Returns (disp_anchor, mono_anchor) float64 arrays — empty when fewer
    than 4 rows survive.
    """
    h, w = mono.shape
    t = np.deg2rad(tilt_deg)
    # rows where the floor ray depth lies in [z_near, z_far]
    dy_of = lambda z: (cam_h / z - np.sin(t)) / np.cos(t)   # noqa: E731
    v_far = int(np.clip(calib.cy + dy_of(z_far) * calib.fy, 0, h - 1))
    v_near = int(np.clip(calib.cy + dy_of(z_near) * calib.fy, 0, h - 1))
    if v_near <= v_far + n_rows:
        return np.empty(0), np.empty(0)
    c0, c1 = int(w * (1 - col_band) / 2), int(w * (1 + col_band) / 2)
    v_cut = -1                              # rows v <= v_cut are occluded
    if golden is not None:
        for g in golden:
            if g["disp"] < 2.0 or not (c0 <= g["xr"] <= c1):
                continue
            den = (g["y"] - calib.cy) / calib.fy * np.cos(t) + np.sin(t)
            if den <= 0:
                continue                    # above the horizon — never floor
            if calib.fb / g["disp"] < 0.6 * (cam_h / den):
                v_cut = max(v_cut, int(g["y"]) + 10)   # obstacle + margin
    rows = np.linspace(v_far, v_near, n_rows).astype(int)
    kept_d, kept_m = [], []
    run_max = -np.inf                       # mono must grow from far to near
    for v in rows:                          # far (top) → near (bottom)
        if v <= v_cut:                      # floor occluded by an obstacle
            continue
        band = mono[v, c0:c1][valid[v, c0:c1]]
        if len(band) < 20:
            continue
        m = float(np.percentile(band, pctl))
        if m <= run_max:                    # wall/obstacle row, not floor
            continue
        dy = (v - calib.cy) / calib.fy
        Z = cam_h / (dy * np.cos(t) + np.sin(t))
        if z_near * 0.5 < Z < z_far * 2:
            kept_d.append(calib.fb / Z)
            kept_m.append(m)
            run_max = m
    if len(kept_d) < 4:
        return np.empty(0), np.empty(0)
    return np.asarray(kept_d, np.float64), np.asarray(kept_m, np.float64)


def estimate_tilt(golden, calib, cam_h, img_shape,
                  row_frac=0.70, col_frac=0.35, min_disp_px=5.0):
    """Per-frame camera tilt (rad) from golden points on the near floor.

    The tilt servo can sit at a different angle every run, so a fixed
    mount-tilt constant is unreliable. Golden points in the bottom-center of
    the frame are almost surely floor; their stereo depth is metric, so with
    the measured camera height each one solves
        Z·(dy·cos t + sin t) = cam_h   →   t = asin(cam_h/R) − atan2(Z·dy, Z)
    Median over points; None when fewer than 3 usable (caller falls back to
    its configured tilt).
    """
    h, w = img_shape
    ts = []
    for g in golden:
        if g["disp"] < min_disp_px:
            continue
        if g["y"] < row_frac * h or abs(g["xr"] - calib.cx) > col_frac * w:
            continue
        Z = calib.fb / g["disp"]
        A = Z * (g["y"] - calib.cy) / calib.fy
        R = float(np.hypot(A, Z))
        if R <= cam_h:                       # ray too short to reach the floor
            continue
        t = float(np.arcsin(cam_h / R) - np.arctan2(A, Z))
        if 0.0 < t < np.deg2rad(70):
            ts.append(t)
    return float(np.median(ts)) if len(ts) >= 3 else None


def fuse_metric(golden, mono, valid, calib, min_disp=0.5, min_span_ratio=2.0,
                cam_h=None, tilt_deg=13.0, trust_beyond=1.5):
    """Fusion stage: sample mono at the golden coords, RANSAC affine
    d_mono = s*disp + t, dense pseudo-disparity, Z[m] = f*B / disp.

    Golden points with disp < min_disp are excluded from the fit (negative /
    near-zero disparities are garbage matches that would feed RANSAC).
    When cam_h is given, floor_anchors() pins the fit's far range via
    refit_with_anchors; the camera tilt is estimated per frame from the
    bottom-center golden points (estimate_tilt — the tilt servo can sit
    anywhere), falling back to tilt_deg. Without anchors, an inlier
    disparity span narrower than min_span_ratio (max/min) means the far
    scene would be extrapolated — the frame is rejected instead.

    Raises RuntimeError when THIS frame cannot be fused (<10 usable golden
    points, RANSAC failure, or narrow span with no anchors) — an EXPECTED
    per-frame failure a multi-frame caller may skip while continuing.

    Returns (Z_m float32, valid bool,
             info dict s/t/rms/n_golden/n_inliers/span/n_anchors).
    """
    h, w = mono.shape
    disp, dmono = [], []
    for g in golden:
        if g["disp"] < min_disp:            # garbage match — never fit on it
            continue
        xi, yi = int(round(g["xr"])), int(round(g["y"]))
        if 0 <= xi < w and 0 <= yi < h and valid[yi, xi]:
            disp.append(g["disp"])
            dmono.append(float(mono[yi, xi]))
    if len(disp) < 10:
        raise RuntimeError(f"[ruler] only {len(disp)} golden points — need >=10 "
                           f"(scene too textureless?)")
    disp = np.array(disp)
    dmono = np.array(dmono)
    try:
        s, t, inl, rms, _thr = fit_affine_ransac(disp, dmono)
    except SystemExit as e:            # fit_affine_ransac sys.exit()s on failure
        raise RuntimeError(str(e)) from None

    span = float(disp[inl].max() / max(disp[inl].min(), min_disp))
    n_anchors = 0
    tilt_used = tilt_deg
    if cam_h:
        t_est = estimate_tilt(golden, calib, cam_h, mono.shape)
        if t_est is not None:
            tilt_used = float(np.degrees(t_est))
        ad, am = floor_anchors(mono, valid, calib, cam_h, tilt_used,
                               golden=golden)
        if len(ad):
            # Two-pass: anchors first, then drop golden points that disagree
            # with the anchor-informed line. Aliased stereo matches (NCC on
            # repetitive floor texture) form a cluster INCONSISTENT with the
            # floor ramp; a single weighted LS over both clusters yields a
            # slope flatter than either (Simpson blend) → still-compressed
            # depth. Gate first, blend after.
            A = np.stack([ad, np.ones_like(ad)], axis=1)
            sa, ta = np.linalg.lstsq(A, am, rcond=None)[0]
            gate = max(0.05 * float(np.ptp(np.concatenate([dmono, am]))), 1e-6)
            keep = inl & (np.abs(dmono - (sa * disp + ta)) < gate)
            s, t, rms = refit_with_anchors(np.concatenate([disp[keep], ad]),
                                           np.concatenate([dmono[keep], am]),
                                           np.ones(keep.sum(), bool),
                                           sa, ta, len(ad))
            inl = keep
            n_anchors = len(ad)
            span = float(max(disp[keep].max(), ad.max())
                         / max(min(disp[keep].min(), ad.min()), min_disp)
                         ) if keep.any() else float(ad.max() / ad.min())
    if s <= 0:
        raise RuntimeError(f"[ruler] fit slope s={s:.4f} <= 0 after anchor "
                           f"refit — frame unusable")
    if n_anchors == 0 and span < min_span_ratio:
        raise RuntimeError(
            f"[ruler] inlier disparity span x{span:.2f} < x{min_span_ratio:.1f}"
            f" — all golden points at similar depth; far scene would be "
            f"extrapolated (no floor anchors available)")

    # Trust horizon: the metric evidence (golden inliers + anchors) only
    # reaches so deep — beyond it the affine line is pure extrapolation and
    # DA-V2's hallucinated "very far" regions (dark cavities, glass) land at
    # arbitrary depths. Such pixels are marked UNRESOLVED instead of guessed:
    # an honest hole in the map beats a wall at a made-up distance (the robot
    # re-observes it when closer).
    evid_disp = np.concatenate([disp[inl], ad]) if n_anchors else disp[inl]
    z_trust = trust_beyond * calib.fb / max(float(evid_disp.min()), min_disp)

    disp_dense = np.clip((mono - t) / s, 0.0, None)
    resolvable = disp_dense > max(min_disp, calib.fb / z_trust)
    Z = np.where(resolvable, calib.fb / np.maximum(disp_dense, min_disp), 0.0)
    return (Z.astype(np.float32), valid & resolvable,
            {"s": s, "t": t, "rms": rms, "n_golden": len(disp),
             "n_inliers": int(inl.sum()), "span": span, "z_trust": z_trust,
             "n_anchors": n_anchors, "tilt_deg": tilt_used})


def compute_metric_depth(left_bgr, right_bgr, calib, model, input_size=518,
                         d_max=140, block=11, ncc_min=0.70, min_disp=0.5,
                         min_span_ratio=2.0, cam_h=None, tilt_deg=13.0):
    """Full stereo-ruler pipeline for ONE pair, as a library call.

    calib = load_rectify(path, (w, h)) and model = load_model(...)[0] are
    created ONCE by the caller and reused across frames (no model reload).

    = stereo_half + depth_half + fuse_metric run back to back (the pipeline
    runs the same three pieces on separate threads — no logic duplicated).

    Returns (Z_m, valid, info):
      Z_m    float32 HxW metric depth in the RIGHT-rectified frame (0 where invalid)
      valid  bool HxW — inside the rectified warp AND disparity resolvable
      info   dict: s, t, rms, n_golden, n_inliers, left_rect, right_rect
    Raises RuntimeError if too few golden points survive to fit.
    """
    left_r, right_r, golden = stereo_half(left_bgr, right_bgr, calib,
                                          d_max=d_max, block=block,
                                          ncc_min=ncc_min)
    mono, valid = depth_half(right_bgr, calib, model, input_size)
    Z, ok, info = fuse_metric(golden, mono, valid, calib, min_disp,
                              min_span_ratio=min_span_ratio,
                              cam_h=cam_h, tilt_deg=tilt_deg)
    info["left_rect"] = left_r
    info["right_rect"] = right_r
    return Z, ok, info


# ---------------------------------------------------------------- scale fit --
def load_anchors(path, fb, mono, w, h):
    """Manual ground-truth points: x,y (left-rectified frame, as seen in
    golden_overlay.jpg) + z_m (real measured distance). Used to correct the
    affine fit across the FULL depth range, since auto golden points tend to
    cluster near-field and the fit then extrapolates badly at far distances.
    Returns list of dicts matching the golden_points() schema (disp, d_mono).
    """
    if fb is None or not fb:
        sys.exit("[ruler] --anchors requires --fb (or stereo_rectify.yml) to "
                 "convert real z_m into an expected disparity.")
    out = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            x, y, z_m = float(row["x"]), float(row["y"]), float(row["z_m"])
            if z_m <= 0:
                continue
            disp_a = fb / z_m
            xr = x - disp_a
            xri, yi = int(round(xr)), int(round(y))
            if not (0 <= xri < w and 0 <= yi < h):
                print(f"[ruler] anchor ({x:.0f},{y:.0f} z={z_m}m) -> xr={xr:.1f} "
                      f"out of bounds, skipped")
                continue
            out.append({"x": x, "y": y, "xr": float(xr), "disp": float(disp_a),
                        "ncc": 1.0, "d_mono": float(mono[yi, xri]),
                        "z_m": z_m, "anchor": True})
    return out


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


def refit_with_anchors(disp, mono, inl, s0, t0, n_anchors, anchor_weight=5.0):
    """Weighted least-squares refit: RANSAC-inlier auto points (weight 1) plus
    manual anchors (last n_anchors rows of disp/mono, weight anchor_weight).
    Anchors are real measured distances spanning near+far, so they pull the
    fit's (s, t) to match the true depth range instead of only the near-field
    cluster the automatic golden points tend to find.
    """
    n = len(disp)
    mask = best = inl.copy()
    if n_anchors:
        mask = np.concatenate([inl, np.ones(n_anchors, dtype=bool)])
    w = np.ones(mask.sum())
    if n_anchors:
        w[-n_anchors:] = anchor_weight
    d = disp[mask]
    m = mono[mask]
    sw = np.sqrt(w)
    A = np.stack([d, np.ones_like(d)], axis=1) * sw[:, None]
    b = m * sw
    s, t = np.linalg.lstsq(A, b, rcond=None)[0]
    r = np.abs(mono - (s * disp + t))
    rms = float(np.sqrt(np.mean((r[mask] * w / w.max()) ** 2)))
    return float(s), float(t), rms


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
        is_anchor = g.get("anchor", False)
        u = (g["disp"] - lo) / (hi - lo)
        color = tuple(int(c) for c in cv2.applyColorMap(
            np.array([[int(u * 255)]], np.uint8), cv2.COLORMAP_JET)[0, 0])
        x, y = int(g["x"]), int(g["y"])
        if is_anchor:
            cv2.drawMarker(out, (x, y), (0, 255, 255), cv2.MARKER_SQUARE, 14, 2)
        else:
            cv2.circle(out, (x, y), 6, color, 2)
        if not ok:
            cv2.line(out, (x - 8, y - 8), (x + 8, y + 8), (0, 0, 255), 2)
            continue                               # label only inlier points
        if fb and g["disp"] < 2.0:
            label = "far"              # <2px disparity: distance unresolvable
        elif is_anchor:
            dist = _point_distance(g["disp"], fb) * norm
            label = f"{dist:.2f}{unit} (meas {g['z_m']:.2f})"
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
    p.add_argument("--camera", action="store_true",
                   help="grab a live stereo pair from the cameras instead of "
                        "reading --left/--right image files")
    p.add_argument("--device-left", type=int, default=_LEFT_IDX,
                   help="left camera index (/dev/videoN)")
    p.add_argument("--device-right", type=int, default=_RIGHT_IDX,
                   help="right camera index (/dev/videoN)")
    p.add_argument("--cam-width", type=int, default=_CAM_W)
    p.add_argument("--cam-height", type=int, default=_CAM_H)
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
    p.add_argument("--anchors", default=None,
                   help="CSV (x,y,z_m) of manually measured real distances, in "
                        "the left-rectified frame (pixel coords as shown in "
                        "golden_overlay.jpg). Span near AND far to correct the "
                        "affine fit's extrapolation. Requires --fb.")
    p.add_argument("--anchor-weight", type=float, default=5.0,
                   help="relative weight of each anchor vs. an auto golden "
                        "inlier in the refit (default 5)")
    args = p.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.camera:
        left, right = capture_pair(args.device_left, args.device_right,
                                   args.cam_width, args.cam_height)
        cv2.imwrite(str(out_dir / "capture_left.jpg"), left)
        cv2.imwrite(str(out_dir / "capture_right.jpg"), right)
        print(f"[ruler] captured live pair from /dev/video{args.device_left} "
              f"(L) & /dev/video{args.device_right} (R) -> saved to {out_dir}")
    else:
        left = cv2.imread(args.left)
        right = cv2.imread(args.right)
        if left is None or right is None:
            sys.exit(f"[ruler] cannot read {args.left} / {args.right}")

    # 1 — bring both images into a row-aligned frame
    h, w = left.shape[:2]
    fb = args.fb
    if Path(args.rectify).exists():
        rect = load_rectify(args.rectify, (w, h))
        map_l, map_r = rect.map_l, rect.map_r
        left = cv2.remap(left, map_l[0], map_l[1], cv2.INTER_LINEAR)
        right_al = cv2.remap(right, map_r[0], map_r[1], cv2.INTER_LINEAR)
        warp_mono = lambda m: cv2.remap(m, map_r[0], map_r[1], cv2.INTER_LINEAR)
        if fb is None:
            fb = rect.fb
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
    right_name = "live capture" if args.camera else Path(args.right).name
    print(f"[ruler] DA-V2 {args.encoder} on {right_name} (device={device})")
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

    # 4 — robust affine fit: the ruler (auto golden points only)
    s, t, inl, rms, thr = fit_affine_ransac(disp, dmono)
    print(f"[ruler] d_mono = s*disp + t : s={s:.5f}  t={t:.4f}  "
          f"inliers={int(inl.sum())}/{len(golden)}  rms={rms:.4f} (thr={thr:.4f})")

    n_anchors = 0
    if args.anchors:
        anchors = load_anchors(args.anchors, fb, mono, w, h)
        n_anchors = len(anchors)
        if n_anchors:
            disp = np.concatenate([disp, [a["disp"] for a in anchors]])
            dmono = np.concatenate([dmono, [a["d_mono"] for a in anchors]])
            golden = golden + anchors
            inl = np.concatenate([inl, np.zeros(n_anchors, dtype=bool)])
            s, t, rms = refit_with_anchors(disp, dmono, inl, s, t, n_anchors,
                                            args.anchor_weight)
            inl = np.concatenate([inl[:-n_anchors], np.ones(n_anchors, dtype=bool)])
            print(f"[ruler] refit with {n_anchors} anchor(s) (weight="
                  f"{args.anchor_weight}): s={s:.5f}  t={t:.4f}  rms={rms:.4f}")
        else:
            print("[ruler] no usable anchors (all out of bounds) — keeping "
                  "auto-only fit")

    print(f"[ruler] pseudo-disparity of ANY right pixel: disp_px = (d_mono - t) / s")
    if fb:
        print(f"[ruler] metric: Z[m] = {fb:.4f} / disp_px")
    else:
        print(f"[ruler] no f*B -> depth stays in disparity units "
              f"(Z proportional to 1/disp_px).")

    # 5 — outputs
    with open(out_dir / "golden_points.csv", "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=["x", "y", "xr", "disp", "ncc",
                                           "d_mono", "inlier", "anchor", "z_m"],
                            extrasaction="ignore")
        wr.writeheader()
        for g, ok in zip(golden, inl):
            row = {k: (f"{v:.3f}" if isinstance(v, float) else v)
                   for k, v in g.items()}
            row["inlier"] = int(ok)
            row.setdefault("anchor", 0)
            wr.writerow(row)

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
    fs.write("anchors", n_anchors)
    fs.release()
    print(f"[ruler] outputs -> {out_dir}")


if __name__ == "__main__":
    main()
