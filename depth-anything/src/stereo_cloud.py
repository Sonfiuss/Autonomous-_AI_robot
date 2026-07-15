#!/usr/bin/env python3
"""
stereo_cloud.py — live stereo pair + DA-V2 → metric point cloud (XYZ, no color).

Task 2026-07-13_stereo-cloud-live. The camera looks LEVEL now — the floor-anchor
path of stereo_ruler (cam_h / tilt) does not apply and is not used.

Everything runs GRAYSCALE for speed; the cloud carries no color.

Pipeline:
  1. Capture a pair straight from the cameras at the calibration size
     (stereo_rectify.yml is 1280x720 — a size mismatch is a hard error),
     or --left/--right for saved images.
  2. Rectify single-channel with stereo_rectify.yml (epipolar horizontal).
  3. DENSE golden points (fine grid) = many independent metric stereo
     measurements spread over the frame.
  4. DA-V2 relative inverse depth on the RAW right image, warped to the
     rectified frame (same convention as stereo_ruler.depth_half).
  5. Global RANSAC affine fit  d_mono = s*disp + t  (the ruler). Stereo
     disparities are first corrected by --disp-offset (rig yaw drifted since
     calibration; measured +51px on 2026-07-14). Golden points whose stereo
     depth disagrees with the mono prediction are rejected — the cloud
     geometry comes from DA-V2; stereo only pins the metric scale.
  6. LOCAL residual interpolation — the new step: at each golden inlier the
     leftover disparity error  Δ = disp_stereo − disp_fit  is interpolated
     into a smooth field Δ(u,v) over the image (griddata on a coarse grid,
     decaying to 0 away from evidence), correcting DA-V2's local scale
     wobble instead of trusting one global line. Validated on a 20% holdout
     of the golden points (rms before vs after is printed).
  7. Back-project with the rectified P2 intrinsics → binary PLY (xyz only,
     MILLIMETRES; depth_metric_m.npy stays in metres).

Usage:
  python3 stereo_cloud.py --camera            # live pair from /dev/video0+2
  python3 stereo_cloud.py --left L.jpg --right R.jpg
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import cv2

_HERE = Path(__file__).resolve().parent            # depth-anything/src
sys.path.insert(0, str(_HERE))
import stereo_ruler as sr                          # noqa: E402

_DA_ROOT = _HERE.parent
_REPO = _DA_ROOT.parent
_DEF_RECTIFY = _REPO / "stereo-camera" / "calib" / "stereo_rectify.yml"
_DEF_LEFT = _REPO / "stereo-camera" / "tools" / "captures" / "left.jpg"
_DEF_RIGHT = _REPO / "stereo-camera" / "tools" / "captures" / "righ.jpg"
_DEF_OUT = _DA_ROOT / "output" / "stereo_cloud"


def calib_image_size(path):
    """(width, height) the calibration was made at — captures must match."""
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    fs.release()
    if w <= 0 or h <= 0:
        sys.exit(f"[cloud] image_width/image_height missing in {path}")
    return w, h


# ------------------------------------------------------ local residual field --
# NOTE: system scipy (1.3.3) is broken against the local numpy (np.int gone),
# so scattered linear interpolation uses matplotlib.tri (Delaunay) instead.
def _tri_linear(pts_xy, vals, qx, qy):
    """Scattered linear interpolation at query points. Returns (values,
    inside_hull mask); None when triangulation is degenerate (collinear)."""
    import matplotlib.tri as mtri
    try:
        tri = mtri.Triangulation(pts_xy[:, 0], pts_xy[:, 1])
        lin = mtri.LinearTriInterpolator(tri, vals)(qx, qy)
    except (ValueError, RuntimeError):
        return None
    inside = ~np.ma.getmaskarray(lin)
    return np.asarray(np.ma.filled(lin, 0.0)), inside


def _nearest_vals(pts_xy, vals, q):
    """Value of the nearest measurement for each query point (Nx2)."""
    d2 = ((q[:, None, :] - pts_xy[None, :, :]) ** 2).sum(-1)
    return vals[np.argmin(d2, axis=1)]


def residual_field(pts_xy, dres, shape, cell=8, tau_px=120.0, blur_sigma=2.0):
    """Interpolate sparse disparity residuals into a dense field Δ(u,v).

    Runs on a coarse grid (1 sample per `cell` px) then resizes up —
    interpolating 1280x720 directly is ~64x slower for no accuracy gain (the
    field is smooth by construction). Inside the convex hull of the
    measurements the field is the linear interpolation; outside it decays to
    0 with length scale tau_px, because extrapolating a correction beyond all
    evidence is guessing — far from any golden point the global fit is the
    best we know. Returns zeros when triangulation fails.
    """
    h, w = shape
    gx, gy = np.meshgrid(np.arange(0, w, cell, dtype=np.float64),
                         np.arange(0, h, cell, dtype=np.float64))
    r = _tri_linear(pts_xy, dres, gx, gy)
    if r is None:
        return np.zeros((h, w), np.float32)
    lin, inside = r
    q = np.stack([gx[~inside], gy[~inside]], axis=1)
    near = np.zeros_like(lin)
    if len(q):
        near[~inside] = _nearest_vals(pts_xy, dres, q)
    # distance (px) from each outside cell to the hull, for the decay weight
    dist = cv2.distanceTransform((~inside).astype(np.uint8), cv2.DIST_L2, 3)
    fld = np.where(inside, lin, near * np.exp(-dist * cell / tau_px))
    fld = cv2.GaussianBlur(fld.astype(np.float32), (0, 0), blur_sigma)
    return cv2.resize(fld, (w, h), interpolation=cv2.INTER_LINEAR)


def field_at(pts_train, res_train, pts_eval):
    """Field values at scattered eval points (for holdout validation).
    Returns None when triangulation is degenerate."""
    r = _tri_linear(pts_train, res_train, pts_eval[:, 0], pts_eval[:, 1])
    if r is None:
        return None
    lin, inside = r
    out = lin.copy()
    if (~inside).any():
        out[~inside] = _nearest_vals(pts_train, res_train, pts_eval[~inside])
    return out


def neighbor_consistent(pxy, res, k=5, gate=np.log(1.35)):
    """Keep measurements whose residual agrees with their spatial neighbors.

    Aliased NCC matches on repetitive texture (the granite-speckle floor)
    pass the LR-consistency check but land at a disparity off by a texture
    period — an error their non-aliased neighbors do not share. A point is
    kept when its log-ratio residual is within `gate` of the median residual
    of its k nearest (image-plane) neighbors. Genuine local DA-V2 wobble IS
    shared by neighbors, so it survives — that is exactly the signal the
    residual field interpolates.
    """
    n = len(res)
    if n <= k:
        return np.ones(n, bool)
    d2 = ((pxy[:, None, :] - pxy[None, :, :]) ** 2).sum(-1)
    np.fill_diagonal(d2, np.inf)
    nb = np.argsort(d2, axis=1)[:, :k]
    med = np.median(res[nb], axis=1)
    return np.abs(res - med) < gate


# --------------------------------------------------------------- point cloud --
def depth_to_cloud(Z, ok, calib, stride=2):
    """Back-project metric depth (right-rectified frame) → Nx3 float32 XYZ.
    Camera frame: +X right, +Y down, +Z forward."""
    Zs = Z[::stride, ::stride]
    oks = ok[::stride, ::stride]
    h, w = Z.shape
    us, vs = np.meshgrid(np.arange(0, w, stride, dtype=np.float32),
                         np.arange(0, h, stride, dtype=np.float32))
    z = Zs[oks].astype(np.float32)
    u = us[oks]
    v = vs[oks]
    x = (u - calib.cx) * z / calib.fx
    y = (v - calib.cy) * z / calib.fy
    return np.stack([x, y, z], axis=1)


def save_ply_xyz(path, pts):
    """Binary little-endian PLY, xyz only — no color, fast to write/load."""
    header = ("ply\nformat binary_little_endian 1.0\n"
              f"element vertex {len(pts)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "end_header\n")
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())


# ----------------------------------------------------------------------- main --
def main():
    p = argparse.ArgumentParser(
        description="Stereo-anchored DA-V2 metric point cloud (grayscale).")
    p.add_argument("--camera", action="store_true",
                   help="grab a live pair instead of reading --left/--right")
    p.add_argument("--device-left", type=int, default=sr._LEFT_IDX)
    p.add_argument("--device-right", type=int, default=sr._RIGHT_IDX)
    p.add_argument("--left", default=str(_DEF_LEFT))
    p.add_argument("--right", default=str(_DEF_RIGHT))
    p.add_argument("--rectify", default=str(_DEF_RECTIFY))
    p.add_argument("--encoder", default="vits", choices=list(sr._CFGS))
    p.add_argument("--input-size", type=int, default=518,
                   help="DA-V2 inference size (multiple of 14; lower = faster)")
    p.add_argument("--device", default="auto", choices=["auto", "cuda", "cpu"])
    p.add_argument("--baseline-m", type=float, default=None,
                   help="MEASURED distance between the two camera optical "
                        "centers, metres. stereo_rectify.yml assumed 5.4cm "
                        "to pin its scale — if the rig changed, every Z is "
                        "off by B_real/0.054. This recomputes f*B = fx * B.")
    p.add_argument("--disp-offset", type=float, default=51.0,
                   help="constant disparity bias in px, SUBTRACTED from every "
                        "stereo match before Z=fb/disp. The cameras' relative "
                        "yaw drifted ~2.2deg since stereo_rectify.yml "
                        "(2026-07-02); +51px was measured 2026-07-14 against "
                        "known-distance chessboard pairs + a tape-measured "
                        "wall. Set to 0 after recalibrating.")
    p.add_argument("--d-max", type=int, default=140)
    p.add_argument("--block", type=int, default=11)
    p.add_argument("--ncc-min", type=float, default=0.70)
    p.add_argument("--min-disp", type=float, default=0.5)
    p.add_argument("--z-max", type=float, default=8.0,
                   help="golden points implying Z beyond this are garbage "
                        "matches (indoor default 8m) and are dropped")
    p.add_argument("--min-span-ratio", type=float, default=1.5,
                   help="reject frames whose metric evidence covers less "
                        "than this depth ratio (truly single-depth scenes). "
                        "The trust horizon already clips extrapolation, so "
                        "this only guards degenerate frames.")
    p.add_argument("--trust-beyond", type=float, default=1.2,
                   help="pixels deeper than this x farthest metric evidence "
                        "are marked unresolved instead of extrapolated")
    p.add_argument("--mono-gate", type=float, default=2.0,
                   help="reject a golden point when its stereo depth differs "
                        "from the mono-fit prediction by more than this "
                        "ratio. Mono is the noise filter: a lone stereo "
                        "match claiming 12m where DA-V2 sees a nearby wall "
                        "is a bad match, not a measurement.")
    p.add_argument("--tau-px", type=float, default=120.0,
                   help="decay length of the local correction outside the "
                        "golden-point hull")
    p.add_argument("--no-local", action="store_true",
                   help="skip the local residual field (global fit only, A/B)")
    p.add_argument("--stride", type=int, default=2,
                   help="cloud subsample step in px")
    p.add_argument("--out-dir", default=str(_DEF_OUT))
    args = p.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    times = {}

    # 0 — calibration (defines the mandatory capture size)
    if not Path(args.rectify).exists():
        sys.exit(f"[cloud] {args.rectify} not found — run "
                 f"stereo_calibrate_2view.py first")
    w, h = calib_image_size(args.rectify)
    calib = sr.load_rectify(args.rectify, (w, h))
    if args.baseline_m:
        calib.fb = calib.fx * args.baseline_m
        print(f"[cloud] baseline override {args.baseline_m*100:.1f}cm -> "
              f"f*B={calib.fb:.2f}")
    print(f"[cloud] calib {Path(args.rectify).name}: {w}x{h}  "
          f"f*B={calib.fb:.2f}  fx={calib.fx:.1f}")

    # 1 — grayscale pair at calib size
    t0 = time.perf_counter()
    if args.camera:
        left, right = sr.capture_pair(args.device_left, args.device_right, w, h)
        gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(str(out_dir / "capture_left.jpg"), gray_l)
        cv2.imwrite(str(out_dir / "capture_right.jpg"), gray_r)
        print(f"[cloud] live pair /dev/video{args.device_left} (L) + "
              f"/dev/video{args.device_right} (R)")
    else:
        gray_l = cv2.imread(args.left, cv2.IMREAD_GRAYSCALE)
        gray_r = cv2.imread(args.right, cv2.IMREAD_GRAYSCALE)
        if gray_l is None or gray_r is None:
            sys.exit(f"[cloud] cannot read {args.left} / {args.right}")
    if gray_l.shape != (h, w) or gray_r.shape != (h, w):
        sys.exit(f"[cloud] pair is {gray_l.shape[1]}x{gray_l.shape[0]} but "
                 f"calibration is {w}x{h} — capture at the calib size")
    times["capture"] = time.perf_counter() - t0

    # 2 — rectify (single channel)
    t0 = time.perf_counter()
    rect_l = cv2.remap(gray_l, calib.map_l[0], calib.map_l[1], cv2.INTER_LINEAR)
    rect_r = cv2.remap(gray_r, calib.map_r[0], calib.map_r[1], cv2.INTER_LINEAR)
    times["rectify"] = time.perf_counter() - t0

    # 3 — dense golden points: many metric measurements across the frame.
    # This rig's baseline is tilted ~27°, so rectification ROTATES both views
    # and leaves big slanted black borders — and the borders sit at different
    # places in each view (R1 != R2). A corner on the left border edge NCC-
    # matches the right border edge at a garbage disparity, so any golden
    # point whose patch touches the invalid zone of EITHER view is dropped.
    t0 = time.perf_counter()
    ones = np.ones((h, w), np.float32)
    k = np.ones((args.block + 4, args.block + 4), np.uint8)
    val_l = cv2.erode((cv2.remap(ones, calib.map_l[0], calib.map_l[1],
                                 cv2.INTER_LINEAR) > 0.99).astype(np.uint8), k)
    val_r = cv2.erode((cv2.remap(ones, calib.map_r[0], calib.map_r[1],
                                 cv2.INTER_LINEAR) > 0.99).astype(np.uint8), k)
    golden = sr.golden_points(rect_l, rect_r, d_max=args.d_max,
                              block=args.block, ncc_min=args.ncc_min,
                              grid=(24, 14), per_cell=4)
    n_raw = len(golden)
    golden = [g for g in golden
              if 0 <= int(g["xr"]) < w
              and val_l[int(g["y"]), int(g["x"])]
              and val_r[int(g["y"]), int(g["xr"])]]
    # yaw-drift correction: every raw match is biased by a constant +offset px.
    # Done HERE so every consumer (z-max gate, affine fit, field, overlay)
    # sees the corrected disparity. Points at disp <= offset are unresolvable
    # (denominator ~0 explodes Z) — the disp_floor gate below drops them.
    if args.disp_offset:
        for g in golden:
            g["disp"] -= args.disp_offset
    times["golden"] = time.perf_counter() - t0
    print(f"[cloud] {len(golden)} golden points ({n_raw - len(golden)} border-"
          f"garbage dropped, disp-offset {args.disp_offset:+.1f}px, "
          f"{times['golden']:.2f}s)")

    # 4 — DA-V2 on the RAW right image (gray→3ch), warp to rectified frame
    t0 = time.perf_counter()
    model, device = sr.load_model(args.encoder, args.device)
    times["model_load"] = time.perf_counter() - t0
    t0 = time.perf_counter()
    right_3ch = cv2.cvtColor(gray_r, cv2.COLOR_GRAY2BGR)
    mono_raw = model.infer_image(right_3ch, args.input_size).astype(np.float64)
    mono = cv2.remap(mono_raw, calib.map_r[0], calib.map_r[1], cv2.INTER_LINEAR)
    valid = cv2.remap(np.ones_like(mono_raw), calib.map_r[0], calib.map_r[1],
                      cv2.INTER_LINEAR) > 0.99
    times["da_v2"] = time.perf_counter() - t0
    print(f"[cloud] DA-V2 {args.encoder}@{args.input_size} on {device} "
          f"({times['da_v2']:.2f}s)")

    # 5 — global affine fit (no floor anchors: camera is level)
    t0 = time.perf_counter()
    disp_floor = max(args.min_disp, calib.fb / args.z_max)
    disp, dmono, pxy, kept_golden = [], [], [], []
    for g in golden:
        if g["disp"] < disp_floor:          # implies Z > z_max: garbage match
            continue
        xi, yi = int(round(g["xr"])), int(round(g["y"]))
        if 0 <= xi < w and 0 <= yi < h and valid[yi, xi]:
            disp.append(g["disp"])
            dmono.append(float(mono[yi, xi]))
            pxy.append((g["xr"], g["y"]))
            kept_golden.append(g)
    disp = np.asarray(disp)
    dmono = np.asarray(dmono)
    pxy = np.asarray(pxy)

    def reject(msg):
        """Frame unusable — still save the overlay so the user can SEE why
        (where the measurements landed), then exit loudly."""
        cv2.imwrite(str(out_dir / "golden_overlay.jpg"),
                    sr._overlay(cv2.cvtColor(rect_l, cv2.COLOR_GRAY2BGR),
                                kept_golden,
                                np.zeros(len(kept_golden), bool),
                                fb=calib.fb))
        sys.exit(f"\n[cloud] ============ FRAME REJECTED ============\n"
                 f"[cloud] {msg}\n"
                 f"[cloud] No cloud.ply written. Capture + golden_overlay "
                 f"saved in {out_dir}.\n"
                 f"[cloud] Aim at a scene with BOTH near and far texture, "
                 f"or relax --min-span-ratio.")

    if len(disp) < 10:
        reject(f"only {len(disp)} usable golden points — need >=10 "
               f"(scene too textureless?)")
    try:
        s, t, inl, rms, _thr = sr.fit_affine_ransac(disp, dmono)
    except SystemExit as e:            # fit_affine_ransac sys.exit()s
        reject(str(e))
    span_inl = float(disp[inl].max() / max(disp[inl].min(), args.min_disp))
    print(f"[cloud] fit s={s:.5f} t={t:.4f} inliers={int(inl.sum())}/"
          f"{len(disp)} rms={rms:.4f} inlier-span=x{span_inl:.1f}")

    # 6 — local residual field. Residuals live in LOG-RATIO space:
    # field = log(disp_stereo / disp_fit), applied multiplicatively. An
    # additive px correction is meaningless across depths (at disp=3px a
    # +2px "fix" halves Z; at disp=100px it is noise) — a ratio means the
    # same relative depth error everywhere.
    # ALL golden survivors feed the field, not only RANSAC inliers: DA-V2's
    # local scale wobble makes a globally-affine fit miss whole regions, and
    # those "outliers" are precisely the correction signal. Aliased speckle
    # matches are removed by the neighbor-consistency gate instead.
    disp_fit_g = np.clip((dmono - t) / s, args.min_disp, None)
    res_all = np.log(disp / disp_fit_g)
    keep = (neighbor_consistent(pxy, res_all)
            & (np.abs(res_all) < np.log(args.mono_gate)))
    di, pi, res = disp[keep], pxy[keep], res_all[keep]
    print(f"[cloud] residual field points: {int(keep.sum())}/{len(disp)} "
          f"(neighbor-consistent + mono-gate x{args.mono_gate:.1f})")
    # trust horizon = deepest surviving metric evidence; also capped on the
    # near side (closer than any evidence x 1.5 = extrapolation too).
    # FAR side uses the 5th disparity percentile, not the minimum: one stray
    # deep golden point must not drag the horizon out and let mono
    # extrapolate a phantom volume behind the real back wall. The near side
    # keeps the raw extreme — near evidence is dense and reliable (floor).
    evid_min = (float(np.percentile(di, 5)) if len(di)
                else float(disp[inl].min()))
    evid_max = float(di.max()) if len(di) else float(disp[inl].max())
    z_trust = args.trust_beyond * calib.fb / max(evid_min, args.min_disp)
    disp_hi = 1.5 * evid_max
    print(f"[cloud] z_trust={calib.fb / disp_hi:.2f}..{z_trust:.2f}m")
    field = np.zeros((h, w), np.float32)
    field_used = False
    if not args.no_local and len(res) >= 8:
        # holdout: does interpolating train residuals predict test residuals?
        rng = np.random.default_rng(0)
        idx = rng.permutation(len(res))
        n_te = max(len(res) // 5, 2)
        te, tr = idx[:n_te], idx[n_te:]
        pred = field_at(pi[tr], res[tr], pi[te])
        rms_g = float(np.sqrt(np.mean(res[te] ** 2)))
        rms_l = float(np.sqrt(np.mean((res[te] - pred) ** 2))) \
            if pred is not None else np.inf
        pct = lambda r: (np.exp(r) - 1) * 100          # noqa: E731
        print(f"[cloud] holdout depth-ratio rms: global-only "
              f"{pct(rms_g):.1f}% -> local {pct(rms_l):.1f}% "
              f"({len(tr)} train / {len(te)} test)")
        if rms_l < rms_g:
            field = residual_field(pi, res, (h, w), tau_px=args.tau_px)
            lo, hi = float(res.min()), float(res.max())
            field = np.clip(field, lo, hi)      # never exceed the evidence
            field_used = True
        else:
            print("[cloud] local field does not generalize on this frame — "
                  "keeping global fit only")
    # Span gate on the evidence that actually constrains the output: the
    # field points when the field is in use (they pin near AND far regions
    # regardless of which cluster RANSAC latched onto — RANSAC's inlier set
    # varies run to run on wobbly mono), else the RANSAC inliers.
    span = (float(di.max() / max(float(di.min()), args.min_disp))
            if field_used else span_inl)
    if span < args.min_span_ratio:
        reject(f"metric evidence span x{span:.2f} < x{args.min_span_ratio:.1f}"
               f" — all measurements at similar depth, the rest of the scene "
               f"would be extrapolated.")
    print(f"[cloud] evidence span x{span:.1f} "
          f"({'field' if field_used else 'inliers'})")
    times["fit"] = time.perf_counter() - t0

    # 7 — dense metric depth + cloud
    t0 = time.perf_counter()
    disp_dense = np.clip((mono - t) / s, 0.0, None) * np.exp(field)
    resolvable = ((disp_dense > max(args.min_disp, calib.fb / z_trust))
                  & (disp_dense < disp_hi))
    ok = valid & resolvable
    Z = np.where(ok, calib.fb / np.maximum(disp_dense, args.min_disp),
                 0.0).astype(np.float32)
    # depth-edge smear: mono blurs object boundaries into radial "curtains"
    # of points connecting foreground to background (on the test frame 11%
    # of points landed BEHIND the tape-measured wall). A real surface seen
    # at this resolution changes depth < ~8%/px; smears jump far more. Cut
    # high-gradient pixels (dilated to take the whole curtain with them).
    logz = np.log(np.where(Z > 0, Z, 1.0)).astype(np.float32)
    gmag = np.hypot(cv2.Sobel(logz, cv2.CV_32F, 1, 0, ksize=3),
                    cv2.Sobel(logz, cv2.CV_32F, 0, 1, ksize=3))
    smear = cv2.dilate((gmag > 8 * 0.08).astype(np.uint8),
                       np.ones((3, 3), np.uint8)).astype(bool)
    n_ok = int(ok.sum())
    ok &= ~smear
    print(f"[cloud] edge-smear filter: {n_ok - int(ok.sum())} px dropped "
          f"({(n_ok - int(ok.sum())) / max(n_ok, 1) * 100:.1f}%)")
    pts = depth_to_cloud(Z, ok, calib, stride=args.stride)
    save_ply_xyz(out_dir / "cloud.ply", pts * 1000.0)   # PLY unit: mm
    times["cloud"] = time.perf_counter() - t0

    # depth error at the measurements (what the stereo ruler says is truth)
    zs = calib.fb / di
    fld_i = field[pi[:, 1].astype(int), pi[:, 0].astype(int)]
    zp = calib.fb / np.clip(di * np.exp(fld_i - res), args.min_disp, None)
    err = np.abs(zp - zs)
    print(f"[cloud] |Z err| at golden inliers: mean {err.mean()*100:.1f}cm  "
          f"median {np.median(err)*100:.1f}cm  (Z range "
          f"{zs.min():.2f}..{zs.max():.2f}m)")

    # 8 — debug outputs (visualization only — the cloud itself has no color)
    np.save(out_dir / "depth_metric_m.npy", Z)
    zv = np.clip(Z / z_trust, 0, 1)
    vis = cv2.applyColorMap((zv * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    vis[~ok] = 0
    cv2.imwrite(str(out_dir / "depth_vis.jpg"), vis)
    # overlay marks the points that fed the residual field (keep) as inliers
    cv2.imwrite(str(out_dir / "golden_overlay.jpg"),
                sr._overlay(cv2.cvtColor(rect_l, cv2.COLOR_GRAY2BGR),
                            kept_golden, keep, fb=calib.fb))

    total = sum(times.values())
    stage = "  ".join(f"{k}={v:.2f}s" for k, v in times.items())
    print(f"[cloud] {len(pts)} points (mm) -> {out_dir / 'cloud.ply'}")
    print(f"[cloud] timing: {stage}  total={total:.2f}s")


if __name__ == "__main__":
    main()
