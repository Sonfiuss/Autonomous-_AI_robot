#!/usr/bin/env python3
"""
stereo_calibrate_2view.py — metric stereo calibration.

TWO paths:

PRINTED-BOARD path (default, Stage B 2026-07-11) — full 9×6 board printed at a
known square size (make_checkerboard.py, ruler-verified), 20–25 pairs spread
over 0.4–2 m with ±20–30° skews and full-frame corner coverage:
  - findChessboardCornersSB + cornerSubPix per image (calib_io.find_board)
  - PER-CAMERA intrinsics (the two USB cams are different physical units —
    the old pooled single-K was a modeling error): k1,k2 free, k3+tangential
    zero, principal point free, aspect fixed
  - per-view reprojection report + worst-view rejection (≤2 rounds, keeps ≥12)
  - stereoCalibrate(FIX_INTRINSIC) on metric object points → T in metres
  - stereoRectify(alpha=-1)  (alpha=0 degenerates on this tilted-baseline rig:
    valid-ROI crop explodes the rectified focal to ~166000 px)
  - VERSIONED output calib/stereo_rectify_YYYYMMDD.yml (never overwrites the
    active yml — promote by copying after epipolar_check passes) + corner
    coverage heatmap jpg

LEGACY-SCREEN path (--legacy-screen) — the original 2026-07-02 flow for
partially-visible laptop-screen boards (align_from_chessboard grid-shift +
pooled focal + baseline-pinned scale). Kept for reproducibility only.

Usage:
  python stereo_calibrate_2view.py --session captures/calib_20260712 --square-mm 25.0
  python stereo_calibrate_2view.py --legacy-screen --baseline 0.054
"""
import argparse
import sys
from datetime import date
from pathlib import Path

import numpy as np
import cv2

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
import calib_io  # noqa: E402

_DEF_CAPTURES = _HERE / "captures"
_CALIB_DIR = _HERE.parent / "calib"
_HAND_BASELINE_M = 0.054            # hand-measured lens-to-lens, sanity ref


# ─────────────────────────────────────────────────────────── printed board ───
def detect_all(session, pattern):
    """→ (views, size): views = list of dicts(tag, corners_l, corners_r)."""
    views, size = [], None
    for tag, lp, rp, _z in calib_io.iter_pairs(session):
        left, right = calib_io.read_pair(lp, rp)
        gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        if size is None:
            size = gl.shape[::-1]
        elif gl.shape[::-1] != size:
            sys.exit(f"[calib] {tag}: size mismatch {gl.shape[::-1]} vs {size}")
        cl = calib_io.find_board(gl, pattern)
        cr = calib_io.find_board(gr, pattern)
        if cl is None or cr is None:
            miss = "left" if cl is None else "right"
            print(f"[calib] {tag}: FULL board not found in {miss} image — "
                  f"skipped (both cameras must see all corners)")
            continue
        views.append({"tag": tag, "l": cl, "r": cr})
    return views, size


def obj_points(pattern, square_m):
    cols, rows = pattern
    obj = np.zeros((cols * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return obj


def calibrate_camera(views, key, obj, size, tag):
    """Per-camera intrinsics with per-view errors.
    flags: k1,k2 free; k3 + tangential zero; pp free; aspect fixed."""
    imgs = [v[key].reshape(-1, 1, 2) for v in views]
    objs = [obj] * len(views)
    flags = (cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST
             | cv2.CALIB_FIX_K3 | cv2.CALIB_USE_INTRINSIC_GUESS)
    K0 = np.array([[1100.0, 0, size[0] / 2],
                   [0, 1100.0, size[1] / 2], [0, 0, 1]])
    rms, K, dist, rvecs, tvecs, _, _, per_view = cv2.calibrateCameraExtended(
        objs, imgs, size, K0, np.zeros(5), flags=flags)
    d = np.asarray(dist).ravel()
    print(f"[calib] {tag}: rms={rms:.3f}px  fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} "
          f"cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}  k1={d[0]:+.4f} k2={d[1]:+.4f}")
    return {"rms": rms, "K": K, "dist": dist, "per_view": per_view.ravel(),
            "rvecs": rvecs, "tvecs": tvecs}


def reject_worst(views, obj, size, max_rounds=2, min_views=12):
    """Per-view RMS gate: drop views whose max(L,R) error exceeds
    max(1.5·median, 1.0 px). Returns (views, calL, calR)."""
    for rnd in range(max_rounds + 1):
        cal_l = calibrate_camera(views, "l", obj, size, f"round{rnd} LEFT ")
        cal_r = calibrate_camera(views, "r", obj, size, f"round{rnd} RIGHT")
        err = np.maximum(cal_l["per_view"], cal_r["per_view"])
        thr = max(1.5 * float(np.median(err)), 1.0)
        print(f"[calib] per-view max(L,R) rms px:")
        for v, e in zip(views, err):
            flag = "  <-- REJECT" if e > thr and rnd < max_rounds else ""
            print(f"    {v['tag']:<10} {e:6.3f}{flag}")
        bad = err > thr
        if rnd == max_rounds or not bad.any():
            return views, cal_l, cal_r
        if len(views) - int(bad.sum()) < min_views:
            print(f"[calib] rejection would leave <{min_views} views — "
                  f"keeping all (RECAPTURE the flagged poses)")
            return views, cal_l, cal_r
        views = [v for v, b in zip(views, bad) if not b]
        print(f"[calib] round {rnd}: rejected {int(bad.sum())} view(s), "
              f"{len(views)} remain")


def coverage_heatmap(views, size, out_path):
    """Corner coverage per camera — thin coverage is where focal/distortion
    are unconstrained; no image quadrant may be empty."""
    h, w = 72, 128
    img = np.zeros((h, 2 * w), np.float32)
    for v in views:
        for key, xoff in (("l", 0), ("r", w)):
            c = v[key]
            xi = np.clip((c[:, 0] * w / size[0]).astype(int), 0, w - 1) + xoff
            yi = np.clip((c[:, 1] * h / size[1]).astype(int), 0, h - 1)
            img[yi, xi] += 1
    img = cv2.resize(img, (2 * w * 4, h * 4), interpolation=cv2.INTER_NEAREST)
    img = (255 * img / max(img.max(), 1e-9)).astype(np.uint8)
    vis = cv2.applyColorMap(img, cv2.COLORMAP_TURBO)
    vis = cv2.line(vis, (w * 4, 0), (w * 4, h * 4), (255, 255, 255), 2)
    cv2.putText(vis, "LEFT", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2)
    cv2.putText(vis, "RIGHT", (w * 4 + 8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (255, 255, 255), 2)
    cv2.imwrite(str(out_path), vis)
    print(f"[calib] coverage heatmap -> {out_path}")


def calibrate_printed(args):
    c, r = args.pattern.lower().split("x")
    pattern = (int(c), int(r))
    square_m = args.square_mm / 1000.0
    views, size = detect_all(args.session, pattern)
    print(f"[calib] {len(views)} usable pairs, frame {size[0]}x{size[1]}, "
          f"square {args.square_mm:g}mm")
    if len(views) < 8:
        sys.exit(f"[calib] only {len(views)} full-board pairs — need >=12 "
                 f"(20-25 recommended: 0.4-2m spread, +-30deg skews, corner "
                 f"coverage)")
    out = Path(args.out) if args.out else \
        _CALIB_DIR / f"stereo_rectify_{date.today().strftime('%Y%m%d')}.yml"
    run_calibration(views, size, pattern, square_m, args.pin_baseline, out)


def run_calibration(views, size, pattern, square_m, pin_baseline, out):
    """Core printed-board calibration on detected corner views (testable with
    synthetic corners). Writes the versioned yml + coverage heatmap."""
    obj = obj_points(pattern, square_m)

    views, cal_l, cal_r = reject_worst(views, obj, size)

    imgs_l = [v["l"].reshape(-1, 1, 2) for v in views]
    imgs_r = [v["r"].reshape(-1, 1, 2) for v in views]
    objs = [obj] * len(views)
    rms, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
        objs, imgs_l, imgs_r, cal_l["K"], cal_l["dist"],
        cal_r["K"], cal_r["dist"], size, flags=cv2.CALIB_FIX_INTRINSIC)
    B = float(np.linalg.norm(T))
    rvec, _ = cv2.Rodrigues(R)
    rd = np.degrees(rvec.ravel())
    print(f"\n[calib] stereo rms={rms:.3f}px  (pass target <0.5)")
    print(f"[calib] |T|={B * 100:.2f}cm from printed square "
          f"(hand-measured ref {_HAND_BASELINE_M * 100:.1f}cm, "
          f"ratio {B / _HAND_BASELINE_M:.4f})")
    print(f"[calib] R: tilt={rd[0]:+.2f} yaw={rd[1]:+.2f} roll={rd[2]:+.2f} deg")
    if pin_baseline:
        T = T * (pin_baseline / B)
        B = pin_baseline
        print(f"[calib] --pin-baseline: T rescaled to |T|={B * 100:.2f}cm")

    # board distance sanity per view
    for v, o in zip(views[:6], objs):
        ok, rv, tv = cv2.solvePnP(o, v["l"].reshape(-1, 1, 2),
                                  cal_l["K"], cal_l["dist"])
        print(f"[calib]   {v['tag']}: board Z={tv.ravel()[2]:.2f}m")

    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        cal_l["K"], cal_l["dist"], cal_r["K"], cal_r["dist"],
        size, R, T, alpha=-1)
    fb = abs(float(P2[0, 3]))
    print(f"[calib] rectified fx={P2[0, 0]:.1f}  f*B={fb:.3f} px*m  "
          f"(Z = {fb:.2f}/disp; sanity band ~5% of 60.7)")

    if out.name == "stereo_rectify.yml":
        sys.exit("[calib] refusing to write the ACTIVE yml directly — write "
                 "the versioned file, run epipolar_check.py, then copy over "
                 "manually (archive the old one first)")
    out.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(out), cv2.FILE_STORAGE_WRITE)
    fs.write("date", date.today().isoformat())
    fs.write("image_width", size[0])
    fs.write("image_height", size[1])
    fs.write("K_left", cal_l["K"])
    fs.write("K_right", cal_r["K"])
    fs.write("dist_left", cal_l["dist"])
    fs.write("dist_right", cal_r["dist"])
    fs.write("R", R)
    fs.write("T", T)
    fs.write("R1", R1)
    fs.write("R2", R2)
    fs.write("P1", P1)
    fs.write("P2", P2)
    fs.write("Q", Q)
    fs.write("baseline_m", B)
    fs.write("square_m", square_m)
    fs.write("fb", fb)
    fs.write("stereo_rms_px", float(rms))
    fs.write("mono_rms_l_px", float(cal_l["rms"]))
    fs.write("mono_rms_r_px", float(cal_r["rms"]))
    fs.write("n_views", len(views))
    fs.write("angles_deg_tilt_yaw_roll", np.array(rd))
    fs.release()
    print(f"[calib] saved -> {out}")
    coverage_heatmap(views, size,
                     out.parent / f"coverage_{out.stem.split('_')[-1]}.jpg")
    print(f"\nNEXT: python epipolar_check.py --session <held-out pairs> "
          f"--rectify {out}\n      pass = |dy| p90 < 0.5px (board mode); then "
          f"archive the old yml and copy this one over stereo_rectify.yml")


# ─────────────────────────────────────────────────────── legacy screen path ──
def joint_focal(grids, size):
    """Single fx pooled over all views (both cameras: alignment scale ~1)."""
    objs, imgs = [], []
    for grid in grids:
        rows, cols = grid.shape[:2]
        obj = np.zeros((rows * cols, 3), np.float32)
        obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        objs.append(obj)
        imgs.append(grid.reshape(-1, 1, 2).astype(np.float32))
    flags = (cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST
             | cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3
             | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_USE_INTRINSIC_GUESS)
    K0 = np.array([[1000.0, 0, size[0] / 2],
                   [0, 1000.0, size[1] / 2], [0, 0, 1]])
    rms, K, *_ = cv2.calibrateCamera(objs, imgs, size, K0, np.zeros(5),
                                     flags=flags)
    return K[0, 0], rms


def calibrate_legacy(args):
    import align_from_chessboard as afc
    cap_dir = Path(args.captures)
    pairs = afc._find_pairs(cap_dir)
    if not pairs:
        sys.exit(f"[calib] no chessboard_left*/right* pairs in {cap_dir}")

    grids_all = []
    obj_l, img_l, img_r = [], [], []
    size = None
    for lp, rp in pairs:
        left = cv2.imread(str(lp))
        right = cv2.imread(str(rp))
        gl = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        size = gl.shape[::-1]
        grid_l = afc._detect_grid(gl)
        grid_r = afc._detect_grid(gr)
        if grid_l is None or grid_r is None:
            print(f"[calib] {lp.name}: board not detected, skipped")
            continue
        grids_all += [grid_l, grid_r]
        res = afc._align_pair(left, right, afc._orb_prior(left, right))
        if res is None:
            print(f"[calib] {lp.name}: correspondence failed, skipped")
            continue
        di, dj, flip = res["grid_shift"]
        g_r = grid_r[::-1, ::-1] if flip else grid_r
        rl, cl = grid_l.shape[:2]
        rr, cr = g_r.shape[:2]
        i0, i1 = max(0, -di), min(rl, rr - di)
        j0, j1 = max(0, -dj), min(cl, cr - dj)
        pts_l = grid_l[i0:i1, j0:j1].reshape(-1, 2)
        pts_r = g_r[i0 + di:i1 + di, j0 + dj:j1 + dj].reshape(-1, 2)
        ii, jj = np.mgrid[i0:i1, j0:j1]
        obj = np.stack([jj.ravel(), ii.ravel(), np.zeros(ii.size)], 1)
        obj_l.append(obj.astype(np.float32))
        img_l.append(pts_l.reshape(-1, 1, 2).astype(np.float32))
        img_r.append(pts_r.reshape(-1, 1, 2).astype(np.float32))
        print(f"[calib] {lp.name}: {len(obj)} common corners "
              f"(shift {di},{dj} flip={flip})")
    if not obj_l:
        sys.exit("[calib] no usable pair")

    fx, rms_mono = joint_focal(grids_all, size)
    print(f"[calib] joint focal fx={fx:.1f}px (mono rms {rms_mono:.2f}px, "
          f"HFOV {2 * np.degrees(np.arctan(size[0] / 2 / fx)):.1f}deg)")
    K = np.array([[fx, 0, size[0] / 2], [0, fx, size[1] / 2], [0, 0, 1]])
    dist = np.zeros(5)

    rms, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
        obj_l, img_l, img_r, K, dist, K.copy(), dist, size,
        flags=cv2.CALIB_FIX_INTRINSIC)
    sq_m = args.baseline / float(np.linalg.norm(T))
    T_m = T * sq_m
    rvec, _ = cv2.Rodrigues(R)
    rd = np.degrees(rvec.ravel())
    print(f"[calib] stereo rms={rms:.2f}px  square={sq_m * 100:.2f}cm")
    print(f"[calib] R: tilt={rd[0]:+.2f}deg yaw={rd[1]:+.2f}deg "
          f"roll={rd[2]:+.2f}deg")
    print(f"[calib] T={T_m.ravel() * 100} cm (|T|=baseline="
          f"{args.baseline * 100}cm)")
    for k, o in enumerate(obj_l):
        ok, rv, tv = cv2.solvePnP(o * sq_m, img_l[k], K, dist)
        print(f"[calib] pair{k + 1}: board distance Z={tv.ravel()[2]:.2f}m")

    # alpha=-1 (auto): alpha=0 degenerates on this tilted-baseline rig
    # (valid-ROI crop explodes the rectified focal to ~166000px)
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K, dist, K, dist, size, R, T_m, alpha=-1)
    fb = abs(float(P2[0, 3]))
    print(f"[calib] rectified f*B = {fb:.3f} px*m  "
          f"(Z = {fb:.2f}/disparity_px)")

    out = Path(args.out) if args.out else _CALIB_DIR / "stereo_rectify.yml"
    out.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(out), cv2.FILE_STORAGE_WRITE)
    fs.write("date", date.today().isoformat())
    fs.write("image_width", size[0])
    fs.write("image_height", size[1])
    fs.write("K", K)
    fs.write("dist", dist)
    fs.write("R", R)
    fs.write("T", T_m)
    fs.write("R1", R1)
    fs.write("R2", R2)
    fs.write("P1", P1)
    fs.write("P2", P2)
    fs.write("Q", Q)
    fs.write("fx", float(fx))
    fs.write("baseline_m", args.baseline)
    fs.write("square_m", sq_m)
    fs.write("fb", fb)
    fs.write("stereo_rms_px", float(rms))
    fs.write("angles_deg_tilt_yaw_roll", np.array(rd))
    fs.release()
    print(f"[calib] saved -> {out}")


# ─────────────────────────────────────────────────────────────────── main ────
def main():
    p = argparse.ArgumentParser(
        description="Metric stereo calibration (printed board / legacy screen)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--session", default=str(_DEF_CAPTURES),
                   help="capture session dir (manifest.csv or *_left/right)")
    p.add_argument("--pattern", default="9x6", help="inner corners CxR")
    p.add_argument("--square-mm", type=float, default=25.0,
                   help="RULER-MEASURED printed square size (mm)")
    p.add_argument("--pin-baseline", type=float, default=None, metavar="M",
                   help="rescale |T| to this hand-measured baseline (m); "
                        "default trusts the printed square scale")
    p.add_argument("--out", default=None,
                   help="output yml (default calib/stereo_rectify_YYYYMMDD.yml)")
    p.add_argument("--legacy-screen", action="store_true",
                   help="original 2026-07-02 flow: partial laptop-screen "
                        "board, pooled focal, baseline-pinned scale")
    p.add_argument("--captures", default=str(_DEF_CAPTURES),
                   help="(legacy) dir with chessboard_left*/right*")
    p.add_argument("--baseline", type=float, default=_HAND_BASELINE_M,
                   help="(legacy) hand-measured lens-to-lens distance (m)")
    args = p.parse_args()
    if args.legacy_screen:
        calibrate_legacy(args)
    else:
        calibrate_printed(args)


if __name__ == "__main__":
    main()
