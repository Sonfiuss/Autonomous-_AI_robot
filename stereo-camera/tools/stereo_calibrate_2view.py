#!/usr/bin/env python3
"""
stereo_calibrate_2view.py — metric stereo calibration from chessboard pairs,
scale anchored by the hand-measured baseline (lens-to-lens distance).

Works with the same partially-visible laptop-screen chessboard pairs as
align_from_chessboard.py (reuses its detection + L/R grid correspondence).
The physical square size is unknown, so stereoCalibrate returns T in square
units; the measured baseline |T| = B pins the metric scale.

Outputs calib/stereo_rectify.yml with K, R, T (metres), R1/R2/P1/P2/Q from
stereoRectify — epipolar lines become horizontal even though this rig is
toed-in (~2°) with a tilted baseline (right cam sits right AND down).

Usage:
  python3 stereo_calibrate_2view.py --baseline 0.054
"""
import argparse
import sys
from datetime import date
from pathlib import Path

import numpy as np
import cv2

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
import align_from_chessboard as afc

_DEF_CAPTURES = _HERE / "captures"
_DEF_OUT = _HERE.parent / "calib" / "stereo_rectify.yml"


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


def main():
    p = argparse.ArgumentParser(
        description="Metric stereo calibration anchored by measured baseline.")
    p.add_argument("--captures", default=str(_DEF_CAPTURES))
    p.add_argument("--baseline", type=float, default=0.054,
                   help="lens-to-lens distance in metres (measured by hand)")
    p.add_argument("--out", default=str(_DEF_OUT))
    args = p.parse_args()

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
        # resolve which physical corners both cameras see (integer grid shift)
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
          f"HFOV {2*np.degrees(np.arctan(size[0]/2/fx)):.1f}deg)")
    K = np.array([[fx, 0, size[0] / 2], [0, fx, size[1] / 2], [0, 0, 1]])
    dist = np.zeros(5)

    rms, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
        obj_l, img_l, img_r, K, dist, K.copy(), dist, size,
        flags=cv2.CALIB_FIX_INTRINSIC)
    sq_m = args.baseline / float(np.linalg.norm(T))
    T_m = T * sq_m
    rvec, _ = cv2.Rodrigues(R)
    rd = np.degrees(rvec.ravel())
    print(f"[calib] stereo rms={rms:.2f}px  square={sq_m*100:.2f}cm")
    print(f"[calib] R: tilt={rd[0]:+.2f}deg yaw={rd[1]:+.2f}deg "
          f"roll={rd[2]:+.2f}deg")
    print(f"[calib] T={T_m.ravel()*100} cm (|T|=baseline={args.baseline*100}cm)")
    for k, o in enumerate(obj_l):
        ok, rv, tv = cv2.solvePnP(o * sq_m, img_l[k], K, dist)
        print(f"[calib] pair{k+1}: board distance Z={tv.ravel()[2]:.2f}m")

    # alpha=-1 (auto): alpha=0 degenerates on this tilted-baseline rig
    # (valid-ROI crop explodes the rectified focal to ~166000px)
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K, dist, K, dist, size, R, T_m, alpha=-1)
    fb = abs(float(P2[0, 3]))          # fx_rect * baseline (px*m)
    print(f"[calib] rectified f*B = {fb:.3f} px*m  "
          f"(Z = {fb:.2f}/disparity_px)")

    out = Path(args.out)
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


if __name__ == "__main__":
    main()
