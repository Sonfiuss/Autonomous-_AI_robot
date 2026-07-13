#!/usr/bin/env python3
"""
replay_shots — offline: saved stereo pairs → common-scale metric clouds → map.

Task 2026-07-12_scale-merge-deepmap: the three shot_N pairs are a mini walk
session (robot creeping forward). Each frame's DA-V2 relative depth is scaled
to metres by ITS OWN stereo measurements (fuse_metric), which puts every frame
on the same absolute scale — the merge then only needs a pose per stop, not a
scale transfer between frames.

Runs on Windows (CPU torch) with the CURRENT imperfect calib: rectification
still leaves |dy| p50 ≈ 7.5 px (measured 2026-07-12), so
  * golden_points gets a WIDE dy_search (default ±12 rows, vs ±1 stock) — the
    NCC uniqueness + LR check keep false matches out, runtime is the cost;
  * when the golden fit still fails or is inlier-starved, sparse SGBM
    disparities stand in as anchors (same {x,y,xr,disp} contract).
Re-run unchanged after the chessboard recalibration — only --rectify changes.

  python replay_shots.py                      # 3 pairs from stereo-camera/captures
  python replay_shots.py --step-cm 5          # dead-reckon prior between stops

Outputs (never overwrites an existing session dir's PLYs from another day):
  <out-dir>/stop_N.ply        leveled, world-placed per-stop cloud (metres)
  <out-dir>/depth_N_m.npy     metric depth per stop
  <out-dir>/merged_walk.ply   all stops merged (ICP when open3d exists, else
                              dead-reckon/VO placement + voxel dedup)
  <out-dir>/report.csv        per-stop scale fit + VO consistency numbers
"""
import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, '..', 'depth-anything', 'src')))
sys.path.insert(0, ROOT)

import stereo_ruler as sr                              # noqa: E402
from merge_360 import write_ply                        # noqa: E402
from explore_map import Pose, voxel_dedup, icp_merge_clouds  # noqa: E402
from stereo_walk_map import (                          # noqa: E402
    backproject, level_to_floor, to_world, _pose_from_xzyaw, _pose_snapshot,
)
import visual_odom as vo                               # noqa: E402

_DEF_SESSION = os.path.abspath(os.path.join(
    ROOT, '..', 'stereo-camera', 'captures'))
_DEF_RECTIFY = os.path.abspath(os.path.join(
    ROOT, '..', 'stereo-camera', 'calib', 'stereo_rectify.yml'))
_DEF_OUT = os.path.abspath(os.path.join(
    ROOT, '..', 'depth-anything', 'output', 'pointcloud',
    time.strftime('session_%Y%m%d')))


def _dy_tuple(radius):
    """(0, -1, 1, ..., -r, r) — nearest rows first so NCC ties keep small dy."""
    out = [0]
    for d in range(1, radius + 1):
        out += [-d, d]
    return tuple(out)


def sgbm_anchor_points(gray_l, gray_r, num_disp=160, block=5, grid_px=24,
                       max_pts=500):
    """Sparse SGBM disparities in the golden_points {x,y,xr,disp} contract.

    Fallback anchor source when the golden fit fails: SGBM's own
    disp12MaxDiff LR check plays the role of golden's back-match, and the
    grid subsample keeps the RANSAC input small and spatially spread.
    """
    sgbm = cv2.StereoSGBM_create(
        minDisparity=0, numDisparities=num_disp, blockSize=block,
        P1=8 * block * block, P2=32 * block * block,
        uniquenessRatio=10, speckleWindowSize=100, speckleRange=2,
        disp12MaxDiff=1)
    disp = sgbm.compute(gray_l, gray_r).astype(np.float32) / 16.0
    h, w = disp.shape
    pts = []
    for y in range(block, h - block, grid_px):
        for x in range(num_disp + block, w - block, grid_px):
            d = float(disp[y, x])
            if d <= 1.0:
                continue
            pts.append({"x": float(x), "y": float(y),
                        "xr": float(x - d), "disp": d, "ncc": 1.0})
    if len(pts) > max_pts:                      # even thinning, keep spread
        pts = pts[::len(pts) // max_pts + 1]
    return pts


def fuse_stop(left, right, calib, model, args):
    """One pair → (Z, ok, info, right_rect); info['src'] says which anchor
    source won. Raises RuntimeError when neither source fuses."""
    left_r = cv2.remap(left, calib.map_l[0], calib.map_l[1], cv2.INTER_LINEAR)
    right_r = cv2.remap(right, calib.map_r[0], calib.map_r[1], cv2.INTER_LINEAR)
    gray_l = cv2.cvtColor(left_r, cv2.COLOR_BGR2GRAY)
    gray_r = cv2.cvtColor(right_r, cv2.COLOR_BGR2GRAY)
    golden = sr.golden_points(gray_l, gray_r, d_max=args.d_max,
                              dy_search=_dy_tuple(args.dy_search))
    mono, valid = sr.depth_half(right, calib, model, args.input_size)
    cam_h = args.cam_height_m or None

    result, err = None, None
    try:
        Z, ok, info = sr.fuse_metric(golden, mono, valid, calib,
                                     cam_h=cam_h, tilt_deg=args.tilt_deg)
        info['src'] = 'golden'
        result = (Z, ok, info)
    except RuntimeError as e:
        err = str(e)
    # inlier-starved golden fit → an SGBM-anchored fit is the better ruler
    if result is None or result[2]['n_inliers'] < args.min_inliers:
        anchors = sgbm_anchor_points(gray_l, gray_r, num_disp=args.d_max + 16)
        try:
            Z, ok, info = sr.fuse_metric(anchors, mono, valid, calib,
                                         cam_h=cam_h, tilt_deg=args.tilt_deg)
            info['src'] = 'sgbm'
            info['n_golden_orig'] = len(golden)
            if result is None or info['n_inliers'] > result[2]['n_inliers']:
                result = (Z, ok, info)
        except RuntimeError as e2:
            if result is None:
                raise RuntimeError(f'golden: {err} | sgbm: {e2}') from None
    Z, ok, info = result
    return Z, ok, info, right_r


def main():
    p = argparse.ArgumentParser(
        description='Offline replay: saved stereo pairs → metric clouds → map',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('--session', default=_DEF_SESSION,
                   help='dir with shot_N_left.jpg / shot_N_right.jpg')
    p.add_argument('--rectify', default=_DEF_RECTIFY)
    p.add_argument('--out-dir', default=_DEF_OUT)
    p.add_argument('--encoder', default='vits', choices=['vits', 'vitb', 'vitl'])
    p.add_argument('--input-size', type=int, default=518)
    p.add_argument('--dy-search', type=int, default=12,
                   help='golden row search radius (px) — covers the measured '
                        'rectification residual, stock is 1')
    p.add_argument('--d-max', type=int, default=160,
                   help='max disparity (px); nearest scene point ~fb/d_max')
    p.add_argument('--min-inliers', type=int, default=8,
                   help='golden fit below this tries the SGBM anchor fallback')
    p.add_argument('--cam-height-m', type=float, default=0.0,
                   help='measured camera height (m); 0 = no floor anchors')
    p.add_argument('--tilt-deg', type=float, default=13.0)
    p.add_argument('--step-cm', type=float, default=0.0,
                   help='dead-reckon forward prior between stops (cm); 0 = '
                        'unknown → VO accepts only small motions (≤0.15 m)')
    p.add_argument('--stride', type=int, default=2)
    p.add_argument('--z-min', type=float, default=0.15)
    p.add_argument('--z-max', type=float, default=3.0)
    p.add_argument('--bottom-frac', type=float, default=0.30)
    p.add_argument('--voxel', type=float, default=0.01)
    p.add_argument('--no-vo', action='store_true')
    p.add_argument('--no-level', action='store_true')
    args = p.parse_args()

    # collect shot indices present in the session dir
    idxs = []
    i = 0
    while os.path.isfile(os.path.join(args.session, f'shot_{i}_left.jpg')):
        idxs.append(i)
        i += 1
    if not idxs:
        sys.exit(f'no shot_N_left.jpg pairs in {args.session}')

    probe = cv2.imread(os.path.join(args.session, 'shot_0_left.jpg'))
    calib = sr.load_rectify(args.rectify, (probe.shape[1], probe.shape[0]))
    model, device = sr.load_model(args.encoder)
    print(f'pairs={len(idxs)}  calib={os.path.basename(args.rectify)} '
          f'fb={calib.fb:.2f}  DA-V2 {args.encoder} on {device}  '
          f'dy_search=±{args.dy_search}px', flush=True)
    os.makedirs(args.out_dir, exist_ok=True)

    tracker = None if args.no_vo else vo.VOTracker(
        calib, z_min=args.z_min, z_max=args.z_max)
    prior = Pose()
    all_pts, all_cols, rows_csv = [], [], []
    for i in idxs:
        t0 = time.time()
        left = cv2.imread(os.path.join(args.session, f'shot_{i}_left.jpg'))
        right = cv2.imread(os.path.join(args.session, f'shot_{i}_right.jpg'))
        if i > 0:
            prior.advance(args.step_cm / 100.0)
        try:
            Z, ok, info, right_r = fuse_stop(left, right, calib, model, args)
        except RuntimeError as e:
            print(f'stop {i}: SKIPPED — {e}', flush=True)
            rows_csv.append([i, 'skip', '', '', '', '', '', '', '', str(e)])
            continue
        np.save(os.path.join(args.out_dir, f'depth_{i}_m.npy'), Z)
        pts, cols, rows = backproject(Z, ok, right_r, calib,
                                      args.stride, args.z_min, args.z_max)
        lvl, cam_h_est = None, ''
        if not args.no_level:
            pts, cam_h_val, lok, lvl = level_to_floor(
                pts, rows, Z.shape[0], args.bottom_frac,
                tilt_deg=info.get('tilt_deg', args.tilt_deg))
            cam_h_est = f'{cam_h_val:.3f}' if cam_h_val is not None else ''
        pose = _pose_snapshot(prior)
        note = ''
        if tracker is not None:
            xzyaw, note = tracker.update(right_r, Z, ok, lvl,
                                         (pose.x, pose.z, pose.yaw))
            pose = _pose_from_xzyaw(xzyaw)
        wpts = to_world(pts, pose)
        write_ply(os.path.join(args.out_dir, f'stop_{i}.ply'), wpts, cols)
        all_pts.append(wpts)
        all_cols.append(cols)

        zc = Z[ok]
        # centre-patch depth: the "distance to the object ahead" a user can
        # tape-measure to sanity-check the common scale
        h, w = Z.shape
        cp = Z[h // 2 - 40:h // 2 + 40, w // 2 - 40:w // 2 + 40]
        z_ctr = float(np.median(cp[cp > 0])) if (cp > 0).any() else float('nan')
        print(f'stop {i}: src={info["src"]}  golden={info["n_golden"]} '
              f'inl={info["n_inliers"]}  s={info["s"]:.5f} rms={info["rms"]:.4f} '
              f'span=x{info["span"]:.1f}  Z[{zc.min():.2f},{zc.max():.2f}]m '
              f'ctr={z_ctr:.2f}m  cam_h={cam_h_est or "-"}  pts={len(pts)} '
              f'[{time.time() - t0:.1f}s]', flush=True)
        if note:
            print(f'stop {i}: {note}', flush=True)
        rows_csv.append([i, info['src'], info['n_golden'], info['n_inliers'],
                         f'{info["s"]:.6f}', f'{info["rms"]:.4f}',
                         f'{info["span"]:.2f}', f'{z_ctr:.3f}', cam_h_est, note])

    if all_pts:
        pts, cols = icp_merge_clouds(all_pts, all_cols, args.voxel)
        pts, cols = voxel_dedup(pts, cols, args.voxel)
        out = os.path.join(args.out_dir, 'merged_walk.ply')
        write_ply(out, pts, cols)
        print(f'merged {len(all_pts)} stops → {out}  ({len(pts)} pts)',
              flush=True)
    with open(os.path.join(args.out_dir, 'report.csv'), 'w', newline='') as f:
        wcsv = csv.writer(f)
        wcsv.writerow(['stop', 'src', 'n_golden', 'n_inliers', 's', 'rms',
                       'span', 'z_center_m', 'cam_h_est_m', 'vo_note'])
        wcsv.writerows(rows_csv)
    print(f'report → {os.path.join(args.out_dir, "report.csv")}', flush=True)


if __name__ == '__main__':
    main()
