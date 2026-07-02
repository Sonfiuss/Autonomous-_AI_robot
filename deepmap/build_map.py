#!/usr/bin/env python3
"""
Feature 2 — Build a 360° depth map from rotate_scan.py output.

Loads each captured image, runs Depth Anything V2 to get per-pixel depth,
projects to 3D, rotates each cloud to its recorded world angle, and merges
all frames into one 360° point cloud saved as a PLY file.

Reuses the existing depth-anything/src pipeline (DRY — no code duplication).

Usage:
  python build_map.py                            # uses deepmap/shots/ by default
  python build_map.py --shots shots/ --show
  python build_map.py --shots shots/ --encoder vitb --voxel 0.02 --icp
"""

import argparse
import os
import sys

import cv2
import numpy as np
import torch
import yaml

# Reuse the depth-anything pipeline without copying any code
_DEPTH_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', 'depth-anything', 'src')
sys.path.insert(0, os.path.abspath(_DEPTH_SRC))

from depth_to_3d_timed import (          # depth pipeline primitives
    CFGS, pick_device, Timer,
    step_prep, step_infer, step_project,
)
from depth_anything_v2.dpt import DepthAnythingV2
from merge_360 import (                  # geometry + IO helpers
    list_images, load_angles,
    transform_to_world, write_ply,
    try_open3d, to_o3d, icp_merge,
)
import scale_calib                        # step A4: metric scale recovery
from drive_area import (                   # movable-space determination
    preprocess_depth, compute_drive_polygon,
)

ROOT = os.path.dirname(os.path.abspath(__file__))
_MODEL_DIR = os.path.join(_DEPTH_SRC, '..', 'model')


def parse_args(argv=None):
    p = argparse.ArgumentParser(description='Build 360° depth map from rotate_scan output')
    p.add_argument('--shots', default=os.path.join(ROOT, 'shots'),
                   help='Directory produced by rotate_scan.py (contains images + angles.csv)')
    p.add_argument('--manifest', default=None,
                   help='Override manifest path (default: <shots>/angles.csv)')
    p.add_argument('--cam-offset', type=float, default=0.0,
                   help='Distance of camera in front of rotation center (m)')
    # Depth model
    p.add_argument('--encoder', default='vits', choices=['vits', 'vitb'])
    p.add_argument('--input-size', type=int, default=518)
    p.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    # Projection
    p.add_argument('--fov', type=float, default=72.0,
                   help='Camera horizontal FOV in degrees (real lens ≈ 72)')
    p.add_argument('--keep-deg', type=float, default=None,
                   help='Central horizontal wedge kept per frame (deg) to remove '
                        'inter-frame overlap. None = auto (= scan step from angles). '
                        '0 or >=fov = keep full frame (no crop).')
    p.add_argument('--cam-yaw-deg', type=float, default=0.0,
                   help='Constant yaw offset (deg) added to every frame to compensate '
                        'a camera mounted/pointing left(+)/right(-) of robot forward.')
    p.add_argument('--stride', type=int, default=2,
                   help='Pixel stride for point cloud density (1=full, 2=half)')
    # Output
    p.add_argument('--out', default=os.path.join(ROOT, 'output', 'map_360.ply'))
    p.add_argument('--voxel', type=float, default=0.0,
                   help='Voxel downsample size in metres (0 = off, needs open3d)')
    p.add_argument('--icp', action='store_true',
                   help='Refine frame boundaries with ICP (needs open3d)')
    p.add_argument('--show', action='store_true',
                   help='Open interactive 3D viewer after building (needs open3d)')
    # Scale calibration (plan step A4)
    p.add_argument('--d-max', type=float, default=5.0,
                   help='Far-clip for metric depth in metres (plan D_max)')
    p.add_argument('--test-round360', action='store_true',
                   help='Test mode: SKIP the 30 cm forward scale-recovery move; '
                        'use a placeholder k, confirm metric-depth output TYPE, '
                        'and still build a small PLY from the fixed-image frames')
    # Movable-space classification method
    p.add_argument('--classify', choices=['drive-area', 'height'], default='drive-area',
                   help="How to label floor vs obstacle. 'drive-area' = old ray-linearity "
                        "polygon (drive_area.py). 'height' = height-above-ground: metric "
                        "back-project + per-frame floor-plane fit + physical-height threshold.")
    p.add_argument('--scale-yaml', default=os.path.join(
                       ROOT, '..', 'slam', 'config', 'scale.yaml'),
                   help='YAML with calibrated metric scale k (from scale_calib). Used by '
                        "--classify height so the height thresholds are in real metres.")
    p.add_argument('--h-floor', type=float, default=0.05,
                   help='|height above floor plane| <= this = floor/drivable (m). Default 0.05')
    p.add_argument('--h-obs', type=float, default=0.10,
                   help='height above floor plane > this = obstacle (m). Lower to catch '
                        'shorter objects. Default 0.10')
    p.add_argument('--h-max', type=float, default=2.0,
                   help='ignore points higher above the floor than this (ceiling / over '
                        'robot head) instead of marking them obstacle (m). Default 2.0')
    p.add_argument('--bottom-frac', type=float, default=0.30,
                   help='bottom fraction of each frame used as the floor seed for the '
                        'plane fit (--classify height). Default 0.30')
    p.add_argument('--bev-cell', type=float, default=0.05,
                   help='top-down BEV map cell size in metres. Default 0.05 (5 cm)')
    p.add_argument('--bev-range', type=float, default=3.0,
                   help='top-down BEV half-extent in metres around the robot. Default 3.0')
    p.add_argument('--bev-out', default=None,
                   help='BEV map PNG path (default: <out dir>/map_360_bev.png)')
    # Timing instrumentation
    p.add_argument('--no-drive-area', action='store_true',
                   help='Skip the movable-space step (and its timing + class colours)')
    p.add_argument('--no-class-color', action='store_true',
                   help='Keep raw RGB instead of colouring the map by class '
                        '(green=movable floor, red=obstacle)')
    p.add_argument('--timing-csv', default=None,
                   help='Per-frame timing output (default: <out dir>/timing_map.csv)')
    return p.parse_args(argv)


def load_model(encoder, device):
    model = DepthAnythingV2(**CFGS[encoder])
    ckpt = os.path.join(_MODEL_DIR, f'depth_anything_v2_{encoder}.pth')
    if not os.path.isfile(ckpt):
        raise FileNotFoundError(
            f'Model checkpoint not found: {ckpt}\n'
            f'Download from the depth-anything repo and place in {_MODEL_DIR}/')
    model.load_state_dict(torch.load(ckpt, map_location='cpu'))
    return model.to(device).eval()


def auto_step_deg(angles, files):
    """Median angular spacing between consecutive frames (deg) — the natural
    central-wedge width so frames tile without overlap."""
    degs = sorted(float(np.rad2deg(angles[f])) % 360.0 for f in files)
    diffs = np.diff(degs)
    diffs = diffs[diffs > 1e-3]
    return float(np.median(diffs)) if len(diffs) else 360.0


def crop_central_wedge(pts, cols, keep_deg, cls=None):
    """Keep only points whose horizontal bearing is within ±keep_deg/2 of the
    optical axis. With a 72° lens and 30° steps, adjacent frames overlap ~58%;
    cropping to the central `keep_deg` makes them tile edge-to-edge (no double
    geometry to merge). Bearing = atan2(X, -Z) (X=right, -Z=forward).

    `cls` (per-point class array) is cropped with the same mask when given."""
    if keep_deg is None or keep_deg <= 0 or keep_deg >= 180:
        return pts, cols, cls
    bearing = np.degrees(np.arctan2(pts[:, 0], -pts[:, 2]))
    m = np.abs(bearing) <= keep_deg / 2.0
    return pts[m], cols[m], (cls[m] if cls is not None else None)


# ── Height-above-ground classification (--classify height) ────────────────────

def load_scale_k(path, d_max_default):
    """Read calibrated metric scale k (and d_max) from scale_calib's YAML.
    Returns (k, d_max). Falls back to (1.0, d_max_default) with a warning if the
    file is missing — geometry is then correct only up to a global scale, so the
    height thresholds are in relative units rather than true metres."""
    if path and os.path.isfile(path):
        with open(path) as fh:
            y = yaml.safe_load(fh) or {}
        k = float(y.get('k', 1.0))
        d_max = float(y.get('d_max', d_max_default))
        print(f'  [height] metric scale k={k:.4f} d_max={d_max} (from {path})')
        return k, d_max
    print(f'  [height] WARNING: {path} not found -> k=1.0 (RELATIVE units; '
          'height thresholds are NOT real metres). Run scale_calib to fix.')
    return 1.0, d_max_default


def fit_floor_plane_svd(floor_pts, max_samples=2000):
    """SVD plane fit. Returns (normal pointing +Y, centroid) or (None, None)."""
    if len(floor_pts) < 50:
        return None, None
    if len(floor_pts) > max_samples:
        floor_pts = floor_pts[np.random.choice(len(floor_pts), max_samples, replace=False)]
    centroid = floor_pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(floor_pts - centroid, full_matrices=False)
    normal = Vt[-1]
    if normal[1] < 0:           # force +Y (up in camera frame) so height>0 = above floor
        normal = -normal
    return normal, centroid


def classify_height(pts, keep, depth_shape, stride, bottom_frac, h_floor, h_obs,
                    h_max):
    """Label each KEPT point by height above the floor plane.

    Floor seed = bottom `bottom_frac` rows of the frame (robust to camera pitch:
    we fit whatever plane those points lie on, then measure signed distance to it,
    re-fitting once on inliers to shed any obstacle that crept into the seed band).
    Obstacle only within (h_obs, h_max] — higher points (ceiling, things over the
    robot's head) stay unknown, same semantics as obstacle_grid.classify.
    Returns uint8 array (len = #kept points): 0=unknown, 1=floor, 2=obstacle."""
    H, W = depth_shape
    ys = np.arange(0, H, stride)
    xs = np.arange(0, W, stride)
    H_s, W_s = len(ys), len(xs)
    row_idx = (np.arange(H_s * W_s) // W_s)
    floor_seed_full = row_idx >= (H_s - max(1, int(H_s * bottom_frac)))
    floor_seed = floor_seed_full[keep] if keep is not None else floor_seed_full

    normal, centroid = fit_floor_plane_svd(pts[floor_seed])
    if normal is None:
        return np.zeros(len(pts), dtype=np.uint8)
    height = (pts - centroid) @ normal
    inlier = floor_seed & (np.abs(height) < h_floor)
    if inlier.sum() >= 50:
        n2, c2 = fit_floor_plane_svd(pts[inlier])
        if n2 is not None:
            normal, centroid = n2, c2
            height = (pts - centroid) @ normal

    cls = np.zeros(len(pts), dtype=np.uint8)
    cls[np.abs(height) <= h_floor] = 1
    cls[(height > h_obs) & (height <= h_max)] = 2
    return cls


CLS_COLOR = {1: (60, 200, 60), 2: (210, 50, 50)}   # RGB: floor green, obstacle red


def color_by_cls(cols, cls, alpha=0.55):
    """Blend point colours toward green (floor) / red (obstacle); leave unknown raw."""
    out = cols.astype(np.float32)
    for v, c in CLS_COLOR.items():
        m = cls == v
        out[m] = out[m] * (1.0 - alpha) + np.asarray(c, np.float32) * alpha
    return out.astype(np.uint8)


def render_bev_map(all_pts, all_cls, cell, rng, robot_px=4):
    """Top-down occupancy image from the merged world cloud (X right, Z forward).
    Obstacle cells override floor cells. Robot at centre. Returns BGR image."""
    pts = np.concatenate(all_pts, axis=0)
    cls = np.concatenate(all_cls, axis=0)
    n = max(1, int(round(2 * rng / cell)))
    gx = np.floor((pts[:, 0] + rng) / cell).astype(int)
    gz = np.floor((pts[:, 2] + rng) / cell).astype(int)
    m = (gx >= 0) & (gx < n) & (gz >= 0) & (gz < n)
    gx, gz, c = gx[m], gz[m], cls[m]

    grid = np.zeros((n, n), dtype=np.uint8)            # 0 unknown
    grid[gz[c == 1], gx[c == 1]] = 1                   # floor
    grid[gz[c == 2], gx[c == 2]] = 2                   # obstacle (overrides)

    img = np.full((n, n, 3), 60, dtype=np.uint8)       # unknown = grey
    img[grid == 1] = (60, 200, 60)                     # floor green (BGR-ish; ok)
    img[grid == 2] = (50, 50, 210)                     # obstacle red
    img = np.ascontiguousarray(np.flipud(img))         # forward (+Z) at top
    cv2.circle(img, (n // 2, n // 2), robot_px, (0, 255, 255), -1)  # robot
    return img


def main(argv=None):
    args = parse_args(argv)
    manifest = args.manifest or os.path.join(args.shots, 'angles.csv')

    # Discover images and their angles
    files = list_images(args.shots)
    if not files:
        print(f'No images found in {args.shots}')
        return

    angles = load_angles(files, manifest, step_deg=None, ccw=False)
    files = [f for f in files if f in angles]
    if not files:
        print('No images matched manifest angles — check angles.csv')
        return

    # Overlap-merge + camera-yaw compensation (plan: deepmap improvements)
    keep_deg = args.keep_deg if args.keep_deg is not None else auto_step_deg(angles, files)
    cam_yaw = np.deg2rad(args.cam_yaw_deg)
    wedge_on = bool(keep_deg) and 0 < keep_deg < args.fov

    print(f'360° map: {len(files)} frames, encoder={args.encoder}, device={args.device}')
    print(f'  fov={args.fov}°  wedge-crop={"%.1f°" % keep_deg if wedge_on else "off"}'
          f'  cam-yaw={args.cam_yaw_deg:+.1f}°')

    device = pick_device(args.device)

    with Timer(device) as t0:
        model = load_model(args.encoder, device)
    print(f'[load model] {t0.ms:.1f} ms')

    use_o3d = args.icp or args.voxel > 0
    o3d = try_open3d() if use_o3d else None
    if use_o3d and o3d is None:
        print('WARNING: open3d not installed — --icp/--voxel disabled. pip install open3d')

    all_pts, all_cols = [], []
    all_cls = []     # per-point class (height mode) for the top-down BEV map
    o3d_clouds = []
    k_scale = None   # metric scale; recovered/confirmed once in test mode
    d_max = args.d_max
    timing = []      # per-frame stage latencies (ms)

    height_mode = args.classify == 'height'
    if height_mode:
        k_scale, d_max = load_scale_k(args.scale_yaml, args.d_max)

    for k, f in enumerate(files):
        img = cv2.imread(f)
        if img is None:
            print(f'  skip (read error): {f}')
            continue

        with Timer(device) as tp: tensor, hw = step_prep(model, img, args.input_size)
        with Timer(device) as ti: depth      = step_infer(model, tensor, hw)

        with Timer(device) as tpr:
            if height_mode or args.test_round360:
                # Metric back-project (Z = k/disp) so the floor reconstructs flat —
                # required for height-above-ground. height mode uses the calibrated k
                # (loaded above); test mode confirms output type once with placeholder k.
                if args.test_round360 and not height_mode and k_scale is None:
                    k_scale, metric = scale_calib.calibrate(
                        depth, d_max, test_round360=True)
                else:
                    metric = scale_calib.apply_scale(depth, k_scale, d_max)
                pts, cols, keep = scale_calib.project_metric(
                    metric, img, args.fov, args.stride, return_keep=True)
            else:
                pts, cols = step_project(depth, img, args.fov, args.stride)
                keep = None

        # Per-point floor/obstacle classification + map colours.
        ms_ms = 0.0
        cls = None
        if height_mode:
            with Timer(device) as tms:
                cls = classify_height(pts, keep, depth.shape, args.stride,
                                      args.bottom_frac, args.h_floor, args.h_obs,
                                      args.h_max)
            ms_ms = tms.ms
            if not args.no_class_color:
                cols = color_by_cls(cols, cls)
        elif not args.no_drive_area:
            # Old ray-linearity drive area -> per-point drivable(floor)/obstacle.
            with Timer(device) as tms:
                dnorm = preprocess_depth(depth)
                polygon, _ = compute_drive_polygon(dnorm)
                drivable = scale_calib.drivable_labels(
                    depth.shape, polygon, args.stride, keep)
            ms_ms = tms.ms
            if not args.no_class_color:
                cols = scale_calib.colorize_by_class(cols, drivable)

        # Crop to the central wedge so overlapping frames tile instead of doubling up.
        if wedge_on:
            pts, cols, cls = crop_central_wedge(pts, cols, keep_deg, cls)

        with Timer(device) as tm:
            # cam_yaw compensates a camera not aligned with robot forward.
            wpts = transform_to_world(pts, angles[f] + cam_yaw, args.cam_offset)
            if o3d is not None:
                o3d_clouds.append(to_o3d(o3d, wpts, cols))
                if height_mode:
                    # BEV needs per-point classes, which the o3d merge cannot
                    # carry — keep the classified raw points alongside.
                    all_pts.append(wpts)
                    all_cls.append(cls)
            else:
                all_pts.append(wpts)
                all_cols.append(cols)
                if cls is not None:
                    all_cls.append(cls)

        timing.append({
            'step': k, 'file': os.path.basename(f),
            'prep_ms': round(tp.ms, 1), 'infer_ms': round(ti.ms, 1),
            'project_ms': round(tpr.ms, 1), 'merge_ms': round(tm.ms, 1),
            'movable_space_ms': round(ms_ms, 1), 'points': len(wpts),
        })

        print(f'  [{k+1}/{len(files)}] {os.path.basename(f):<16} '
              f'yaw={np.rad2deg(angles[f]):6.1f}°  {len(wpts)} pts | '
              f'prep {tp.ms:.0f} infer {ti.ms:.0f} project {tpr.ms:.0f} '
              f'merge {tm.ms:.0f} drive {ms_ms:.0f} ms')

    if not all_pts and not o3d_clouds:
        print('No frames processed — nothing to save.')
        return

    # Merge all clouds
    if o3d is not None:
        if args.icp and len(o3d_clouds) > 1:
            print('[merge] ICP alignment...')
            merged = icp_merge(o3d, o3d_clouds, args.voxel)
        else:
            merged = o3d_clouds[0]
            for pc in o3d_clouds[1:]:
                merged += pc
        if args.voxel > 0:
            merged = merged.voxel_down_sample(args.voxel)
        mpts  = np.asarray(merged.points, dtype=np.float32)
        mcols = (np.asarray(merged.colors) * 255.0).astype(np.uint8)
    else:
        mpts  = np.concatenate(all_pts,  axis=0)
        mcols = np.concatenate(all_cols, axis=0)

    with Timer(device) as tw:
        write_ply(args.out, mpts, mcols)
    print(f'\n360° map: {len(mpts):,} points → {args.out}')

    # Top-down BEV occupancy map (height mode only — needs per-point classes).
    if height_mode and all_cls:
        bev = render_bev_map(all_pts, all_cls, args.bev_cell, args.bev_range)
        bev_out = args.bev_out or os.path.join(os.path.dirname(args.out), 'map_360_bev.png')
        os.makedirs(os.path.dirname(bev_out), exist_ok=True)
        # upscale for visibility
        scale = max(1, int(round(600 / bev.shape[0])))
        cv2.imwrite(bev_out, cv2.resize(bev, (0, 0), fx=scale, fy=scale,
                                        interpolation=cv2.INTER_NEAREST))
        cat = np.concatenate(all_cls)
        print(f'BEV map ({args.bev_cell*100:.0f} cm cells, ±{args.bev_range} m): '
              f'{int((cat==1).sum())} floor / {int((cat==2).sum())} obstacle pts '
              f'→ {bev_out}')

    # ── Timing report + CSV ───────────────────────────────────────────────────
    if timing:
        keys = ['prep_ms', 'infer_ms', 'project_ms', 'merge_ms', 'movable_space_ms']
        avg = {key: sum(r[key] for r in timing) / len(timing) for key in keys}
        compute_per_step = sum(avg.values())
        print('-' * 64)
        print(f'[load model] {t0.ms:.1f} ms (once)   [export] {tw.ms:.1f} ms (once)')
        print('avg/frame (ms): ' + '  '.join(f'{key[:-3]}={avg[key]:.0f}' for key in keys))
        print(f'compute/frame ≈ {compute_per_step:.0f} ms')

        csv_path = args.timing_csv or os.path.join(
            os.path.dirname(args.out), 'timing_map.csv')
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)
        cols_order = ['step', 'file', 'prep_ms', 'infer_ms', 'project_ms',
                      'merge_ms', 'movable_space_ms', 'points']
        import csv as _csv
        with open(csv_path, 'w', newline='') as fh:
            w = _csv.DictWriter(fh, fieldnames=cols_order)
            w.writeheader()
            w.writerows(timing)
            w.writerow({'step': 'AVG', 'file': '',
                        **{key: round(avg[key], 1) for key in keys}, 'points': ''})
            w.writerow({'step': 'load_model_ms', 'file': round(t0.ms, 1)})
            w.writerow({'step': 'export_ms', 'file': round(tw.ms, 1)})
        print(f'Timing CSV: {csv_path}')

    if args.show:
        viewer = o3d or try_open3d()
        if viewer:
            viewer.visualization.draw_geometries(
                [viewer.io.read_point_cloud(args.out)],
                window_name='360° Depth Map')
        else:
            print('open3d not installed — skipping --show')

    return args.out


if __name__ == '__main__':
    main()
