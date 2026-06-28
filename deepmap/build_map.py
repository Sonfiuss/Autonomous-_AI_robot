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


def parse_args():
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
    p.add_argument('--fov', type=float, default=60.0,
                   help='Camera horizontal FOV in degrees')
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
    # Timing instrumentation
    p.add_argument('--no-drive-area', action='store_true',
                   help='Skip the movable-space step (and its timing + class colours)')
    p.add_argument('--no-class-color', action='store_true',
                   help='Keep raw RGB instead of colouring the map by class '
                        '(green=movable floor, red=obstacle)')
    p.add_argument('--timing-csv', default=None,
                   help='Per-frame timing output (default: <out dir>/timing_map.csv)')
    return p.parse_args()


def load_model(encoder, device):
    model = DepthAnythingV2(**CFGS[encoder])
    ckpt = os.path.join(_MODEL_DIR, f'depth_anything_v2_{encoder}.pth')
    if not os.path.isfile(ckpt):
        raise FileNotFoundError(
            f'Model checkpoint not found: {ckpt}\n'
            f'Download from the depth-anything repo and place in {_MODEL_DIR}/')
    model.load_state_dict(torch.load(ckpt, map_location='cpu'))
    return model.to(device).eval()


def main():
    args = parse_args()
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

    print(f'360° map: {len(files)} frames, encoder={args.encoder}, device={args.device}')

    device = pick_device(args.device)

    with Timer(device) as t0:
        model = load_model(args.encoder, device)
    print(f'[load model] {t0.ms:.1f} ms')

    use_o3d = args.icp or args.voxel > 0
    o3d = try_open3d() if use_o3d else None
    if use_o3d and o3d is None:
        print('WARNING: open3d not installed — --icp/--voxel disabled. pip install open3d')

    all_pts, all_cols = [], []
    o3d_clouds = []
    k_scale = None   # metric scale; recovered/confirmed once in test mode
    timing = []      # per-frame stage latencies (ms)

    for k, f in enumerate(files):
        img = cv2.imread(f)
        if img is None:
            print(f'  skip (read error): {f}')
            continue

        with Timer(device) as tp: tensor, hw = step_prep(model, img, args.input_size)
        with Timer(device) as ti: depth      = step_infer(model, tensor, hw)

        with Timer(device) as tpr:
            if args.test_round360:
                # Step A4 with the 30 cm move SKIPPED. First frame: run the scale
                # step (placeholder k) and confirm the metric-depth output TYPE.
                # Later frames: reuse k. Project with metric depth.
                if k_scale is None:
                    k_scale, metric = scale_calib.calibrate(
                        depth, args.d_max, test_round360=True)
                else:
                    metric = scale_calib.apply_scale(depth, k_scale, args.d_max)
                pts, cols, keep = scale_calib.project_metric(
                    metric, img, args.fov, args.stride, return_keep=True)
            else:
                pts, cols = step_project(depth, img, args.fov, args.stride)
                keep = None

        # Movable-space determination (ray-linearity drive area) + per-point class.
        # Labels each projected point drivable(floor)/obstacle and colours the map.
        ms_ms = 0.0
        if not args.no_drive_area:
            with Timer(device) as tms:
                dnorm = preprocess_depth(depth)
                polygon, _ = compute_drive_polygon(dnorm)
                drivable = scale_calib.drivable_labels(
                    depth.shape, polygon, args.stride, keep)
            ms_ms = tms.ms
            if not args.no_class_color:
                cols = scale_calib.colorize_by_class(cols, drivable)

        with Timer(device) as tm:
            wpts = transform_to_world(pts, angles[f], args.cam_offset)
            if o3d is not None:
                o3d_clouds.append(to_o3d(o3d, wpts, cols))
            else:
                all_pts.append(wpts)
                all_cols.append(cols)

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


if __name__ == '__main__':
    main()
