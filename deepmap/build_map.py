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

    for k, f in enumerate(files):
        img = cv2.imread(f)
        if img is None:
            print(f'  skip (read error): {f}')
            continue

        tensor, hw  = step_prep(model, img, args.input_size)
        depth        = step_infer(model, tensor, hw)
        pts, cols    = step_project(depth, img, args.fov, args.stride)
        wpts         = transform_to_world(pts, angles[f], args.cam_offset)

        if o3d is not None:
            o3d_clouds.append(to_o3d(o3d, wpts, cols))
        else:
            all_pts.append(wpts)
            all_cols.append(cols)

        print(f'  [{k+1}/{len(files)}] {os.path.basename(f):<16} '
              f'yaw={np.rad2deg(angles[f]):6.1f}°  {len(wpts)} pts')

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

    write_ply(args.out, mpts, mcols)
    print(f'\n360° map: {len(mpts):,} points → {args.out}')

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
