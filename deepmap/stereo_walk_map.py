#!/usr/bin/env python3
"""
stereo_walk_map — straight-line stereo-metric mapping walk.

Drive forward in fixed steps; at every stop capture a STEREO pair, run the
stereo-ruler pipeline (DA-V2 + golden points + f*B) for a METRIC depth map,
back-project with the rectified K, place the cloud at the dead-reckoned pose
(z = 0 / 0.3 / 0.6 ...), and merge everything into one PLY.

Why this replaces explore_map's floor-anchor heuristic (level_and_scale): the
stereo f*B (5.4 cm baseline) puts EVERY frame on the same real-metre scale by
construction, so the per-frame scale guessing goes away. The floor plane is
still fitted, but only for a rotation (level the camera tilt) — never a scale.

Nothing is re-implemented — this chains proven parts:
  stereo_ruler   capture_pair / load_rectify / load_model / compute_metric_depth
  explore_map    Pose dead-reckoning, voxel_dedup, icp_merge_clouds
  slam/robot_drive  calibrated drive_forward(cm)
  merge_360      write_ply

Usage:
  python3 stereo_walk_map.py --steps 3 --step-cm 30     # hardware run
  python3 stereo_walk_map.py --test                     # offline, saved pair
Outputs in --out-dir (default deepmap/output/stereo_walk/):
  shot_N_left.jpg / shot_N_right.jpg   raw captures per stop
  depth_N_m.npy                        metric depth (m) per stop
  walk_map.ply                         merged metric cloud (incremental saves too)
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, '..', 'depth-anything', 'src')))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.abspath(os.path.join(ROOT, '..', 'slam')))

import stereo_ruler as sr                              # noqa: E402
from depth_to_3d import fit_floor_plane                # noqa: E402
from merge_360 import write_ply                        # noqa: E402
from explore_map import (                              # noqa: E402
    Pose, voxel_dedup, icp_merge_clouds, _rot_to_up,
)

_DEF_RECTIFY = os.path.abspath(os.path.join(
    ROOT, '..', 'stereo-camera', 'calib', 'stereo_rectify.yml'))
_DEF_TEST_LEFT = os.path.abspath(os.path.join(
    ROOT, '..', 'stereo-camera', 'tools', 'captures', 'left.jpg'))
_DEF_TEST_RIGHT = os.path.abspath(os.path.join(
    ROOT, '..', 'stereo-camera', 'tools', 'captures', 'righ.jpg'))


# ─────────────────────────────────────────────────────────────────────────────
# Geometry — back-project metric depth, level camera tilt, place at world pose
# ─────────────────────────────────────────────────────────────────────────────
def backproject(Z, valid, right_rect, calib, stride, z_min, z_max):
    """Metric depth (right-rectified frame) → camera-frame 3D + colors.

    Camera frame matches explore_map/depth_to_points: X right, Y UP, Z forward
    (image v grows downward, hence the minus on Y).
    Returns (pts Nx3 float32, cols Nx3 uint8 RGB, rows N source image row).
    """
    h, w = Z.shape
    ys, xs = np.mgrid[0:h:stride, 0:w:stride]
    ys, xs = ys.ravel(), xs.ravel()
    z = Z[ys, xs]
    ok = valid[ys, xs] & (z > z_min) & (z < z_max)
    z, u, v = z[ok], xs[ok], ys[ok]
    x = (u - calib.cx) * z / calib.fx
    y = -(v - calib.cy) * z / calib.fy
    pts = np.stack([x, y, z], axis=1).astype(np.float32)
    cols = right_rect[v, u][:, ::-1].astype(np.uint8)          # BGR → RGB
    return pts, cols, v


def level_to_floor(pts, rows, img_h, bottom_frac):
    """Rotate the cloud so the fitted floor is horizontal, floor at Y=0.

    The camera is tilted down; without leveling, pose.advance() along world Z
    would not match the robot's horizontal travel. ROTATION ONLY — the stereo
    scale is already metric, so unlike explore_map.level_and_scale nothing is
    scaled here. Returns (pts, cam_height_m, ok); on a failed fit the points
    are unchanged and ok=False.
    """
    floor_mask = rows >= int((1.0 - bottom_frac) * img_h)
    normal, centroid = fit_floor_plane(pts, floor_mask)
    if normal is None:
        return pts, None, False
    R = _rot_to_up(normal.astype(np.float32))
    p = pts @ R.T
    floor_y = float(centroid.astype(np.float32) @ R.T[:, 1])   # (centroid@R.T)[1]
    p[:, 1] -= floor_y                     # floor → Y=0, camera at Y=|floor_y|
    return p.astype(np.float32), -floor_y, True


def to_world(pts, pose):
    """Place a camera cloud at the dead-reckoned pose (straight walk: yaw=0)."""
    p = pts.copy()
    if abs(pose.yaw) > 1e-9:
        c, s = np.cos(pose.yaw), np.sin(pose.yaw)
        x, z = p[:, 0].copy(), p[:, 2].copy()
        p[:, 0] = c * x + s * z
        p[:, 2] = -s * x + c * z
    p[:, 0] += pose.x
    p[:, 2] += pose.z
    return p


# ─────────────────────────────────────────────────────────────────────────────
# Main walk
# ─────────────────────────────────────────────────────────────────────────────
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Stereo-metric walk map: drive N stops, metric depth per '
                    'stop, merge into one PLY',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    w = p.add_argument_group('walk')
    w.add_argument('--steps', type=int, default=3,
                   help='Number of capture stops (drives happen between them)')
    w.add_argument('--step-cm', type=float, default=30.0,
                   help='Forward distance between stops (cm)')
    w.add_argument('--port', default='/dev/ttyUSB0', help='ESP32 serial port')
    w.add_argument('--hz', type=int, default=3000, help='Stepper step frequency')
    w.add_argument('--settle', type=float, default=0.5,
                   help='Dwell after a move before capture (s)')

    c = p.add_argument_group('cameras / calib (1280x720 = stereo_rectify.yml frame)')
    c.add_argument('--device-left', type=int, default=0)
    c.add_argument('--device-right', type=int, default=2)
    c.add_argument('--width', type=int, default=1280)
    c.add_argument('--height', type=int, default=720)
    c.add_argument('--rectify', default=_DEF_RECTIFY)

    d = p.add_argument_group('depth / cloud')
    d.add_argument('--encoder', default='vits', choices=['vits', 'vitb', 'vitl'])
    d.add_argument('--input-size', type=int, default=518)
    d.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    d.add_argument('--stride', type=int, default=2, help='Pixel subsampling')
    d.add_argument('--z-min', type=float, default=0.2, help='Reject closer (m)')
    d.add_argument('--z-max', type=float, default=5.0, help='Reject farther (m)')
    d.add_argument('--bottom-frac', type=float, default=0.30,
                   help='Bottom image fraction assumed floor (for leveling)')
    d.add_argument('--no-level', dest='level', action='store_false', default=True,
                   help='Skip floor leveling (keep raw tilted camera frame)')

    o = p.add_argument_group('output')
    o.add_argument('--out-dir', default=os.path.join(ROOT, 'output', 'stereo_walk'))
    o.add_argument('--merge', choices=['icp', 'all'], default='icp',
                   help="'icp' refines residual dead-reckon drift on final save")
    o.add_argument('--voxel', type=float, default=0.02,
                   help='Voxel dedup (m) on save; 0 = off')
    o.add_argument('--test', action='store_true',
                   help='Offline: no robot, no cameras — a fixed saved pair is '
                        'reused at every stop (pipeline check; the object WILL '
                        'appear once per stop since the scene never changes)')
    o.add_argument('--test-left', default=_DEF_TEST_LEFT)
    o.add_argument('--test-right', default=_DEF_TEST_RIGHT)
    return p.parse_args(argv)


def get_pair(args, out_dir, idx):
    if args.test:
        left = cv2.imread(args.test_left)
        right = cv2.imread(args.test_right)
        if left is None or right is None:
            raise FileNotFoundError(f'--test pair missing: {args.test_left} / '
                                    f'{args.test_right}')
    else:
        left, right = sr.capture_pair(args.device_left, args.device_right,
                                      args.width, args.height)
    cv2.imwrite(os.path.join(out_dir, f'shot_{idx}_left.jpg'), left)
    cv2.imwrite(os.path.join(out_dir, f'shot_{idx}_right.jpg'), right)
    return left, right


def save_map(all_pts, all_cols, out_path, args, final=False):
    if not all_pts:
        return 0
    if final and args.merge == 'icp':
        pts, cols = icp_merge_clouds(all_pts, all_cols, max(args.voxel, 0.01))
    else:
        pts, cols = np.vstack(all_pts), np.vstack(all_cols)
    pts, cols = voxel_dedup(pts, cols, args.voxel)
    write_ply(out_path, pts, cols)
    return len(pts)


def main(argv=None):
    args = parse_args(argv)
    os.makedirs(args.out_dir, exist_ok=True)
    out_ply = os.path.join(args.out_dir, 'walk_map.ply')

    h, w = args.height, args.width
    calib = sr.load_rectify(args.rectify, (w, h))
    model, device = sr.load_model(args.encoder, args.device)
    print(f'device={device} encoder={args.encoder}  f*B={calib.fb:.2f}  '
          f'{args.steps} stops x {args.step_cm}cm  '
          f'{"TEST (offline)" if args.test else "HARDWARE"}')

    if not args.test:
        import robot_drive                       # slam/ — needs built binaries

    pose = Pose()
    all_pts, all_cols = [], []
    try:
        for i in range(args.steps):
            t0 = time.time()
            left, right = get_pair(args, args.out_dir, i)
            if (left.shape[0], left.shape[1]) != (h, w):
                raise RuntimeError(f'shot {i} is {left.shape[1]}x{left.shape[0]}'
                                   f', calib expects {w}x{h}')
            Z, valid, info = sr.compute_metric_depth(
                left, right, calib, model, input_size=args.input_size)
            np.save(os.path.join(args.out_dir, f'depth_{i}_m.npy'), Z)

            pts, cols, rows = backproject(Z, valid, info['right_rect'], calib,
                                          args.stride, args.z_min, args.z_max)
            height_note = ''
            if args.level:
                pts, cam_h, ok = level_to_floor(pts, rows, h, args.bottom_frac)
                height_note = (f'cam_h={cam_h:.2f}m' if ok
                               else 'floor fit FAILED (raw frame)')
            all_pts.append(to_world(pts, pose))
            all_cols.append(cols)

            zc = Z[valid]
            print(f'stop {i}: {pose}  golden={info["n_golden"]} '
                  f'(inl={info["n_inliers"]}, rms={info["rms"]:.4f})  '
                  f'pts={len(pts)}  Z[{zc.min():.2f},{zc.max():.2f}]m  '
                  f'{height_note}  [{time.time() - t0:.1f}s]')

            save_map(all_pts, all_cols, out_ply, args)     # incremental, cheap

            if i < args.steps - 1:
                if args.test:
                    print(f'  [test] skip drive_forward({args.step_cm:g})')
                else:
                    if not robot_drive.drive_forward(args.step_cm,
                                                     port=args.port, hz=args.hz):
                        print('  WARNING: forward step failed — stopping walk')
                        break
                    time.sleep(args.settle)
                pose.advance(args.step_cm / 100.0)
    finally:
        n = save_map(all_pts, all_cols, out_ply, args, final=True)
        if all_pts:
            m = np.vstack(all_pts)
            lo, hi = m.min(axis=0), m.max(axis=0)
            print(f'\nDONE  {len(all_pts)} stops  final pose {pose}')
            print(f'  {n} points -> {out_ply}')
            print(f'  bounds  X[{lo[0]:+.2f},{hi[0]:+.2f}]  '
                  f'Y[{lo[1]:+.2f},{hi[1]:+.2f}]  Z[{lo[2]:+.2f},{hi[2]:+.2f}] m')
    return out_ply


if __name__ == '__main__':
    main()
