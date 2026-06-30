#!/usr/bin/env python3
"""
explore_map — autonomous explore-and-map loop.

The robot drives toward the LARGEST reachable free area in front of it, stopping
every --step-cm centimetres. At each stop it captures an image, runs a depthmap,
back-projects to 3D, places those points at the robot's accumulated pose, and
MERGES them into one growing global ("origin") point cloud.

Nothing here re-implements the proven stages — it chains them:
  - obstacle_grid.py  → BEV occupancy + flood-fill reachability (pick the area)
  - slam/robot_drive  → drive_forward(cm) / spin left|right (move)
  - rotate_scan.py    → camera open + fresh-frame capture
  - depth_to_3d.py    → depth → 3D back-projection
  - merge_360.py      → world transform + PLY write + ICP merge

The robot has no encoder/IMU, so pose is dead-reckoned from the COMMANDED moves
(same assumption the project's odometry already makes). Long runs drift; --voxel
ICP merge corrects some of it. The loop refuses to advance unless the cell
directly ahead is confirmed free.

Usage:
  python explore_map.py                              # default 30 cm steps, cap 12
  python explore_map.py --step-cm 30 --max-steps 20
  python explore_map.py --max-dist-m 5 --voxel 0.03  # ICP merge, 5 m budget
  python explore_map.py --dry-run                    # camera only, no robot UART
  python explore_map.py --test-explore               # offline: no robot, no camera
"""
import argparse
import os
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
_DEPTH_SRC = os.path.abspath(os.path.join(ROOT, '..', 'depth-anything', 'src'))
sys.path.insert(0, _DEPTH_SRC)                       # obstacle_grid, depth_to_3d, merge_360
sys.path.insert(0, ROOT)                             # rotate_scan (capture helpers)
sys.path.insert(0, os.path.join(ROOT, '..', 'slam'))  # robot_drive

from depth_to_3d import depth_to_points, fit_floor_plane  # noqa: E402
from merge_360 import (                               # noqa: E402
    transform_to_world, write_ply, try_open3d, to_o3d, icp_merge,
)
import obstacle_grid as og                            # noqa: E402
import rotate_scan                                    # noqa: E402  capture_fresh/configure_camera
import robot_drive                                    # noqa: E402


# ─────────────────────────────────────────────────────────────────────────────
# Pose — dead-reckoned world pose (X right, Z forward, yaw about +Y, +yaw = left)
# ─────────────────────────────────────────────────────────────────────────────
class Pose:
    """Accumulated world pose from commanded moves. Frame matches depth_to_points
    / transform_to_world: X right, Y up, Z forward; yaw=0 faces +Z."""

    def __init__(self):
        self.x = 0.0
        self.z = 0.0
        self.yaw = 0.0   # radians, +CCW (left)

    def advance(self, dist_m):
        """Move straight along current heading."""
        self.x += dist_m * np.sin(self.yaw)
        self.z += dist_m * np.cos(self.yaw)

    def turn(self, deg, direction):
        """Rotate in place. direction 'left' = +yaw (CCW), 'right' = -yaw."""
        d = np.deg2rad(abs(deg))
        self.yaw += d if direction == 'left' else -d

    def __repr__(self):
        return (f'Pose(x={self.x:+.2f} z={self.z:+.2f} '
                f'yaw={np.degrees(self.yaw):+.1f}°)')


def transform_to_world_pose(pts_cam, pose, cam_offset):
    """merge_360.transform_to_world (yaw + cam_offset) PLUS robot translation.

    transform_to_world handles the in-place rotation case; a moving robot also
    needs to shift every cloud by its accumulated (x, z) ground position."""
    pw = transform_to_world(pts_cam, pose.yaw, cam_offset)
    pw[:, 0] += pose.x
    pw[:, 2] += pose.z
    return pw


# ─────────────────────────────────────────────────────────────────────────────
# Navigation — pick the largest reachable free area from the BEV occupancy grid
# ─────────────────────────────────────────────────────────────────────────────
def goal_from_bev(grid, cell_size, range_x):
    """From an obstacle_grid occupancy grid pick a heading toward the largest
    reachable free area. grid==3 cells are free AND flood-reachable from the
    robot, so they already form one connected region; steer to its centroid.

    Returns (turn_deg, n_free, ahead_clear):
      turn_deg    signed degrees to face the area (+ = left/CCW, - = right)
      n_free      number of drivable cells (size of the available area)
      ahead_clear True if the cells straight ahead are free (safe to advance)
    """
    nz, nx = grid.shape
    free = np.argwhere(grid == 3)                     # (row, col): row 0 near robot
    if len(free) == 0:
        return 0.0, 0, False

    rows, cols = free[:, 0], free[:, 1]
    # Cell centre → robot-local world coords (X right, Z forward)
    xs = (cols + 0.5) * cell_size - range_x
    zs = (rows + 0.5) * cell_size
    xc, zc = float(xs.mean()), float(zs.mean())

    bearing = np.degrees(np.arctan2(xc, zc))          # + = target to the right
    turn_deg = -bearing                                # + = left turn (CCW)

    # Ahead-clear / safety: central columns must hold free cells and no blocked
    # cell in the near band straight in front of the robot.
    cmid = nx // 2
    r0 = int(rows.min())                               # nearest visible free row
    central_free = (np.abs(cols - cmid) <= 1)
    has_central_free = bool(central_free.any())
    near_band = grid[r0:r0 + 3, max(0, cmid - 1):cmid + 2]
    blocked_ahead = bool((near_band == 2).any())
    ahead_clear = has_central_free and not blocked_ahead

    return float(turn_deg), int(len(free)), ahead_clear


# ─────────────────────────────────────────────────────────────────────────────
# Per-stop perception — one depth inference → nav grid (local) + map cloud (world)
# ─────────────────────────────────────────────────────────────────────────────
def grid_args(args):
    """Namespace of the occupancy-grid knobs obstacle_grid.build_occupancy reads."""
    return SimpleNamespace(grid_range_x=args.grid_range_x,
                           grid_range_z=args.grid_range_z,
                           cell_size=args.cell_size,
                           min_pts_cell=args.min_pts_cell)


def _rot_to_up(normal):
    """Rotation that maps `normal` (unit) onto +Y (Rodrigues)."""
    n = normal / (np.linalg.norm(normal) + 1e-9)
    if n[1] < 0:
        n = -n                                  # floor normal must point up
    up = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    v = np.cross(n, up)
    s = np.linalg.norm(v)
    c = float(np.dot(n, up))
    if s < 1e-6:
        return np.eye(3, dtype=np.float32)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=np.float32)
    return (np.eye(3, dtype=np.float32) + vx + vx @ vx * ((1.0 - c) / (s * s)))


def level_and_scale(pts_cam, floor_mask, camera_height, h_floor):
    """Metric anchor via the floor plane — the fix for the "3 separate copies" bug.

    Depth-Anything depth is per-frame non-metric, so each capture has its own scale
    AND its own apparent floor tilt → the same object lands at a different place each
    frame. Here we fit the floor, ROTATE so it is horizontal (+Y up), then SCALE so
    the camera sits exactly `camera_height` above it. Every frame is then in the same
    metric, gravity-aligned frame → captures of one object overlap.

    Returns (pts_metric, scale, ok). On a bad/absent floor fit ok=False and the
    points are returned unchanged so the caller can fall back.
    """
    normal, centroid = fit_floor_plane(pts_cam, floor_mask)
    if normal is None:
        return pts_cam, 1.0, False
    R = _rot_to_up(normal)
    p = pts_cam @ R.T                            # floor now horizontal
    floor_y = float((centroid @ R.T)[1])         # camera→floor (camera at origin, floor below)
    h_units = abs(floor_y)
    if h_units < 1e-4:
        return pts_cam, 1.0, False
    s = camera_height / h_units
    p = p * s
    p[:, 1] += camera_height                     # camera at +camera_height, floor at Y≈0
    return p.astype(np.float32), float(s), True


def perceive(model, pose, img, args):
    """Run depth once; return (world_pts, colors, occupancy_grid_or_None, scale)."""
    H, W = img.shape[:2]
    depth = model.infer_image(img, args.input_size)
    depth = cv2.resize(depth.astype(np.float32), (W, H), interpolation=cv2.INTER_LINEAR)

    # Raw camera-frame 3D (no tilt, no height) — the floor fit derives the true
    # tilt + metric scale below, so we don't rely on the fixed --tilt guess.
    pts, cols = depth_to_points(depth, img, args.fov, args.stride,
                                camera_height=0.0, tilt_deg=0.0)

    ys, xs = np.mgrid[0:H:args.stride, 0:W:args.stride]
    H_s, W_s = ys.shape
    n_floor = max(1, int(H_s * args.bottom_frac))
    row_idx = np.arange(H_s * W_s) // W_s
    floor_mask = row_idx >= (H_s - n_floor)

    if args.metric_floor:
        pts, scale, ok = level_and_scale(pts, floor_mask, args.camera_height, args.h_floor)
        if not ok:
            # fallback: fixed tilt + manual scale (old behaviour)
            pts, cols = depth_to_points(depth, img, args.fov, args.stride,
                                        args.camera_height, args.tilt)
            pts = pts * args.scale
            scale = args.scale
    else:
        pts, cols = depth_to_points(depth, img, args.fov, args.stride,
                                    args.camera_height, args.tilt)
        pts = pts * args.scale
        scale = args.scale

    # --- navigation grid (robot-relative; thresholds now real metres) ---
    grid = None
    normal, _centroid, heights = og.fit_plane_robust(pts, floor_mask, args.h_floor)
    if normal is not None:
        residual = og.ground_profile_residual(pts, heights,
                                               args.grid_range_z, args.cell_size)
        cls_flat = og.classify(residual, args.h_floor, args.h_obs, args.h_max)
        grid = og.build_occupancy(pts, cls_flat, grid_args(args))

    # --- map cloud (placed at the robot's accumulated world pose) ---
    wpts = transform_to_world_pose(pts, pose, args.cam_offset)
    return wpts.astype(np.float32), cols.astype(np.uint8), grid, scale


# ─────────────────────────────────────────────────────────────────────────────
# Cloud accumulation / save
# ─────────────────────────────────────────────────────────────────────────────
def nearest_merge(pts, cols, viewpoint, ang_res_deg):
    """Spherical z-buffer — for each viewing DIRECTION from `viewpoint`, keep only
    the NEAREST point. Depth-Anything depth is per-frame non-metric, so the same
    surface lands at a different radius each capture → concentric duplicate shells.
    Binning by (azimuth, elevation) and keeping min-range per cell collapses those
    shells into one nearest surface. This is the "lấy point gần nhất" rule.

    viewpoint: (3,) world point to measure direction/range from.
    ang_res_deg: angular cell size; smaller = denser surface, more points kept.
    """
    d = pts - viewpoint
    r = np.linalg.norm(d, axis=1)
    horiz = np.hypot(d[:, 0], d[:, 2])
    az = np.arctan2(d[:, 0], d[:, 2])          # around vertical (Y) axis
    el = np.arctan2(d[:, 1], horiz + 1e-9)     # up/down
    res = np.deg2rad(max(1e-3, ang_res_deg))
    ai = np.floor(az / res).astype(np.int64)
    ei = np.floor(el / res).astype(np.int64)

    # sort by (cell, range) → within each angular cell the first row is nearest
    order = np.lexsort((r, ei, ai))
    ai_s, ei_s = ai[order], ei[order]
    first = np.empty(len(order), dtype=bool)
    first[0] = True
    first[1:] = (ai_s[1:] != ai_s[:-1]) | (ei_s[1:] != ei_s[:-1])
    sel = order[first]
    return pts[sel], cols[sel]


def voxel_dedup(pts, cols, voxel):
    """One point per `voxel`-sized cell (numpy, no open3d). With the captures now
    metric-aligned, overlapping copies fall in the same cell → this thins them to
    a single surface instead of 3× density."""
    if voxel <= 0:
        return pts, cols
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx], cols[idx]


def icp_merge_clouds(all_pts, all_cols, voxel):
    """Refine the dead-reckon placement with open3d ICP, then merge.

    Metric floor-anchoring already fixes scale + level, so each cloud is roughly in
    the right world spot — ICP only has to correct the residual dead-reckon pose
    drift (point-to-plane, rigid, initialised from the current placement). Reuses
    merge_360.icp_merge. Falls back to concat if open3d is missing."""
    o3d = try_open3d()
    if o3d is None:
        print('  [icp] open3d unavailable → concat fallback')
        return np.vstack(all_pts), np.vstack(all_cols)
    if len(all_pts) == 1:
        return all_pts[0], all_cols[0]
    clouds = [to_o3d(o3d, p, c) for p, c in zip(all_pts, all_cols)]
    merged = icp_merge(o3d, clouds, voxel)
    pts = np.asarray(merged.points, dtype=np.float32)
    cols = (np.asarray(merged.colors) * 255.0).astype(np.uint8)
    return pts, cols


def save_cloud(all_pts, all_cols, cam_origins, out_path, args, final=False):
    """Merge per-capture clouds and write a PLY.

    Split-into-copies is fixed upstream by metric floor-anchoring (level_and_scale);
    the merge then controls residual drift + density:
      'icp'     open3d point-to-plane ICP refines pose drift, then merge (final only)
      'all'     raw concat (overlapping copies stack; --voxel thins)
      'nearest' spherical z-buffer, keep closest point per direction
    ICP is heavy, so incremental saves (final=False) use cheap concat; ICP runs on
    the final write. --voxel applies a numpy dedup after any mode."""
    if not all_pts:
        print('  [save] nothing to write')
        return 0

    if final and args.merge == 'icp':
        pts, cols = icp_merge_clouds(all_pts, all_cols, args.voxel)
    else:
        pts = np.vstack(all_pts)
        cols = np.vstack(all_cols)
        if final and args.merge == 'nearest' and cam_origins:
            origins = np.asarray(cam_origins, dtype=np.float32)
            if args.view_from == 'start':
                vp = origins[0]
            elif args.view_from == 'last':
                vp = origins[-1]
            else:
                vp = origins.mean(axis=0)
            pts, cols = nearest_merge(pts, cols, vp, args.ang_res)

    pts, cols = voxel_dedup(pts, cols, args.voxel)
    write_ply(out_path, pts, cols)
    return len(pts)


# ─────────────────────────────────────────────────────────────────────────────
# Camera (reuses rotate_scan's MJPG config + fresh-frame drain)
# ─────────────────────────────────────────────────────────────────────────────
def open_camera(args):
    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        raise RuntimeError(f'cannot open camera index {args.cam}')
    rotate_scan.configure_camera(cap, args)
    return cap


def grab(cap, flush_n):
    ok, frame = rotate_scan.capture_fresh(cap, flush_n)
    if not ok or frame is None:
        raise RuntimeError('camera read failed')
    return frame


# ─────────────────────────────────────────────────────────────────────────────
# Main explore loop
# ─────────────────────────────────────────────────────────────────────────────
def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Autonomous explore-and-map: drive to largest area, '
                    'capture+depth every step, merge into one cloud',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    g = p.add_argument_group('explore')
    g.add_argument('--step-cm', type=float, default=30.0,
                   help='Forward distance per step (cm)')
    g.add_argument('--max-steps', type=int, default=12,
                   help='Stop after this many steps (distance cap)')
    g.add_argument('--max-dist-m', type=float, default=None,
                   help='Also stop once total travelled distance exceeds this (m)')
    g.add_argument('--max-turn-deg', type=float, default=45.0,
                   help='Clamp per-step turn toward the area (deg)')
    g.add_argument('--min-free-cells', type=int, default=6,
                   help='Stop if the largest reachable area is smaller than this')

    m = p.add_argument_group('motion / camera')
    m.add_argument('--port', default='/dev/ttyUSB0', help='ESP32 serial port')
    m.add_argument('--hz', type=int, default=3000, help='Stepper step frequency')
    m.add_argument('--cam', type=int, default=0, help='Camera device index')
    m.add_argument('--width', type=int, default=1280)
    m.add_argument('--height', type=int, default=720)
    m.add_argument('--flush', type=int, default=5,
                   help='Frames to grab-and-discard before each capture')
    m.add_argument('--settle', type=float, default=0.3,
                   help='Dwell after a move before capture (anti motion-blur)')

    d = p.add_argument_group('depth / projection (match obstacle_grid defaults)')
    d.add_argument('--encoder', default='vits', choices=['vits', 'vitb'])
    d.add_argument('--input-size', type=int, default=518)
    d.add_argument('--device', default='auto', choices=['auto', 'cuda', 'cpu'])
    d.add_argument('--fov', type=float, default=60.0)
    d.add_argument('--stride', type=int, default=2)
    d.add_argument('--camera-height', type=float, default=0.32)
    d.add_argument('--tilt', type=float, default=15.0)
    d.add_argument('--scale', type=float, default=1.0,
                   help='Multiply 3D coords to real metres (deepmap/scale_calib)')
    d.add_argument('--cam-offset', type=float, default=0.0,
                   help='Camera forward offset from the turn centre (m)')
    # classification thresholds (forwarded to obstacle_grid helpers)
    d.add_argument('--bottom-frac', type=float, default=0.30)
    d.add_argument('--h-floor', type=float, default=0.04)
    d.add_argument('--h-obs', type=float, default=0.06)
    d.add_argument('--h-max', type=float, default=2.0)
    d.add_argument('--grid-range-x', type=float, default=2.0)
    d.add_argument('--grid-range-z', type=float, default=4.0)
    d.add_argument('--cell-size', type=float, default=0.10)
    d.add_argument('--min-pts-cell', type=int, default=2)
    d.add_argument('--metric-floor', dest='metric_floor', action='store_true',
                   default=True,
                   help='Anchor each frame to a real metric scale via the floor '
                        'plane (level + scale to camera-height). Fixes split copies.')
    d.add_argument('--no-metric-floor', dest='metric_floor', action='store_false',
                   help='Disable floor anchoring; use fixed --tilt + --scale instead.')

    o = p.add_argument_group('output / flow')
    o.add_argument('--out', default=os.path.join(ROOT, 'output', 'explore_map.ply'),
                   help='Merged point cloud PLY')
    o.add_argument('--merge', choices=['icp', 'nearest', 'all'], default='icp',
                   help="'icp' = open3d point-to-plane refine of dead-reckon drift "
                        "then merge (default); 'all' = raw concat (--voxel to thin); "
                        "'nearest' = keep closest point per direction.")
    o.add_argument('--ang-res', type=float, default=0.3,
                   help='Angular cell size (deg) for nearest merge. Smaller = denser.')
    o.add_argument('--view-from', choices=['mean', 'start', 'last'], default='mean',
                   help='Viewpoint the nearest merge measures direction/range from')
    o.add_argument('--voxel', type=float, default=0.02,
                   help='Voxel dedup size in m on save (numpy, no open3d). 0 = off.')
    o.add_argument('--dry-run', action='store_true',
                   help='Capture + map but DO NOT command the robot (no UART)')
    o.add_argument('--test-explore', action='store_true',
                   help='Offline: no robot, no camera — fixed image every stop')
    o.add_argument('--test-img', default=os.path.join(ROOT, 'test.jpg'),
                   help='Image used for --test-explore stops')
    return p.parse_args(argv)


def move_robot(pose, turn_deg, step_cm, args):
    """Execute one step on the hardware (skipped in dry/test runs) and update pose."""
    turn_deg = float(np.clip(turn_deg, -args.max_turn_deg, args.max_turn_deg))
    if abs(turn_deg) >= 1.0:
        direction = 'left' if turn_deg > 0 else 'right'
        if not (args.dry_run or args.test_explore):
            ok = robot_drive.drive(f'spin {direction} {abs(turn_deg):.1f}',
                                   port=args.port, hz=args.hz)
            if not ok:
                print('  WARNING: spin step failed')
        pose.turn(turn_deg, direction)

    if not (args.dry_run or args.test_explore):
        ok = robot_drive.drive_forward(step_cm, port=args.port, hz=args.hz)
        if not ok:
            print('  WARNING: forward step failed')
    pose.advance(step_cm / 100.0)


def main(argv=None):
    args = parse_args(argv)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    device = og.pick_device(args.device)
    print(f'device={device} encoder={args.encoder} scale={args.scale}  '
          f'step={args.step_cm}cm cap={args.max_steps} steps')
    if args.scale == 1.0:
        print('  [note] --scale 1.0 → thresholds in RELATIVE depth units, not real '
              'metres. Set --scale from deepmap/scale_calib for true cm.')

    model = og.load_model(args.encoder, device)

    cap = None
    fixed_img = None
    if args.test_explore:
        fixed_img = cv2.imread(args.test_img)
        if fixed_img is None:
            raise FileNotFoundError(f'--test-explore needs an image: {args.test_img}')
        print(f'  [test-explore] offline, fixed image {args.test_img} '
              f'{fixed_img.shape[1]}x{fixed_img.shape[0]}')
    else:
        cap = open_camera(args)

    pose = Pose()
    all_pts, all_cols, cam_origins = [], [], []
    travelled = 0.0

    try:
        for step in range(args.max_steps):
            t0 = time.time()
            img = fixed_img if fixed_img is not None else grab(cap, args.flush)

            wpts, cols, grid, scale = perceive(model, pose, img, args)
            all_pts.append(wpts)
            all_cols.append(cols)
            # camera world origin at capture time (viewpoint for nearest merge)
            cam_origins.append([pose.x, args.camera_height, pose.z])

            if grid is None:
                print(f'step {step}: no floor plane → stop')
                break
            turn_deg, n_free, ahead_clear = goal_from_bev(
                grid, args.cell_size, args.grid_range_x)
            n_block = int((grid == 2).sum())
            print(f'step {step}: {pose}  area={n_free} free cells '
                  f'({n_block} blocked)  turn={turn_deg:+.1f}°  '
                  f'ahead={"clear" if ahead_clear else "BLOCKED"}  '
                  f'scale={scale:.3f}  pts+={len(wpts)}  '
                  f'[{(time.time()-t0)*1000:.0f}ms]')

            # incremental save so a long run is never lost
            save_cloud(all_pts, all_cols, cam_origins, args.out, args)

            # stop conditions (distance cap handled by loop range; area / safety here)
            if n_free < args.min_free_cells:
                print(f'  largest area < {args.min_free_cells} cells → boxed in, stop')
                break
            if not ahead_clear:
                print('  cell directly ahead not confirmed free → stop (safety)')
                break
            if args.max_dist_m is not None and travelled >= args.max_dist_m:
                print(f'  travelled {travelled:.2f} m ≥ {args.max_dist_m} m → stop')
                break

            move_robot(pose, turn_deg, args.step_cm, args)
            travelled += args.step_cm / 100.0
            time.sleep(args.settle)
    finally:
        if cap is not None:
            cap.release()

    n = save_cloud(all_pts, all_cols, cam_origins, args.out, args, final=True)
    if all_pts:
        merged = np.vstack(all_pts)
        lo = merged.min(axis=0)
        hi = merged.max(axis=0)
        print(f'\nDONE  final pose {pose}  travelled {travelled:.2f} m')
        print(f'  {n} points → {args.out}')
        print(f'  bounds  X[{lo[0]:+.2f},{hi[0]:+.2f}]  '
              f'Y[{lo[1]:+.2f},{hi[1]:+.2f}]  Z[{lo[2]:+.2f},{hi[2]:+.2f}] (m·scale)')
    return args.out


if __name__ == '__main__':
    main()
