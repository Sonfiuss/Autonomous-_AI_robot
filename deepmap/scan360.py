#!/usr/bin/env python3
"""
scan360 — one command: robot self-spins 360°, captures, and builds the map.

Chains the two existing, hardware-proven stages into a single autonomous run:
  1. rotate_scan.main()  → robot spins 360° in steps, captures fresh frames → shots/
  2. build_map.main()    → depth → metric back-project → 360° merge → PLY (+ BEV)

Both stages are imported and called in-process (no logic is copied). Defaults are
tuned for a fast spin (hz=6000, settle=0.15) and the height-above-ground map method
(--classify height → map_360.ply + map_360_bev.png).

Usage:
  python scan360.py                       # spin + capture + build (height mode)
  python scan360.py --step-deg 45 --hz 8000
  python scan360.py --skip-spin           # rebuild map from existing shots/
  python scan360.py --skip-build          # only spin + capture
  python scan360.py --test-round360       # offline: no robot, no camera
"""

import argparse
import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT)   # so `import rotate_scan` / `import build_map` resolve here


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Self-run a 360° spin and build the map around the robot',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    # ── Spin + capture (forwarded to rotate_scan) ────────────────────────────
    g = p.add_argument_group('spin + capture')
    g.add_argument('--step-deg', type=float, default=30.0,
                   help='Degrees per step (30 → 12 frames per full rotation)')
    g.add_argument('--spin-dir', choices=['left', 'right'], default='left',
                   help='Rotation direction (left = CCW = +yaw)')
    g.add_argument('--hz', type=int, default=6000,
                   help='Stepper step frequency = spin speed (2× the old 3000)')
    g.add_argument('--settle', type=float, default=0.15,
                   help='Dwell after each rotation before capture (anti motion-blur)')
    g.add_argument('--cam', type=int, default=0, help='Camera device index')
    g.add_argument('--width', type=int, default=1280)
    g.add_argument('--height', type=int, default=720)
    g.add_argument('--port', default='/dev/ttyUSB0', help='ESP32 serial port')
    g.add_argument('--shots', default=os.path.join(ROOT, 'shots'),
                   help='Capture dir (rotate_scan output / build_map input)')

    # ── Build map (forwarded to build_map) ───────────────────────────────────
    b = p.add_argument_group('build map')
    b.add_argument('--classify', choices=['height', 'drive-area'], default='height',
                   help="Map method. 'height' = height-above-ground (PLY + BEV).")
    b.add_argument('--encoder', default='vits', choices=['vits', 'vitb'])
    b.add_argument('--fov', type=float, default=72.0, help='Camera horizontal FOV (deg)')
    b.add_argument('--keep-deg', type=float, default=None,
                   help='Central wedge kept per frame (deg). None = auto (= scan step)')
    b.add_argument('--cam-yaw-deg', type=float, default=0.0,
                   help='Constant yaw offset to align the camera with robot forward')
    b.add_argument('--voxel', type=float, default=0.0,
                   help='Voxel downsample size in metres (0 = off, needs open3d)')
    b.add_argument('--out', default=os.path.join(ROOT, 'output', 'map_360.ply'),
                   help='Output PLY path')
    b.add_argument('--show', action='store_true',
                   help='Open the 3D viewer after building (needs open3d)')

    # ── Flow control ─────────────────────────────────────────────────────────
    c = p.add_argument_group('flow control')
    c.add_argument('--skip-spin', action='store_true',
                   help='Skip the spin/capture; build from existing --shots')
    c.add_argument('--skip-build', action='store_true',
                   help='Only spin + capture; do not build the map')
    c.add_argument('--dry-run', action='store_true',
                   help='Capture images only, skip the robot UART (test without robot)')
    c.add_argument('--test-round360', action='store_true',
                   help='Offline: no robot, no camera — fixed image for every frame')

    return p.parse_args(argv)


def _spin_argv(args):
    """Build the rotate_scan arg list from the orchestrator args."""
    a = ['--step-deg', str(args.step_deg), '--spin-dir', args.spin_dir,
         '--hz', str(args.hz), '--settle', str(args.settle),
         '--cam', str(args.cam), '--width', str(args.width),
         '--height', str(args.height), '--port', args.port,
         '--out-dir', args.shots]
    if args.dry_run:
        a.append('--dry-run')
    if args.test_round360:
        a.append('--test-round360')
    return a


def _build_argv(args, shots_dir):
    """Build the build_map arg list from the orchestrator args."""
    a = ['--shots', shots_dir, '--classify', args.classify,
         '--encoder', args.encoder, '--fov', str(args.fov),
         '--cam-yaw-deg', str(args.cam_yaw_deg), '--voxel', str(args.voxel),
         '--out', args.out]
    if args.keep_deg is not None:
        a += ['--keep-deg', str(args.keep_deg)]
    if args.show:
        a.append('--show')
    if args.test_round360:
        a.append('--test-round360')
    return a


def main(argv=None):
    args = parse_args(argv)

    shots_dir = args.shots
    n_captured = None

    # ── Stage 1: spin + capture ──────────────────────────────────────────────
    if args.skip_spin:
        print('=== scan360: skip-spin → building from existing shots ===')
    else:
        print('=== scan360 [1/2]: spin 360° + capture '
              f'(step={args.step_deg}° dir={args.spin_dir} hz={args.hz} '
              f'settle={args.settle}s) ===')
        import rotate_scan
        shots_dir, n_captured = rotate_scan.main(_spin_argv(args))
        if n_captured is None:
            print('scan360: capture stage failed (camera/robot) — aborting before build.')
            return 1
        if n_captured == 0:
            print('scan360: no frames captured — nothing to build.')
            return 1
        print(f'--- captured {n_captured} frame(s) → {shots_dir}')

    if args.skip_build:
        print('=== scan360: skip-build → done after capture ===')
        return 0

    # ── Stage 2: build the 360° map ──────────────────────────────────────────
    print(f'=== scan360 [2/2]: build map (classify={args.classify} '
          f'encoder={args.encoder}) ===')
    import build_map
    out_ply = build_map.main(_build_argv(args, shots_dir))
    if not out_ply or not os.path.isfile(out_ply):
        print('scan360: build stage produced no map.')
        return 1

    print('\n=== scan360 done ===')
    print(f'  map  : {out_ply}')
    if args.classify == 'height':
        bev = os.path.join(os.path.dirname(out_ply), 'map_360_bev.png')
        if os.path.isfile(bev):
            print(f'  bev  : {bev}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
