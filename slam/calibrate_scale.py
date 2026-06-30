#!/usr/bin/env python3
"""Recover metric scale k once, then write slam/config/scale.yaml (plan step A4).

Standalone (NOT a ROS node) so it can own /dev/ttyUSB0 directly for the one-shot
30 cm parallax drive — run this BEFORE launching the mapping stack (which gives
the port to serial_bridge). Reuses deepmap/scale_calib.recover_scale_motion_parallax.

  source slam/env.sh        # only needed for torch/cv env; no ROS used here
  python slam/calibrate_scale.py --cam 0 --port /dev/ttyUSB0 --a-cm 30

Procedure: capture D1 at origin -> drive forward a_cm -> capture D2 -> match ORB
features -> k = median(a * d1 * d2 / (d2 - d1)) over IQR inliers.
"""
import argparse
import os
import sys

import cv2
import numpy as np
import torch
import yaml

_DOCS = os.path.expanduser('~/Documents')
sys.path.insert(0, os.path.join(_DOCS, 'depth-anything', 'src'))
sys.path.insert(0, os.path.join(_DOCS, 'deepmap'))
sys.path.insert(0, os.path.join(_DOCS, 'slam'))

from depth_to_3d_timed import CFGS, pick_device, step_prep, step_infer
from depth_anything_v2.dpt import DepthAnythingV2
import scale_calib
import robot_drive   # drives via core_control + stepper_ctrl (not the 'F' command)

_MODEL_DIR = os.path.join(_DOCS, 'depth-anything', 'model')
_OUT = os.path.join(_DOCS, 'slam', 'config', 'scale.yaml')
_TEMPLATE = _OUT + '.template'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cam', type=int, default=0)
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--hz', type=int, default=3000, help='stepper step frequency')
    ap.add_argument('--encoder', default='vits', choices=['vits', 'vitb'])
    ap.add_argument('--input-size', type=int, default=518)
    ap.add_argument('--a-cm', type=float, default=30.0, help='forward drive distance')
    ap.add_argument('--d-max', type=float, default=5.0)
    ap.add_argument('--device', default='auto')
    ap.add_argument('--dry-run', action='store_true',
                    help='skip camera/robot; write placeholder k=1.0 for a pipeline test')
    args = ap.parse_args()

    # Seed defaults from the template if present
    base = {}
    if os.path.isfile(_TEMPLATE):
        with open(_TEMPLATE) as fh:
            base = yaml.safe_load(fh) or {}

    if args.dry_run:
        k, median = 1.0, 0.0
        print('[dry-run] writing placeholder k=1.0 (no camera/robot used)')
    else:
        device = pick_device(args.device)
        model = DepthAnythingV2(**CFGS[args.encoder])
        ckpt = os.path.join(_MODEL_DIR, f'depth_anything_v2_{args.encoder}.pth')
        model.load_state_dict(torch.load(ckpt, map_location='cpu'))
        model = model.to(device).eval()

        cap = cv2.VideoCapture(args.cam, cv2.CAP_V4L2)
        if not cap.isOpened():
            sys.exit(f'cannot open camera {args.cam}')

        # stepper_ctrl owns /dev/ttyUSB0 for the move; we don't hold serial here.
        def drive_fn(a_cm):
            if not robot_drive.drive_forward(a_cm, port=args.port, hz=args.hz):
                raise RuntimeError('robot drive failed (core_control/stepper_ctrl)')

        print(f'Calibrating: forward {args.a_cm} cm drive via stepper_ctrl on '
              f'{args.port} ...')
        timing = {}
        with torch.inference_mode():
            k = scale_calib.recover_scale_motion_parallax(
                cap, None, model, step_prep, step_infer, args.input_size,
                a_cm=args.a_cm, timing=timing, drive_fn=drive_fn)
            d1, _ = scale_calib._capture_relative_depth(
                cap, model, step_infer, step_prep, args.input_size)
        median = float(np.median(d1))
        cap.release()
        print(f'  k = {k:.5f}   (timing: {timing})')

    out = {
        'k': float(k),
        'd_max': float(args.d_max),
        'near_clip_m': float(base.get('near_clip_m', 0.26)),
        'ground_margin_m': float(base.get('ground_margin_m', 0.02)),
        'ceiling_height_m': float(base.get('ceiling_height_m', 0.0)),
        'calib_median_depth': float(median),
        'calib_forward_m': float(args.a_cm) / 100.0,
    }
    os.makedirs(os.path.dirname(_OUT), exist_ok=True)
    with open(_OUT, 'w') as fh:
        yaml.safe_dump(out, fh, sort_keys=False)
    print(f'Wrote {_OUT}:\n  k={out["k"]:.5f}  d_max={out["d_max"]}  '
          f'near_clip={out["near_clip_m"]}  median_rel={out["calib_median_depth"]:.4f}')


if __name__ == '__main__':
    main()
