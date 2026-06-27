#!/usr/bin/env python3
"""
run.py — drivable-area detection on the robot's stereo camera.

Pipeline per frame:
  1. Grab left+right (live USB cameras) or load a --left/--right image pair.
  2. Rectify (uncalibrated ORB) — skipped for already-rectified inputs (--no-rectify).
  3. SGBM + WLS disparity                          (stereo.py)
  4. U/V-disparity histograms                      (disparity.py)
  5. Free-space boundary: NumPy Viterbi + V-disparity road plane (boundary.py)
  6. Green overlay (drivable ground) blended onto the left frame -> captures/.

Examples:
  python3 run.py                                   # live cameras, write captures/area.jpg
  python3 run.py --display                         # + live window
  python3 run.py --left left.png --right right.png --no-rectify   # offline KITTI sample
  python3 run.py --num-disp 128 --obstacle-height 30             # tuning
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import cv2

import stereo
import disparity
import boundary
import rectify

_HERE = Path(__file__).resolve().parent
_DEFAULT_OUT = _HERE / "captures" / "area.jpg"

# Live-camera defaults (match stereo-camera): left=video0, right=video2 on the Jetson.
_LEFT_IDX = 0
_RIGHT_IDX = 2
_WIDTH = 640
_HEIGHT = 480
_FPS = 30
_WARMUP = 5
_ALPHA = 0.45    # overlay opacity


def _open_camera(idx):
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, _WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, _HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, _FPS)
    return cap


def render_overlay(left_bgr, boundary_rows, alpha=_ALPHA):
    """Blend green over every pixel at or below the per-column boundary row."""
    h, w = left_bgr.shape[:2]
    br = np.clip(boundary_rows, 0, h).astype(np.int32)
    free_mask = np.arange(h)[:, None] >= br[None, :]      # (H, W)

    green = np.zeros_like(left_bgr)
    green[:] = (0, 255, 0)
    blended = cv2.addWeighted(left_bgr, 1 - alpha, green, alpha, 0)

    out = left_bgr.copy()
    out[free_mask] = blended[free_mask]
    # Draw the boundary polyline for clarity.
    pts = np.column_stack([np.arange(w), br]).astype(np.int32)
    cv2.polylines(out, [pts], False, (0, 0, 255), 2)
    return out


def detect(left_bgr, right_bgr, args):
    """Run the full pipeline on one BGR stereo pair; returns the annotated frame."""
    if args.rectify:
        left_bgr, lg, rg, ok = rectify.rectify_pair(left_bgr, right_bgr)
        if not ok:
            print("[run] WARNING: rectification failed — disparity may be noisy.")
    else:
        lg = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
        rg = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)

    cfg = stereo.StereoConfig()
    cfg.num_disparities = args.num_disp
    cfg.block_size = args.block
    cfg.baseline_mm = args.baseline_mm
    cfg.focal_px = args.focal_px

    depth = stereo.get_depth_map(lg, rg, cfg)
    v_disp, u_disp = disparity.u_v_disparity(depth, cfg.num_disparities)
    free_bound = boundary.free_boundary(u_disp, args.obstacle_height)
    project = boundary.road_plane(v_disp, args.v_thresh, args.hough_thresh)

    if project is None:
        print("[run] WARNING: no road plane found — no free space estimated.")
        return left_bgr

    boundary_rows = project(free_bound.astype(np.float64))
    out = render_overlay(left_bgr, boundary_rows)

    # Distance to the nearest real obstacle (only columns whose chosen disparity
    # actually has enough U-disparity accumulation to be an obstacle, not empty space).
    cols = np.arange(free_bound.shape[0])
    real = u_disp[free_bound, cols] > args.obstacle_height
    nearest_mm = np.inf
    if real.any():
        dist_mm = stereo.disparity_to_distance_mm(free_bound[real], cfg)
        nearest_mm = float(np.min(dist_mm))
    label = (f"nearest obstacle: {nearest_mm / 1000.0:.2f} m"
             if np.isfinite(nearest_mm) else "nearest obstacle: --")
    cv2.putText(out, label, (8, out.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
    cv2.putText(out, label, (8, out.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return out


def run_files(args):
    left = cv2.imread(args.left)
    right = cv2.imread(args.right)
    if left is None or right is None:
        sys.exit(f"[run] Could not read {args.left} / {args.right}")
    out = detect(left, right, args)
    cv2.imwrite(args.out, out)
    print(f"[run] Wrote {args.out}")
    if args.display:
        cv2.imshow("area-detection", out)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def run_camera(args):
    cap_l = _open_camera(args.device_left)
    cap_r = _open_camera(args.device_right)
    if cap_l is None or cap_r is None:
        sys.exit(f"[run] Could not open cameras {args.device_left}/{args.device_right}")
    for _ in range(_WARMUP):
        cap_l.read()
        cap_r.read()

    print("[run] Live mode. Ctrl-C (or 'q' in the window) to stop.")
    try:
        while True:
            ok_l, left = cap_l.read()
            ok_r, right = cap_r.read()
            if not (ok_l and ok_r):
                print("[run] Frame grab failed.")
                break
            t0 = time.time()
            out = detect(left, right, args)
            cv2.imwrite(args.out, out)
            fps = 1.0 / max(time.time() - t0, 1e-3)
            if args.display:
                cv2.putText(out, f"{fps:4.1f} FPS", (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.imshow("area-detection", out)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if args.once:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()
        print(f"[run] Stopped. Last frame: {args.out}")


def build_parser():
    p = argparse.ArgumentParser(description="Drivable-area detection (stereo).")
    p.add_argument("--left", help="left image (offline mode)")
    p.add_argument("--right", help="right image (offline mode)")
    p.add_argument("--device-left", type=int, default=_LEFT_IDX)
    p.add_argument("--device-right", type=int, default=_RIGHT_IDX)
    p.add_argument("--out", default=str(_DEFAULT_OUT))
    p.add_argument("--display", action="store_true", help="show a live window")
    p.add_argument("--once", action="store_true", help="grab a single live frame and exit")
    rect = p.add_mutually_exclusive_group()
    rect.add_argument("--rectify", dest="rectify", action="store_true")
    rect.add_argument("--no-rectify", dest="rectify", action="store_false")
    p.set_defaults(rectify=None)
    # Tuning knobs (defaults adapted for 640x480).
    p.add_argument("--num-disp", type=int, default=stereo.StereoConfig.num_disparities)
    p.add_argument("--block", type=int, default=stereo.StereoConfig.block_size)
    p.add_argument("--obstacle-height", type=int, default=25)
    p.add_argument("--v-thresh", type=int, default=50)
    p.add_argument("--hough-thresh", type=int, default=50)
    # Stereo geometry for metric distance (dist = focal_px * baseline_mm / disparity).
    p.add_argument("--baseline-mm", type=float, default=stereo.StereoConfig.baseline_mm,
                   help="distance between the two cameras (mm)")
    p.add_argument("--focal-px", type=float, default=stereo.StereoConfig.focal_px,
                   help="rectified focal length (px); set from calibration for accuracy")
    return p


def main():
    args = build_parser().parse_args()
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    file_mode = bool(args.left or args.right)
    # Default: rectify live cameras; trust offline pairs (e.g. KITTI) as already rectified.
    if args.rectify is None:
        args.rectify = not file_mode
    if file_mode:
        if not (args.left and args.right):
            sys.exit("[run] Provide both --left and --right.")
        run_files(args)
    else:
        run_camera(args)


if __name__ == "__main__":
    main()
