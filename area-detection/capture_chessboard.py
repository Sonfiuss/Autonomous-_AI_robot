#!/usr/bin/env python3
"""
capture_chessboard.py — collect synchronized stereo chessboard pairs for calibration.

Saves a left/right image pair only when BOTH cameras detect the full chessboard, so the
pairs are ready to feed a stereo calibration (e.g. stereo-camera/tools/stereo_calibrate.py).
This is the proper fix for area-detection's weak link: the uncalibrated ORB rectification
in rectify.py (see README).

Default board matches stereo_calibrate.py: 9x6 inner corners, 25 mm squares.

Modes:
  --display   cv2 window: corners drawn, green/red border = both-found/missing.
              SPACE saves a pair, 'q' quits.
  --auto      headless: auto-saves a pair every --interval s whenever both detect
              (debounced). Use this when no monitor is attached to the Jetson; Ctrl-C stops.

Examples:
  python3 capture_chessboard.py --display              # interactive, SPACE to capture
  python3 capture_chessboard.py --auto --interval 2    # headless auto-capture
  python3 capture_chessboard.py --display --cols 9 --rows 6 --square 25
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import cv2

_HERE = Path(__file__).resolve().parent
_DEFAULT_OUT = _HERE / "captures" / "calib"

# Live-camera defaults (match run.py): left=video0, right=video2.
_LEFT_IDX = 0
_RIGHT_IDX = 2
_WIDTH = 640
_HEIGHT = 480
_FPS = 30
_WARMUP = 5

_FIND_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
_SUBPIX_CRIT = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def _open_camera(idx, width, height):
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, _FPS)
    return cap


def _find_corners(gray, pattern):
    """Return refined corners (Nx1x2) or None if the full board isn't found."""
    ret, corners = cv2.findChessboardCorners(gray, pattern, _FIND_FLAGS)
    if not ret:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), _SUBPIX_CRIT)


def _next_index(out_dir):
    """Resume numbering after any pairs already in out_dir."""
    existing = sorted(out_dir.glob("left_*.png"))
    if not existing:
        return 0
    return max(int(p.stem.split("_")[1]) for p in existing) + 1


def _save_pair(out_dir, idx, left_bgr, right_bgr):
    lp = out_dir / f"left_{idx:02d}.png"
    rp = out_dir / f"right_{idx:02d}.png"
    cv2.imwrite(str(lp), left_bgr)
    cv2.imwrite(str(rp), right_bgr)
    return lp, rp


def _annotate(disp, pattern, corners, border, status, count):
    cv2.drawChessboardCorners(disp, pattern, corners, corners is not None)
    h, w = disp.shape[:2]
    cv2.rectangle(disp, (0, 0), (w - 1, h - 1), border, 4)
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(disp, status, (10, 26), font, 0.65, border, 2)
    cv2.putText(disp, f"pairs: {count}", (10, 52), font, 0.6, (200, 200, 50), 2)
    return disp


def run(args):
    pattern = (args.cols, args.rows)   # OpenCV order: (width, height) = (cols, rows)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap_l = _open_camera(args.device_left, args.width, args.height)
    cap_r = _open_camera(args.device_right, args.width, args.height)
    if cap_l is None or cap_r is None:
        sys.exit(f"[capture] Could not open cameras "
                 f"{args.device_left}/{args.device_right}")
    for _ in range(_WARMUP):
        cap_l.read()
        cap_r.read()

    count = _next_index(out_dir)
    if count:
        print(f"[capture] Resuming — {count} pair(s) already in {out_dir}")
    print(f"[capture] Board {args.cols}x{args.rows} inner corners, "
          f"square={args.square}mm -> {out_dir}")
    if args.display:
        print("[capture] Display mode: SPACE = save pair (when border is green), q = quit.")
    else:
        print(f"[capture] Auto mode: saving every {args.interval}s when both detect. "
              f"Ctrl-C to stop.")

    last_save = 0.0
    try:
        while True:
            ok_l, left = cap_l.read()
            ok_r, right = cap_r.read()
            if not (ok_l and ok_r):
                print("[capture] Frame grab failed.")
                break

            gray_l = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
            c_l = _find_corners(gray_l, pattern)
            c_r = _find_corners(gray_r, pattern)
            found = (c_l is not None) and (c_r is not None)

            saved_this_frame = False
            if not args.display and found and (time.time() - last_save) >= args.interval:
                _save_pair(out_dir, count, left, right)
                print(f"[capture] saved pair {count:02d}  (total {count + 1})")
                count += 1
                last_save = time.time()
                saved_this_frame = True

            if args.display:
                border = (0, 220, 0) if found else (0, 0, 220)
                status = "READY (SPACE to save)" if found else "searching..."
                disp_l = _annotate(left.copy(), pattern, c_l, border, "L " + status, count)
                disp_r = _annotate(right.copy(), pattern, c_r, border, "R " + status, count)
                cv2.imshow("chessboard capture (L | R)", np.hstack([disp_l, disp_r]))
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord(" "):
                    if found:
                        _save_pair(out_dir, count, left, right)
                        print(f"[capture] saved pair {count:02d}  (total {count + 1})")
                        count += 1
                    else:
                        print("[capture] not saved — board not detected in both cameras.")
            elif not saved_this_frame:
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        cap_l.release()
        cap_r.release()
        cv2.destroyAllWindows()
        print(f"[capture] Stopped. {count} pair(s) in {out_dir}")


def build_parser():
    p = argparse.ArgumentParser(
        description="Capture synchronized stereo chessboard pairs for calibration.")
    p.add_argument("--device-left", type=int, default=_LEFT_IDX)
    p.add_argument("--device-right", type=int, default=_RIGHT_IDX)
    p.add_argument("--out-dir", default=str(_DEFAULT_OUT),
                   help="where to write left_NN.png / right_NN.png")
    p.add_argument("--rows", type=int, default=6, help="inner corners vertical")
    p.add_argument("--cols", type=int, default=9, help="inner corners horizontal")
    p.add_argument("--square", type=float, default=25.0,
                   help="square size in mm (recorded for the operator; not used here)")
    p.add_argument("--width", type=int, default=_WIDTH)
    p.add_argument("--height", type=int, default=_HEIGHT)
    p.add_argument("--display", action="store_true",
                   help="show a window; SPACE saves, q quits")
    p.add_argument("--auto", action="store_true",
                   help="headless auto-capture (default when --display is absent)")
    p.add_argument("--interval", type=float, default=2.0,
                   help="seconds between auto-captures (auto mode)")
    return p


def main():
    args = build_parser().parse_args()
    run(args)


if __name__ == "__main__":
    main()
