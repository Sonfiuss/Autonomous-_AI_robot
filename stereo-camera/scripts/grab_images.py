#!/usr/bin/env python3
"""
grab_images.py  –  Snap one frame from each camera, save to disk.

No servo, no depth, no calibration needed.

Usage:
  python3 grab_images.py [options]

Options:
  --left   N        Left  camera index  (default: 0 → /dev/video0)
  --right  N        Right camera index  (default: 2 → /dev/video2)
  --width  N        Capture width       (default: 640)
  --height N        Capture height      (default: 480)
  --out    dir      Output directory    (default: captures/)
  --show            Display images with cv2.imshow before saving
  --warmup N        Discard N frames before capturing (default: 5)

Saves:
  <dir>/left_YYYYMMDD_HHMMSS.jpg
  <dir>/right_YYYYMMDD_HHMMSS.jpg
  <dir>/stereo_YYYYMMDD_HHMMSS.jpg   (side-by-side)
"""

import argparse
import os
import sys
import time
import datetime
import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description="Stereo camera single-shot capture")
    p.add_argument("--left",   type=int, default=0,          help="Left camera index")
    p.add_argument("--right",  type=int, default=2,          help="Right camera index")
    p.add_argument("--width",  type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--out",    type=str, default="captures", help="Output directory")
    p.add_argument("--show",   action="store_true",          help="Display before saving")
    p.add_argument("--warmup", type=int, default=5,          help="Warm-up frames to discard")
    return p.parse_args()


def open_camera(idx: int, w: int, h: int) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(idx)
    if not cap.isOpened():
        print(f"[grab] ERROR: cannot open /dev/video{idx * 2 if idx > 0 else 0} (index {idx})")
        return None
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, 30)
    return cap


def warmup(cap_l, cap_r, n: int):
    for _ in range(n):
        cap_l.read()
        cap_r.read()


def grab_pair(cap_l, cap_r):
    # grab() without decode to minimise timing skew between cameras
    ok_l = cap_l.grab()
    ok_r = cap_r.grab()
    if not ok_l or not ok_r:
        return None, None
    _, frame_l = cap_l.retrieve()
    _, frame_r = cap_r.retrieve()
    return frame_l, frame_r


def draw_epipolar(img: np.ndarray, label: str) -> np.ndarray:
    h, w = img.shape[:2]
    out = img.copy()
    cy = h // 2
    cv2.line(out, (0, cy), (w, cy), (0, 255, 255), 1)
    cv2.putText(out, label, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    return out


def main():
    args = parse_args()

    cap_l = open_camera(args.left,  args.width, args.height)
    cap_r = open_camera(args.right, args.width, args.height)

    if cap_l is None or cap_r is None:
        sys.exit(1)

    print(f"[grab] Cameras: idx {args.left} (left)  idx {args.right} (right)")
    print(f"[grab] Warming up ({args.warmup} frames)…")
    warmup(cap_l, cap_r, args.warmup)

    frame_l, frame_r = grab_pair(cap_l, cap_r)

    cap_l.release()
    cap_r.release()

    if frame_l is None or frame_r is None:
        print("[grab] ERROR: failed to read frames")
        sys.exit(1)

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    os.makedirs(args.out, exist_ok=True)

    path_l      = os.path.join(args.out, f"left_{ts}.jpg")
    path_r      = os.path.join(args.out, f"right_{ts}.jpg")
    path_stereo = os.path.join(args.out, f"stereo_{ts}.jpg")

    cv2.imwrite(path_l, frame_l)
    cv2.imwrite(path_r, frame_r)

    combined = np.hstack([
        draw_epipolar(frame_l, f"LEFT  (idx {args.left})"),
        draw_epipolar(frame_r, f"RIGHT (idx {args.right})"),
    ])
    cv2.imwrite(path_stereo, combined)

    print(f"[grab] Saved: {path_l}")
    print(f"[grab] Saved: {path_r}")
    print(f"[grab] Saved: {path_stereo}")

    if args.show:
        cv2.imshow("Stereo grab", combined)
        print("[grab] Press any key to close…")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
