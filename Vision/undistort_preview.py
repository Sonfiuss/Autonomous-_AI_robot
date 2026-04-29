"""Preview lens undistortion (curved edges -> rectilinear).

Examples:
  - Imou RTSP (from camera_config.json) + intrinsics:
      python undistort_preview.py --imou-config camera_config.json --intrinsics intrinsics.json

  - Webcam device 0:
      python undistort_preview.py --device 0 --intrinsics intrinsics.json

Keys:
  q: quit
  t: toggle side-by-side view
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Optional

import cv2

from imou_camera_capture import ImouCamera, load_camera_config
from undistort_utils import maybe_make_undistorter


def open_capture(device: Optional[int], rtsp: Optional[str]) -> cv2.VideoCapture:
    if rtsp is not None:
        cap = cv2.VideoCapture(rtsp)
    else:
        cap = cv2.VideoCapture(int(device) if device is not None else 0, cv2.CAP_DSHOW)
    return cap


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument("--imou-config", type=str, help="Path to camera_config.json")
    group.add_argument("--rtsp", type=str, help="RTSP URL")
    group.add_argument("--device", type=int, help="Webcam index")

    parser.add_argument("--intrinsics", type=str, required=True, help="Path to intrinsics JSON")
    parser.add_argument("--no-crop", action="store_true", help="Disable ROI crop after undistort")

    args = parser.parse_args()

    undistorter = maybe_make_undistorter(args.intrinsics)
    if undistorter is None:
        raise SystemExit("Missing --intrinsics")

    # Override crop if requested
    if args.no_crop:
        undistorter.intr.crop = False

    base_dir = Path(__file__).resolve().parent

    cap = None
    camera = None

    if args.imou_config:
        cfg_path = Path(args.imou_config)
        if not cfg_path.is_absolute():
            cfg_path = base_dir / cfg_path
        cfg = load_camera_config(str(cfg_path))
        camera = ImouCamera(cfg)
        if not camera.connect():
            raise SystemExit("Failed to connect Imou camera")
        cap = camera.cap
    else:
        rtsp = args.rtsp
        if rtsp is None and args.device is None:
            # default to device 0
            rtsp = None
        cap = open_capture(args.device, rtsp)

    if cap is None or not cap.isOpened():
        raise SystemExit("Failed to open video source")

    side_by_side = True

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            undist = undistorter(frame)

            if side_by_side:
                # Resize undist to original height for comparison
                h = frame.shape[0]
                if undist.shape[0] != h:
                    scale = h / max(1, undist.shape[0])
                    undist = cv2.resize(undist, (int(undist.shape[1] * scale), h))
                vis = cv2.hconcat([frame, undist])
            else:
                vis = undist

            cv2.imshow("Undistort preview (q quit, t toggle)", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("t"):
                side_by_side = not side_by_side
    finally:
        if camera is not None:
            camera.disconnect()
        else:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
