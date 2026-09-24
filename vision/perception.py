"""Object detection + per-object distance from the Orbbec Astra Pro (RGB + registered depth).

Pipeline, one cycle at <= --fps (default 6 Hz):
  newest RGB frame + nearest-in-time depth frame (refused if out of sync or stale)
  -> YOLO on the RGB -> for each bbox, distance + 3D position from the registered depth
  -> draw + log.
Both cameras are read on background threads (frame_grabber.py), so a USB hiccup while the camera
is moved fast shows up as a NO DEPTH / NO COLOR / OUT OF SYNC banner, never as a frozen program.

Run:
  python vision/perception.py                  # --device auto: CUDA (fp16) if available, else CPU
  python vision/perception.py --device cpu
  python vision/perception.py --headless       # no window (Jetson over SSH), logs only, Ctrl+C stops

Keys (window focused):
  q / ESC   quit
  s         save RGB, registered depth, annotated view and detections JSON to vision/captures/
  o         toggle the depth overlay on the RGB, to check the D2C registration by eye
"""
import argparse
import json
import logging
import os
import sys
import time
from datetime import datetime

import cv2

import depth_source
from color_source import ColorSource
from depth_source import DepthSource, resolve_redist_path
from detector import DEFAULT_CONF, DEFAULT_IMGSZ, DEFAULT_MODEL, Detector, select_device
from drawing import format_range, render_view
from frame_grabber import pair_frames
from object_distance import intrinsics_from_fov, load_intrinsics, measure_object

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CAPTURES_DIR = os.path.join(BASE_DIR, "captures")
DEFAULT_FPS = 6.0
MAX_SKEW_S = 0.040     # RGB/depth further apart than this are not paired (~1 frame period @30 fps)
MAX_AGE_S = 0.5        # a source whose newest frame is older than this counts as lost
LOG_INTERVAL_S = 1.0
HZ_SMOOTHING = 0.2     # EMA weight of the newest cycle in the displayed rate
WINDOW = "Perception"
KEY_ESC = 27

logger = logging.getLogger(__name__)


def _summary(results):
    if not results:
        return "no objects"
    return ", ".join(f"{d.name} {d.conf:.2f} {format_range(r)}" for d, r in results)


def _records(results):
    return [{
        "class": d.name, "class_id": d.class_id, "confidence": round(d.conf, 3),
        "box": [round(v, 1) for v in d.box],
        "range_m": round(r.range_m, 3) if r else None,
        "z_m": round(r.z_m, 3) if r else None,
        "xyz_cam": [round(v, 3) for v in r.xyz] if r else None,
        "valid_fraction": round(r.valid_fraction, 3) if r else None,
    } for d, r in results]


def _save(color, depth, canvas, results):
    os.makedirs(CAPTURES_DIR, exist_ok=True)
    stem = os.path.join(CAPTURES_DIR, datetime.now().strftime("%Y%m%d_%H%M%S"))
    cv2.imwrite(stem + "_color.png", color.data)
    cv2.imwrite(stem + "_depth_mm.png", depth.data)   # 16-bit PNG, registered, raw millimeters
    cv2.imwrite(stem + "_annotated.png", canvas)
    with open(stem + "_detections.json", "w", encoding="utf-8") as f:
        json.dump(_records(results), f, indent=2)
    logger.info("saved %s_* | %s", stem, _summary(results))


def _run(detector, device, depth_src, color_src, intrinsics, fps, headless):
    period = 1.0 / fps
    show_overlay = False
    last_status = None
    last_log_t = 0.0
    hz = 0.0
    prev_t0 = None
    while True:
        t0 = time.monotonic()
        if prev_t0 is not None:
            dt = t0 - prev_t0
            hz = 1.0 / dt if hz == 0.0 else (1 - HZ_SMOOTHING) * hz + HZ_SMOOTHING / dt
        prev_t0 = t0

        if intrinsics is None and depth_src.fov is not None:
            intrinsics = intrinsics_from_fov(depth_source.WIDTH, depth_source.HEIGHT, *depth_src.fov)
            logger.warning("no --intrinsics file: using the OpenNI2 FOV (fx=%.1f fy=%.1f). Z is measured "
                           "directly; lateral X/Y may be off by a few %%", intrinsics.fx, intrinsics.fy)

        color = color_src.latest()
        pair, status = pair_frames(color, depth_src.frames(), t0, MAX_SKEW_S, MAX_AGE_S)
        depth = None
        results = []
        infer_ms = 0.0
        if pair is not None and intrinsics is not None:
            color, depth = pair
            t_infer = time.monotonic()
            detections = detector.detect(color.data)
            infer_ms = (time.monotonic() - t_infer) * 1000.0
            results = [(d, measure_object(depth.data, d.box, intrinsics)) for d in detections]

        if status != last_status:
            (logger.info if status == "OK" else logger.warning)("state: %s", status)
            last_status = status
        if t0 - last_log_t >= LOG_INTERVAL_S:
            logger.info("%.1f Hz | infer %.0f ms | %s", hz, infer_ms,
                        _summary(results) if status == "OK" else status)
            last_log_t = t0

        remaining_s = period - (time.monotonic() - t0)
        if headless:
            time.sleep(max(remaining_s, 0.0))
            continue
        canvas = render_view(color, depth, results, status,
                             f"{hz:4.1f} Hz | infer {infer_ms:3.0f} ms | {device}", show_overlay)
        cv2.imshow(WINDOW, canvas)
        key = cv2.waitKey(max(int(remaining_s * 1000), 1)) & 0xFF
        # Closing the window with its X button must quit too, or imshow silently reopens it.
        if key in (ord("q"), KEY_ESC) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            return
        if key == ord("o"):
            show_overlay = not show_overlay
        elif key == ord("s"):
            if depth is not None:
                _save(color, depth, canvas, results)
            else:
                logger.warning("nothing saved: %s", status)


def _parse_args():
    parser = argparse.ArgumentParser(description="Object detection + distance, Orbbec Astra Pro.")
    parser.add_argument("--device", default="auto", choices=("auto", "cpu", "cuda"),
                        help="auto = CUDA if torch sees a GPU, else CPU")
    parser.add_argument("--model", default=DEFAULT_MODEL, help="YOLO weights (default: %(default)s)")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF, help="min detection confidence")
    parser.add_argument("--imgsz", type=int, default=DEFAULT_IMGSZ, help="YOLO input size")
    parser.add_argument("--fps", type=float, default=DEFAULT_FPS, help="max processing rate, Hz")
    parser.add_argument("--color-index", type=int, default=0, help="OpenCV index of the Astra RGB camera")
    parser.add_argument("--intrinsics", help="JSON {fx, fy, cx, cy} of the RGB camera at 640x480")
    parser.add_argument("--headless", action="store_true", help="no window, logs only")
    return parser.parse_args()


def main():
    args = _parse_args()
    headless = args.headless or (sys.platform != "win32" and not os.environ.get("DISPLAY"))
    if headless and not args.headless:
        logger.info("no DISPLAY: running headless")
    redist_path = resolve_redist_path()
    device = select_device(args.device)
    intrinsics = load_intrinsics(args.intrinsics) if args.intrinsics else None
    detector = Detector(args.model, device, args.conf, args.imgsz)
    depth_src = DepthSource(redist_path)
    color_src = ColorSource(args.color_index)
    depth_src.start()
    color_src.start()
    clean = True
    try:
        _run(detector, device, depth_src, color_src, intrinsics, args.fps, headless)
    except KeyboardInterrupt:
        pass
    finally:
        clean = color_src.close()
        clean = depth_src.close() and clean
        if not headless:
            cv2.destroyAllWindows()
    if not clean:
        # A grabber is stuck inside a driver call; a normal interpreter exit could wait on it.
        logger.warning("exiting hard: a camera thread did not stop")
        logging.shutdown()
        os._exit(0)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    main()
