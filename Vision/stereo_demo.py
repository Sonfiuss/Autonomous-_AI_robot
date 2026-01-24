"""Stereo demo: Imou RTSP (left) + laptop camera (right) using OpenCV SGBM disparity.

Notes on mounting: baseline ~8–25 cm for indoor 1–4 m range; mount both cameras rigidly,
parallel, ~0.4–1.2 m high with slight downward tilt if looking at the floor.
"""

import json
import os
from dataclasses import dataclass
from typing import Optional

import cv2


@dataclass
class CameraSource:
    source_type: str  # "rtsp", "device", or "imou_config"
    url: Optional[str] = None
    index: Optional[int] = None
    width: Optional[int] = None
    height: Optional[int] = None
    config_path: Optional[str] = None  # used when source_type == "imou_config"
    backend: Optional[str] = None  # e.g., "dshow" on Windows for webcams


def load_sources(config_path: str) -> tuple[CameraSource, CameraSource]:
    with open(config_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    left = CameraSource(**data["left"])
    right = CameraSource(**data["right"])
    return left, right


def load_imou_url(config_path: str) -> str:
    with open(config_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    ip = data["ip_address"]
    user = data["username"]
    pwd = data["password"]
    port = int(data.get("rtsp_port", 554))
    return (
        f"rtsp://{user}:{pwd}@{ip}:{port}/cam/realmonitor?channel=1&subtype=0"
    )


def build_sgbm() -> cv2.StereoSGBM:
    # Reasonable defaults; tune for your cameras.
    return cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=16 * 6,  # must be divisible by 16
        blockSize=5,
        P1=8 * 3 * 5 ** 2,
        P2=32 * 3 * 5 ** 2,
        disp12MaxDiff=1,
        uniquenessRatio=10,
        speckleWindowSize=50,
        speckleRange=2,
    )


def normalize_disparity(disparity):
    disp = disparity.astype("float32") / 16.0
    disp[disp < 0] = 0
    disp = cv2.normalize(disp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    return disp.astype("uint8")


def make_same_size(a, b):
    h = min(a.shape[0], b.shape[0])
    w = min(a.shape[1], b.shape[1])
    if a.shape[0] != h or a.shape[1] != w:
        a = cv2.resize(a, (w, h))
    if b.shape[0] != h or b.shape[1] != w:
        b = cv2.resize(b, (w, h))
    return a, b


def open_capture(src: CameraSource, base_dir: str) -> cv2.VideoCapture:
    if src.source_type == "rtsp":
        target = src.url
    elif src.source_type == "imou_config":
        cfg_path = src.config_path or "camera_config.json"
        if not os.path.isabs(cfg_path):
            cfg_path = os.path.join(base_dir, cfg_path)
        target = load_imou_url(cfg_path)
    else:
        target = src.index

    backend = 0
    if src.backend:
        if src.backend.lower() == "dshow":
            backend = cv2.CAP_DSHOW
        elif src.backend.lower() == "msmf":
            backend = cv2.CAP_MSMF

    cap = cv2.VideoCapture(target, backend)
    if src.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, src.width)
    if src.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, src.height)
    return cap


def main():
    base_dir = os.path.dirname(__file__)
    config_path = os.path.join(base_dir, "stereo_config.json")
    left_src, right_src = load_sources(config_path)

    cap_left = open_capture(left_src, base_dir)
    cap_right = open_capture(right_src, base_dir)

    if not cap_left.isOpened() or not cap_right.isOpened():
        print("Failed to open one or both sources.")
        return

    sgbm = build_sgbm()

    try:
        while True:
            ok_l, frame_l = cap_left.read()
            ok_r, frame_r = cap_right.read()
            if not ok_l or not ok_r:
                print("Frame grab failed; exiting.")
                break

            frame_l, frame_r = make_same_size(frame_l, frame_r)

            gray_l = cv2.cvtColor(frame_l, cv2.COLOR_BGR2GRAY)
            gray_r = cv2.cvtColor(frame_r, cv2.COLOR_BGR2GRAY)

            disparity = sgbm.compute(gray_l, gray_r)
            disp_vis = normalize_disparity(disparity)
            disp_vis = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

            top = cv2.hconcat([frame_l, frame_r])
            bottom = cv2.resize(disp_vis, (top.shape[1], frame_l.shape[0]))
            stacked = cv2.vconcat([top, bottom])

            cv2.imshow("Stereo + Disparity (q to quit)", stacked)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap_left.release()
        cap_right.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
