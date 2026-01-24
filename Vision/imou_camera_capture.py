"""Simple RTSP viewer for Imou cameras."""

import json
import os
import time
from dataclasses import dataclass

import cv2


@dataclass
class CameraConfig:
    ip_address: str
    username: str
    password: str
    rtsp_port: int = 554


class ImouCamera:
    def __init__(self, config: CameraConfig):
        self.config = config
        self.rtsp_url = (
            f"rtsp://{config.username}:{config.password}@"
            f"{config.ip_address}:{config.rtsp_port}/cam/realmonitor?channel=1&subtype=0"
        )
        self.alternative_urls = [
            f"rtsp://{config.username}:{config.password}@{config.ip_address}:{config.rtsp_port}/stream1",
            f"rtsp://{config.username}:{config.password}@{config.ip_address}:{config.rtsp_port}/stream2",
            f"rtsp://{config.username}:{config.password}@{config.ip_address}:{config.rtsp_port}/live",
            f"rtsp://{config.username}:{config.password}@{config.ip_address}:{config.rtsp_port}/Streaming/Channels/101",
            f"rtsp://{config.username}:{config.password}@{config.ip_address}:{config.rtsp_port}/11",
        ]
        self.cap = None

    def connect(self, timeout: int = 15) -> bool:
        urls = [self.rtsp_url] + self.alternative_urls
        deadline = time.time() + timeout

        for url in urls:
            if time.time() >= deadline:
                break

            self.cap = cv2.VideoCapture(url)

            while time.time() < deadline:
                if self.cap.isOpened():
                    self.rtsp_url = url
                    return True
                time.sleep(0.2)

            self.cap.release()

        return False

    def live_view(self):
        if self.cap is None or not self.cap.isOpened():
            return

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            height, width = frame.shape[:2]
            cv2.putText(
                frame,
                f"Imou Camera - {self.config.ip_address}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                frame,
                f"Resolution: {width}x{height}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )
            cv2.imshow("Imou Camera Live View", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        cv2.destroyAllWindows()

    def disconnect(self):
        if self.cap is not None:
            self.cap.release()


def load_camera_config(path: str) -> CameraConfig:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return CameraConfig(
        ip_address=data["ip_address"],
        username=data["username"],
        password=data["password"],
        rtsp_port=int(data.get("rtsp_port", 554)),
    )


def main():
    config_path = os.path.join(os.path.dirname(__file__), "camera_config.json")
    config = load_camera_config(config_path)

    camera = ImouCamera(config)
    if not camera.connect():
        return

    try:
        camera.live_view()
    finally:
        camera.disconnect()


if __name__ == "__main__":
    main()
