"""
Perception - Camera Interface

Giao tiếp với camera (IMOU RTSP, USB camera, etc.)
Tham chiếu: Vision/imou_camera_capture.py
"""

import cv2


class CameraInterface:
    """Interface quản lý kết nối camera."""

    def __init__(self, source=0, config_path: str = None):
        """
        Args:
            source: Camera index (int) hoặc RTSP URL (str)
            config_path: Đường dẫn file config camera (JSON)
        """
        self.source = source
        self.config_path = config_path
        self.cap = None

    def connect(self) -> bool:
        """Kết nối đến camera."""
        self.cap = cv2.VideoCapture(self.source)
        return self.cap.isOpened()

    def read_frame(self):
        """Đọc 1 frame từ camera."""
        if self.cap is None or not self.cap.isOpened():
            return None
        ret, frame = self.cap.read()
        return frame if ret else None

    def release(self):
        """Giải phóng camera."""
        if self.cap:
            self.cap.release()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()
