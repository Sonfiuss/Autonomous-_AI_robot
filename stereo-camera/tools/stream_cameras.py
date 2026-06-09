#!/usr/bin/env python3
"""
stream_cameras.py  –  MJPEG HTTP stream cho stereo camera pair
Chạy trên Jetson (qua SSH), sau đó mở browser trên máy local:
  http://<jetson-ip>:8080

Cameras: /dev/video0 (left), /dev/video2 (right)
"""

import cv2
import numpy as np
import threading
import time
import socket
from http.server import BaseHTTPRequestHandler, HTTPServer

# ── Cấu hình ──────────────────────────────────────────────────────────────────
LEFT_IDX  = 0      # /dev/video0
RIGHT_IDX = 2      # /dev/video2
WIDTH     = 640
HEIGHT    = 480
FPS_CAP   = 30
PORT      = 8080
JPEG_QUAL = 80

# ── Shared state ──────────────────────────────────────────────────────────────
_frame_lock  = threading.Lock()
_frame_jpeg  = None
_stats       = {"fps": 0.0, "left_ok": False, "right_ok": False}


def capture_loop():
    cap_l = cv2.VideoCapture(LEFT_IDX)
    cap_r = cv2.VideoCapture(RIGHT_IDX)
    for cap in (cap_l, cap_r):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, FPS_CAP)

    t0, count = time.time(), 0

    while True:
        ok_l, frame_l = cap_l.read()
        ok_r, frame_r = cap_r.read()

        _stats["left_ok"]  = ok_l
        _stats["right_ok"] = ok_r

        if not ok_l:
            frame_l = _error_frame("LEFT FAIL  /dev/video0")
        if not ok_r:
            frame_r = _error_frame("RIGHT FAIL /dev/video2")

        # FPS
        count += 1
        dt = time.time() - t0
        if dt >= 1.0:
            _stats["fps"] = count / dt
            count, t0 = 0, time.time()

        _overlay(frame_l, "LEFT  /dev/video0")
        _overlay(frame_r, "RIGHT /dev/video2")

        combined = np.hstack([frame_l, frame_r])
        _, jpg = cv2.imencode(".jpg", combined,
                              [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUAL])

        with _frame_lock:
            global _frame_jpeg
            _frame_jpeg = jpg.tobytes()


def _error_frame(msg: str) -> np.ndarray:
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    cv2.putText(img, msg, (20, HEIGHT // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return img


def _overlay(img: np.ndarray, label: str):
    fps_txt = f"{_stats['fps']:.1f} fps"
    # Epipolar guide line
    cy = HEIGHT // 2
    cv2.line(img, (0, cy), (WIDTH, cy), (0, 255, 255), 1)
    # Labels
    cv2.putText(img, label,   (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
    cv2.putText(img, fps_txt, (10, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)


# ── HTML page ─────────────────────────────────────────────────────────────────
_HTML = """\
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>Stereo Camera Stream</title>
  <style>
    body  {{ background:#111; color:#ddd; font-family:monospace;
             display:flex; flex-direction:column; align-items:center; }}
    h2    {{ margin:12px 0 4px; }}
    p     {{ margin:2px 0 10px; font-size:.85em; color:#aaa; }}
    img   {{ max-width:100%; border:2px solid #333; }}
  </style>
</head>
<body>
  <h2>Stereo Camera Stream</h2>
  <p>LEFT: /dev/video0 &nbsp;|&nbsp; RIGHT: /dev/video2 &nbsp;|&nbsp;
     {w}&times;{h} &nbsp;|&nbsp; JPEG q={q}</p>
  <img src="/stream" alt="stream"/>
  <p style="margin-top:8px">Đường vàng ngang giữa = epipolar line (dùng để căn chỉnh)</p>
</body>
</html>""".format(w=WIDTH, h=HEIGHT, q=JPEG_QUAL)


# ── HTTP handler ──────────────────────────────────────────────────────────────
class _Handler(BaseHTTPRequestHandler):
    def log_message(self, *_): pass   # tắt log mỗi request

    def do_GET(self):
        if self.path == "/":
            body = _HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _frame_lock:
                        jpg = _frame_jpeg
                    if jpg:
                        hdr = (b"--frame\r\n"
                               b"Content-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(hdr + jpg + b"\r\n")
                    time.sleep(1.0 / FPS_CAP)
            except (BrokenPipeError, ConnectionResetError):
                pass

        else:
            self.send_response(404)
            self.end_headers()


# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Lấy IP thực của Jetson (bỏ loopback)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        jetson_ip = s.getsockname()[0]
        s.close()
    except Exception:
        jetson_ip = "127.0.0.1"

    print(f"[stream_cameras] Cameras : /dev/video{LEFT_IDX} (left) | "
          f"/dev/video{RIGHT_IDX} (right)")
    print(f"[stream_cameras] Khởi động capture thread ...")

    t = threading.Thread(target=capture_loop, daemon=True)
    t.start()

    # Đợi frame đầu tiên
    timeout = time.time() + 5.0
    while _frame_jpeg is None and time.time() < timeout:
        time.sleep(0.05)
    if _frame_jpeg is None:
        print("[stream_cameras] Cảnh báo: chưa có frame sau 5s, kiểm tra camera.")

    print(f"[stream_cameras] HTTP server sẵn sàng trên cổng {PORT}")
    print(f"[stream_cameras] >>> Mở browser: http://{jetson_ip}:{PORT} <<<")
    print(f"[stream_cameras] Nhấn Ctrl+C để dừng.\n")

    server = HTTPServer(("0.0.0.0", PORT), _Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[stream_cameras] Đã dừng.")
