"""
UI Backend - Main Server

Flask server cung cấp REST API và WebSocket cho giao diện điều khiển.
"""

from flask import Flask, render_template, send_from_directory
# from flask_socketio import SocketIO

import os
import sys

# Thêm path để import hardware API
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "hardware"))

app = Flask(
    __name__,
    static_folder=os.path.join(os.path.dirname(__file__), "..", "frontend"),
    template_folder=os.path.join(os.path.dirname(__file__), "..", "frontend"),
)
# socketio = SocketIO(app, cors_allowed_origins="*")


@app.route("/")
def index():
    """Trang chính."""
    return render_template("index.html")


@app.route("/api/status")
def status():
    """Trạng thái hệ thống."""
    return {
        "status": "online",
        "modules": {
            "drive": "disconnected",
            "arm": "disconnected",
            "sensors": "disconnected",
            "camera": "disconnected",
        },
    }


# @socketio.on("control")
# def handle_control(data):
#     """Nhận lệnh điều khiển từ frontend qua WebSocket."""
#     action = data.get("action")
#     params = data.get("params", {})
#     # TODO: Gửi lệnh đến hardware API
#     print(f"Control: {action} - {params}")


if __name__ == "__main__":
    # app.run(host="0.0.0.0", port=5000, debug=True)
    # socketio.run(app, host="0.0.0.0", port=5000, debug=True)
    print("UI Server - chưa cấu hình. Chạy: flask run")
