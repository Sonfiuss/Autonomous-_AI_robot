"""
Flask server for Robot Path Planning Simulator
với ZMQ bridge kết nối tới Jetson motivation module.

Biến môi trường:
  JETSON_IP   IP của Jetson (default: localhost nếu chạy trên cùng Jetson)
  GOAL_PORT   ZMQ port nhận goal      (default: 5555)
  ODOM_PORT   ZMQ port nhận odometry  (default: 5556)

Chạy:
  JETSON_IP=192.168.1.100 python app.py
"""

import os
import threading
import time
from flask import Flask, render_template, jsonify, request, Response
import zmq

app = Flask(__name__)

# ── Config ────────────────────────────────────────────────────────────────────
JETSON_IP  = os.environ.get("JETSON_IP",  "localhost")
GOAL_PORT  = int(os.environ.get("GOAL_PORT",  5555))
ODOM_PORT  = int(os.environ.get("ODOM_PORT",  5556))

# Scale của simulation: 1 ô grid = 20 cm, scale = 2 → world coords tính bằng cm
# Khi gửi sang Jetson cần đổi cm → m
CM_TO_M = 0.01

DEFAULT_CONFIG = {
    "robot": {
        "outer_radius": 19.0,
        "inner_radius": 14.4,
        "max_velocity": 1.0,
        "max_omega": 1.0,
        "safety_margin": 2.0
    },
    "grid": {
        "width": 800,
        "height": 600,
        "cell_size": 20,
        "scale": 2.0
    },
    "simulation": {
        "update_rate": 60,
        "path_color": "#00ff00",
        "robot_color": "#3498db",
        "obstacle_color": "#e74c3c"
    }
}

# ── ZMQ Bridge ────────────────────────────────────────────────────────────────
class RobotBridge:
    def __init__(self):
        self._ctx      = None
        self._pub      = None   # gửi goal tới Jetson
        self._sub      = None   # nhận odom từ Jetson
        self._pose     = {"x": 0.0, "y": 0.0, "theta": 0.0, "ts": 0.0}
        self._lock     = threading.Lock()
        self._connected = False
        self._thread   = None
        self._running  = False

    def start(self):
        try:
            self._ctx = zmq.Context()

            self._pub = self._ctx.socket(zmq.PUB)
            self._pub.connect(f"tcp://{JETSON_IP}:{GOAL_PORT}")

            self._sub = self._ctx.socket(zmq.SUB)
            self._sub.connect(f"tcp://{JETSON_IP}:{ODOM_PORT}")
            self._sub.setsockopt(zmq.SUBSCRIBE, b"")
            self._sub.setsockopt(zmq.RCVTIMEO, 200)

            time.sleep(0.4)   # ZMQ warm-up
            self._running   = True
            self._connected = True
            self._thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._thread.start()
            print(f"[Bridge] Connected to Jetson at {JETSON_IP}")
        except Exception as e:
            print(f"[Bridge] Start failed: {e}")
            self._connected = False

    def send_goal(self, x_m: float, y_m: float):
        if not self._pub:
            return False
        msg = f"G {x_m:.4f} {y_m:.4f} 0.0"
        self._pub.send_string(msg)
        print(f"[Bridge] Goal → x={x_m:.3f}m  y={y_m:.3f}m")
        return True

    def get_pose(self):
        with self._lock:
            return dict(self._pose)

    @property
    def connected(self):
        return self._connected

    def _rx_loop(self):
        while self._running:
            try:
                msg = self._sub.recv_string()
                if msg.startswith("O "):
                    parts = msg.split()
                    if len(parts) == 4:
                        with self._lock:
                            self._pose = {
                                "x":     float(parts[1]),
                                "y":     float(parts[2]),
                                "theta": float(parts[3]),
                                "ts":    time.monotonic()
                            }
            except zmq.Again:
                continue
            except Exception as e:
                print(f"[Bridge] RX error: {e}")


bridge = RobotBridge()
bridge.start()

# ── Flask routes ──────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html', config=DEFAULT_CONFIG)


@app.route('/api/config', methods=['GET'])
def get_config():
    return jsonify(DEFAULT_CONFIG)


@app.route('/api/config', methods=['POST'])
def update_config():
    data = request.get_json()
    for key in data:
        if key in DEFAULT_CONFIG:
            DEFAULT_CONFIG[key].update(data[key])
    return jsonify({"status": "ok", "config": DEFAULT_CONFIG})


@app.route('/api/pathfind', methods=['POST'])
def calculate_path():
    data  = request.get_json()
    start = data.get('start')
    goal  = data.get('goal')
    return jsonify({
        "status": "ok",
        "path": [start, goal],
        "message": "Direct path"
    })


# ── Real robot API ────────────────────────────────────────────────────────────

@app.route('/api/robot/goal', methods=['POST'])
def send_robot_goal():
    """
    Nhận goal từ simulation JS (tọa độ cm) và gửi tới Jetson (mét).
    Body: { "x": <cm>, "y": <cm> }
    """
    data  = request.get_json()
    x_cm  = float(data.get("x", 0))
    y_cm  = float(data.get("y", 0))
    x_m   = x_cm * CM_TO_M
    y_m   = y_cm * CM_TO_M

    ok = bridge.send_goal(x_m, y_m)
    return jsonify({
        "status": "ok" if ok else "error",
        "goal_m": {"x": x_m, "y": y_m}
    })


@app.route('/api/robot/pose', methods=['GET'])
def get_robot_pose():
    """
    Trả về pose hiện tại của robot thực (từ odometry Jetson).
    JS poll endpoint này để vẽ vị trí robot lên map.
    Response: { "x_cm": .., "y_cm": .., "theta_deg": .., "connected": .. }
    """
    pose = bridge.get_pose()
    return jsonify({
        "x_cm":      pose["x"] / CM_TO_M,
        "y_cm":      pose["y"] / CM_TO_M,
        "theta_deg": pose["theta"],
        "connected": bridge.connected,
        "ts":        pose["ts"]
    })


@app.route('/api/robot/stop', methods=['POST'])
def stop_robot():
    if bridge._pub:
        bridge._pub.send_string("S")
    return jsonify({"status": "ok"})


if __name__ == '__main__':
    app.run(debug=False, host='0.0.0.0', port=5000)
