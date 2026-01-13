"""Web-based entry point for the omni-wheel robot simulator (HTML/CSS frontend)."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Dict

from flask import Flask, jsonify, request, send_from_directory

from config import settings
from src.http_input import HttpInputState
from src.robot import Robot
from src import kinematics
from visualization.trajectory import TrajectoryTracker
from utils.math_utils import normalize_angle


BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "web" / "static"

app = Flask(__name__, static_folder=str(STATIC_DIR), static_url_path="/static")

robot = Robot(start_pos=(0.0, 0.0))
trajectory = TrajectoryTracker(max_length=2000)
input_state = HttpInputState()
stop_event = threading.Event()


def simulation_loop() -> None:
	"""Background loop stepping the robot physics."""
	while not stop_event.is_set():
		override = input_state.get_override_omegas()
		if override is not None:
			for wheel, omega in zip(robot.wheels, override):
				wheel.speed = omega
			wheel_speeds = [w.speed for w in robot.wheels]
			wheel_angles = [w.angle for w in robot.wheels]
			vx, vy, omega = kinematics.forward_kinematics(wheel_speeds, wheel_angles, settings.ROBOT_RADIUS)
			gx, gy = kinematics.rotate_vector(vx, vy, robot.orientation)
			robot.position[0] += gx * settings.DT
			robot.position[1] += gy * settings.DT
			robot.orientation = normalize_angle(robot.orientation + omega * settings.DT)
		else:
			commands = input_state.get_wheel_commands()
			robot.update(settings.DT, commands)
		trajectory.add_position(robot.position[0], robot.position[1])
		time.sleep(settings.DT)


@app.route("/")
def index():
	return send_from_directory(STATIC_DIR, "index.html")


@app.post("/input")
def handle_input():
	payload: Dict[str, object] = request.get_json(force=True, silent=True) or {}
	key = str(payload.get("key", "")).lower()
	pressed = bool(payload.get("pressed", False))
	input_state.set_key_state(key, pressed)
	return {"ok": True}


@app.post("/reset")
def reset():
	robot.reset()
	trajectory.clear()
	input_state.reset()
	trajectory.add_position(robot.position[0], robot.position[1])
	return {"ok": True}


@app.post("/pulses")
def set_pulses():
	payload: Dict[str, object] = request.get_json(force=True, silent=True) or {}
	pulses = payload.get("pulses", [])
	microsteps = int(payload.get("microsteps", 16))
	if not isinstance(pulses, list) or len(pulses) != 3:
		return {"ok": False, "error": "pulses must be list of 3"}, 400
	try:
		pulse_vals = [float(p) for p in pulses]
	except Exception:
		return {"ok": False, "error": "pulses must be numeric"}, 400
	omegas = kinematics.pulses_to_wheel_omegas(pulse_vals, microsteps=microsteps)
	input_state.set_override_omegas(omegas)
	return {"ok": True}


@app.get("/state")
def state():
	gx, gy, omega = robot.get_global_velocity()
	wheel_positions = robot.get_wheel_positions()
	xs, ys = trajectory.get_trajectory()
	return jsonify(
		{
			"position": {"x": robot.position[0], "y": robot.position[1]},
			"orientation": robot.orientation,
			"velocity": {"x": gx, "y": gy, "omega": omega},
			"wheel_positions": [{"x": x, "y": y} for x, y in wheel_positions],
			"wheel_speeds": [w.speed for w in robot.wheels],
			"trajectory": {"x": xs, "y": ys},
			"grid_size": settings.GRID_SIZE,
			"robot_radius": settings.ROBOT_RADIUS,
		}
	)


def main() -> None:
	trajectory.add_position(robot.position[0], robot.position[1])
	threading.Thread(target=simulation_loop, daemon=True).start()
	app.run(host="0.0.0.0", port=5000, debug=False, use_reloader=False)
	stop_event.set()


if __name__ == "__main__":
	main()
