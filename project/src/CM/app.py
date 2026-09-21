"""Flask server for the LLM grounding UI.

Run:  python app.py                        -> http://localhost:5001   (provider from CM_PROVIDER)
      CM_PROVIDER=fake python app.py       -> no API key, replies come from a scripted stub
Endpoints:
  GET  /                                    web page
  GET  /api/scene/new?seed=N                new scene (shared scenes/ dir with simulation/room) + candidates
  GET  /api/scene/latest                    last scene + candidates
  POST /api/ground/say   {"text": ...}      one dialog turn -> {type, text, slots, matches, resolved}
  POST /api/ground/reset                    clear slots + history
  GET  /api/prompt                          the prompt JSON currently loaded
  POST /api/plan         {"goal": {...}}    MV A* plan -> {ok, path, waypoints, primitives, length_m}
  POST /api/trajectory   {"goal": {...}}    MV plan + MC executor -> plan plus per-tick wheel speeds
"""
import json
import logging
import os
import sys

from flask import Flask, jsonify, request, send_from_directory

import config
import mc_client
import mv_client
from dialog import GroundingSession, load_prompt
from llm_client import FakeClient, make_client

sys.path.insert(0, config.ROOM_DIR)
from room_generator import generate  # noqa: E402

logger = logging.getLogger(__name__)
app = Flask(__name__, static_folder=config.STATIC_DIR, static_url_path="/static")

LATEST_FILE = os.path.join(config.SCENES_DIR, "latest.json")
FAKE_REPLIES = [                                  # CM_PROVIDER=fake demo script (repeats)
    {"type": "ask", "question": "Which side?", "slots": {"target": "obj_1", "side": None, "spot": None}, "matches": ["obj_1:*"]},
    {"type": "goal", "question": "", "slots": {"target": "obj_1", "side": None, "spot": None}, "matches": []},
]

_state = {"session": None}


class LoopingFake(FakeClient):
    def complete(self, system, messages, schema):
        if not self.replies:
            self.replies = [dict(r) for r in FAKE_REPLIES]
        return super().complete(system, messages, schema)


def _make_llm():
    if config.PROVIDER == "fake":
        return LoopingFake()
    return make_client(config.PROVIDER)


def _save_scene(scene):
    os.makedirs(config.SCENES_DIR, exist_ok=True)
    text = json.dumps(scene, indent=2)
    for path in (os.path.join(config.SCENES_DIR, f"room_{scene['seed']}.json"), LATEST_FILE):
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)


def _load_latest():
    try:
        with open(LATEST_FILE, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _start(scene):
    _state["session"] = GroundingSession(scene, _make_llm(), load_prompt())
    return _state["session"]


def _payload(session):
    return {"scene": session.scene, "candidates": session.spots, "slots": session.slots,
            "provider": config.PROVIDER}


@app.route("/")
def index():
    return send_from_directory(config.STATIC_DIR, "index.html")


@app.route("/api/scene/new")
def api_scene_new():
    seed_arg = request.args.get("seed", "").strip()
    scene = generate(int(seed_arg) if seed_arg.isdigit() else None)
    _save_scene(scene)
    return jsonify(_payload(_start(scene)))


@app.route("/api/scene/latest")
def api_scene_latest():
    if _state["session"] is None:
        scene = _load_latest() or generate(None)
        _save_scene(scene)
        _start(scene)
    return jsonify(_payload(_state["session"]))


@app.route("/api/ground/say", methods=["POST"])
def api_say():
    session = _state["session"]
    if session is None:
        return jsonify({"error": "no scene loaded"}), 400
    text = (request.get_json(silent=True) or {}).get("text", "").strip()
    if not text:
        return jsonify({"error": "empty text"}), 400
    logger.info("user: %s", text)
    resp = session.say(text)
    logger.info("-> %s %s matches=%d", resp["type"], resp["slots"], len(resp["matches"]))
    return jsonify(resp)


@app.route("/api/ground/reset", methods=["POST"])
def api_reset():
    if _state["session"] is not None:
        _state["session"].reset()
    return jsonify({"ok": True})


@app.route("/api/prompt")
def api_prompt():
    return jsonify(load_prompt())


def _plan_for(session, body):
    """Runs MV for the goal in `body` (or the dialog's own). Returns (plan, goal, error_response)."""
    goal = body.get("goal") or session.resolved_goal()
    if not goal:
        return None, None, (jsonify({"ok": False, "reason": "no goal: resolve a target first"}), 400)
    try:
        plan = mv_client.plan_path(session.scene, goal, start=body.get("start"),
                                   holonomic=bool(body.get("holonomic", config.MV_HOLONOMIC)))
    except mv_client.MvError as exc:                  # library missing / unloadable / bad scene
        logger.error("MV unavailable: %s", exc)
        return None, None, (jsonify({"ok": False, "reason": str(exc)}), 503)
    plan["goal"] = goal
    return plan, goal, None


@app.route("/api/plan", methods=["POST"])
def api_plan():
    """Plans the robot pose -> goal with MV (L4). Goal defaults to the session's resolved match."""
    session = _state["session"]
    if session is None:
        return jsonify({"error": "no scene loaded"}), 400
    body = request.get_json(silent=True) or {}
    result, _goal, error = _plan_for(session, body)
    if error is not None:
        return error
    logger.info("plan -> ok=%s len=%s waypoints=%d", result["ok"], result.get("length_m"),
                len(result.get("waypoints", [])))
    return jsonify(result)


@app.route("/api/trajectory", methods=["POST"])
def api_trajectory():
    """Plans with MV, then expands the primitives into per-tick wheel speeds with MC (L4 -> L5)."""
    session = _state["session"]
    if session is None:
        return jsonify({"error": "no scene loaded"}), 400
    body = request.get_json(silent=True) or {}
    plan, _goal, error = _plan_for(session, body)
    if error is not None:
        return error
    if not plan["ok"]:
        return jsonify(plan)                          # unreachable goal: nothing to execute
    start = body.get("start") or session.scene.get("robot") or {}
    try:
        trajectory = mc_client.run(plan["primitives"],
                                   start_theta=float(start.get("theta", 0.0)),
                                   cruise_speed=float(body.get("cruise_speed", 0.0)),
                                   yaw_rate=float(body.get("yaw_rate", 0.0)),
                                   dt=float(body.get("dt", 0.0)),
                                   max_rows=config.MC_UI_MAX_ROWS)
    except mc_client.McError as exc:                  # library missing / unloadable / bad primitive
        logger.error("MC unavailable: %s", exc)
        # MV already found a route: keep it so the UI still draws the path, and
        # report the executor failure alongside it instead of losing both.
        trajectory = {"ok": False, "reason": str(exc)}
    plan["trajectory"] = trajectory
    # The trajectory starts at the snapped cell, not the raw robot pose: give the UI that origin.
    plan["origin"] = {"x": plan["snapped_start"]["x"], "y": plan["snapped_start"]["y"],
                      "theta": float(start.get("theta", 0.0))}
    logger.info("trajectory -> ok=%s steps=%s duration=%s", trajectory["ok"],
                trajectory.get("step_count"), trajectory.get("duration_s"))
    return jsonify(plan)



if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("provider=%s", config.PROVIDER)
    app.run(host="0.0.0.0", port=config.PORT, debug=False)
