"""Flask page of the real room - realroom's counterpart of simulation/room/app.py - and the real robot's remote.

Run:  python3 realroom/app.py [--provider rule] [--dry-run]    -> http://<jetson>:5002
Endpoints:
  GET  /                          web page: the real map, the robot, the spots beside objects, a planned path, the chat
  GET  /api/map/latest            latest.json (robot = current pose) + its known grid, cropped to the room
  GET  /api/map/<map_id>          a published map as it was built, + its grid
  GET  /api/map/latest.png[?spot=<id>]   map_image.py's picture of latest.json (+ the path to that spot)
  GET  /api/map/<map_id>.png      map_image.py's picture of a published map as it was built
  GET  /api/map/version           {version}: latest.json's mtime (a string) - it changes with every publish and pose
  GET  /api/spots                 the spots beside objects CM's grounding picks from (candidates.py)
  POST /api/plan  {"spot": id} | {"goal": {x, y, theta}}   planner.plan from the robot's pose -> RoomPlan (view only)
  POST /api/chat  {"text", "compensate"}   drive_chat.ChatSession.say -> {type: ask | plan | none | busy | error, text,
                                  plan}; "chạy" (RUN_WORDS) with a plan -> runs it: {type: run, text, run: /api/run's};
                                  "đi tiếp" (GO_ON_WORDS) -> ChatSession.go_on(the last run's unreached target)
  POST /api/chat/go_on            ChatSession.go_on(RunManager.unreached()): the rest of the way, from the robot's pose
  POST /api/chat/reset            forget the command in progress and the plan not run
  POST /api/run   {"compensate"}  drive the chat's plan (demo_drive --plan-json): 400 no plan, 409 a run is going
  POST /api/run/stop              SIGINT to that run - the terminal's Ctrl-C (robot_link aborts, sends S)
  GET  /api/run/status            drive_chat.RunManager.status
The robot moves only on POST /api/run; demo_drive then moves the pose (and, after "lập bản đồ", publishes the
map) this page shows. The page is open to the LAN: anyone who reaches :5002 can drive the robot.
"""
import argparse
import logging
import math
import os
import sys

from flask import Flask, Response, jsonify, request, send_from_directory

import drive_chat
import map_image
import map_store
import planner
from cmd_parser import LLM_PROVIDERS, PROVIDER_AUTO, PROVIDER_RULE  # communication/, on the path through drive_chat

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
PORT = 5002
DEG_DIGITS, M_DIGITS = 1, 3             # a leg as the page shows it: degrees, metres
REPLY_BUSY = "busy"
BUSY_TEXT = "Robot đang chạy: chờ xong (hoặc bấm Dừng) rồi ra lệnh."
REPLY_RUN = "run"
RUN_STARTED = "Chạy kế hoạch ({run_id})."
COMPENSATE_DEFAULT = True               # POST /api/run without "compensate": as the page's box starts, ticked

sys.path.insert(0, planner.CM_DIR)
from candidates import build_candidates  # noqa: E402  (CM module, path set just above)

logger = logging.getLogger(__name__)
app = Flask(__name__, static_folder=STATIC_DIR, static_url_path="/static")
chat = drive_chat.ChatSession()         # the __main__ block makes both again from its options
runs = drive_chat.RunManager()


def _grid_payload(scene):
    """The known grid cropped to the scene's room box: {res, origin, rows} with rows[iy] a string of
    'o' occupied / '.' free / ' ' unseen, row 0 at the room's y = 0."""
    grid = map_store.load_grid(scene)
    rows, cols = grid.cells.shape
    ix0 = max(int(math.floor(-grid.origin[0] / grid.res)), 0)
    iy0 = max(int(math.floor(-grid.origin[1] / grid.res)), 0)
    ix1 = min(int(math.ceil((scene["room"]["width"] - grid.origin[0]) / grid.res)), cols)
    iy1 = min(int(math.ceil((scene["room"]["length"] - grid.origin[1]) / grid.res)), rows)
    symbol = {map_store.KNOWN_OCCUPIED: "o", map_store.KNOWN_FREE: ".", map_store.KNOWN_UNSEEN: " "}
    lines = ["".join(symbol[int(v)] for v in grid.cells[iy, ix0:ix1]) for iy in range(iy0, iy1)]
    return {"res": grid.res, "origin": [grid.origin[0] + ix0 * grid.res, grid.origin[1] + iy0 * grid.res], "rows": lines}


def _map_payload(scene):
    return {"scene": scene, "grid": _grid_payload(scene)}


@app.route("/")
def index():
    return send_from_directory(STATIC_DIR, "index.html")


@app.route("/api/map/latest")
def api_map_latest():
    scene = map_store.load_latest()
    if scene is None:
        return jsonify({"error": "no map yet: python3 vision/drive_map.py --run vision/output/<run> --publish"}), 404
    return jsonify(_map_payload(scene))


@app.route("/api/map/<map_id>")
def api_map(map_id):
    scene = map_store.load_map(map_id)
    if scene is None:
        return jsonify({"error": f"no map {map_id}"}), 404
    return jsonify(_map_payload(scene))


def _png(scene, plan_path=None):
    """map_image's picture of scene (+ plan_path) as an image/png response."""
    return Response(map_image.png_bytes(scene, map_store.load_grid(scene), plan_path), mimetype="image/png")


@app.route("/api/map/latest.png")
def api_map_latest_png():
    scene = map_store.load_latest()
    if scene is None:
        return jsonify({"error": "no map yet"}), 404
    path = None
    if request.args.get("spot"):
        spot = _spot(scene, request.args["spot"])
        if spot is None:
            return jsonify({"error": f"no spot {request.args['spot']}"}), 400
        result = planner.plan(scene, map_store.load_grid(scene), (spot["x"], spot["y"], spot["theta"]))
        path = result.path if result.ok else None
    return _png(scene, path)


@app.route("/api/map/<map_id>.png")
def api_map_png(map_id):
    scene = map_store.load_map(map_id)
    if scene is None:
        return jsonify({"error": f"no map {map_id}"}), 404
    return _png(scene)


@app.route("/api/spots")
def api_spots():
    scene = map_store.load_latest()
    if scene is None:
        return jsonify({"error": "no map yet"}), 404
    return jsonify(build_candidates(scene))


def _spot(scene, spot_id):
    """The CM candidate spot_id of a scene, or None."""
    return next((s for s in build_candidates(scene) if s["id"] == spot_id), None)


@app.route("/api/plan", methods=["POST"])
def api_plan():
    scene = map_store.load_latest()
    if scene is None:
        return jsonify({"error": "no map yet"}), 404
    body = request.get_json(silent=True) or {}
    if "spot" in body:
        spot = _spot(scene, body["spot"])
        if spot is None:
            return jsonify({"error": f"no spot {body['spot']}"}), 400
        goal = (spot["x"], spot["y"], spot["theta"])
    elif isinstance(body.get("goal"), dict) and "x" in body["goal"] and "y" in body["goal"]:
        goal = (body["goal"]["x"], body["goal"]["y"], body["goal"].get("theta", 0.0))
    else:
        return jsonify({"error": "send {\"spot\": id} or {\"goal\": {x, y, theta}}"}), 400
    result = planner.plan(scene, map_store.load_grid(scene), goal)
    payload = result._asdict()
    payload["legs"] = [{"kind": kind, "value": value, "shown": f"{kind} " + str(
        round(math.degrees(value), DEG_DIGITS) if kind == planner.ROTATE else round(value, M_DIGITS))}
        for kind, value in result.legs]
    return jsonify(payload)


@app.route("/api/map/version")
def api_map_version():
    version = map_store.map_version()
    return jsonify({"version": None if version is None else str(version)})   # ns: past a JS number's precision


def _compensate(body):
    return bool(body.get("compensate", COMPENSATE_DEFAULT))


def _start_run(compensate):
    """The chat's plan handed to RunManager -> (body, HTTP status): /api/run's reply."""
    if runs.running:
        return {"error": BUSY_TEXT}, 409
    plan = chat.take_plan()
    if plan is None:
        return {"error": "no plan: chat a command first"}, 400
    try:
        run_id = runs.start(plan, compensate)
    except drive_chat.RunBusy:
        chat.give_back(plan)
        return {"error": BUSY_TEXT}, 409
    except OSError as exc:
        chat.give_back(plan)
        return {"error": f"demo_drive not started: {exc}"}, 500
    return {"ok": True, "run_id": run_id, "compensate": compensate and not runs.dry_run, "dry_run": runs.dry_run}, 200


@app.route("/api/chat", methods=["POST"])
def api_chat():
    body = request.get_json(silent=True) or {}
    text = str(body.get("text", "")).strip()
    if not text:
        return jsonify({"error": "send {\"text\": \"...\"}"}), 400
    if runs.running:
        return jsonify({"type": REPLY_BUSY, "text": BUSY_TEXT, "plan": None})
    intent = chat.intent(text)
    if intent == drive_chat.INTENT_GO_ON:
        return jsonify(chat.go_on(runs.unreached()))
    if intent == drive_chat.INTENT_RUN:
        if chat.plan is None:
            return jsonify({"type": drive_chat.REPLY_NONE, "text": drive_chat.NO_PLAN_TO_RUN, "plan": None})
        started, code = _start_run(_compensate(body))
        if code != 200:
            return jsonify({"type": drive_chat.REPLY_ERROR, "text": started["error"], "plan": None})
        return jsonify({"type": REPLY_RUN, "text": RUN_STARTED.format(run_id=started["run_id"]), "plan": None,
                        "run": started})
    return jsonify(chat.say(text))


@app.route("/api/chat/go_on", methods=["POST"])
def api_chat_go_on():
    if runs.running:
        return jsonify({"type": REPLY_BUSY, "text": BUSY_TEXT, "plan": None})
    return jsonify(chat.go_on(runs.unreached()))


@app.route("/api/chat/reset", methods=["POST"])
def api_chat_reset():
    chat.reset()
    return jsonify({"ok": True})


@app.route("/api/run", methods=["POST"])
def api_run():
    started, code = _start_run(_compensate(request.get_json(silent=True) or {}))
    return jsonify(started), code


@app.route("/api/run/stop", methods=["POST"])
def api_run_stop():
    return jsonify({"ok": runs.stop()})


@app.route("/api/run/status")
def api_run_status():
    return jsonify(dict(runs.status(), dry_run=runs.dry_run))


def _parse_args():
    parser = argparse.ArgumentParser(description="The real room's map, and the chat that drives the robot on it.")
    parser.add_argument("--provider", default=PROVIDER_AUTO, choices=(PROVIDER_AUTO, PROVIDER_RULE) + LLM_PROVIDERS,
                        help="the chat's parser: auto = CM's LLM, the rules without it")
    parser.add_argument("--dry-run", action="store_true",
                        help="Run plays a plan's timing with no robot and no camera (to try the page)")
    parser.add_argument("--port", type=int, default=PORT)
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    chat = drive_chat.ChatSession(args.provider)
    runs = drive_chat.RunManager(dry_run=args.dry_run)
    app.run(host="0.0.0.0", port=args.port, threaded=True)
