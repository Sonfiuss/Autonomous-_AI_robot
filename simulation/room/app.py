"""Flask server for the 2D room simulator.

Run:  python app.py            -> http://localhost:5000
Endpoints:
  GET /                        web page
  GET /api/room/new?seed=N     generate (random seed if omitted), save, return scene JSON
  GET /api/room/latest         last generated scene (generates one if none exists)
  GET /api/room/<seed>         load saved scene by seed (generates + saves if missing)
"""
import json
import logging
import os

from flask import Flask, jsonify, request, send_from_directory

from room_generator import generate

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
SCENES_DIR = os.path.join(BASE_DIR, "scenes")
LATEST_FILE = os.path.join(SCENES_DIR, "latest.json")
PORT = 5000

logger = logging.getLogger(__name__)

app = Flask(__name__, static_folder=STATIC_DIR, static_url_path="/static")


def _save_scene(scene):
    os.makedirs(SCENES_DIR, exist_ok=True)
    text = json.dumps(scene, indent=2)
    for path in (os.path.join(SCENES_DIR, f"room_{scene['seed']}.json"), LATEST_FILE):
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)


def _load_scene(path):
    """Read a saved scene; None when the file is missing or corrupt."""
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError) as exc:
        logger.warning("cannot load %s: %s", path, exc)
        return None


def _new_scene(seed=None):
    scene = generate(seed)
    _save_scene(scene)
    return scene


@app.route("/")
def index():
    return send_from_directory(STATIC_DIR, "index.html")


@app.route("/api/room/new")
def api_room_new():
    seed_arg = request.args.get("seed", "").strip()
    seed = int(seed_arg) if seed_arg.isdigit() else None
    logger.debug("new room request, seed=%s", seed)
    return jsonify(_new_scene(seed))


@app.route("/api/room/latest")
def api_room_latest():
    scene = _load_scene(LATEST_FILE) if os.path.exists(LATEST_FILE) else None
    return jsonify(scene if scene is not None else _new_scene())


@app.route("/api/room/<int:seed>")
def api_room_by_seed(seed):
    path = os.path.join(SCENES_DIR, f"room_{seed}.json")
    scene = _load_scene(path) if os.path.exists(path) else None
    return jsonify(scene if scene is not None else _new_scene(seed))


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    logger.info("Room simulator: http://localhost:%d", PORT)
    app.run(host="0.0.0.0", port=PORT, debug=False)
