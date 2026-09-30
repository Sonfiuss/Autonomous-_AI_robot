"""Offline tests of the realroom page's chat and run control - no robot, no camera, no LLM (the rule parser).
Run: python3 realroom/test_chat.py      (~1.5 min: every run is a real demo_drive --dry-run --no-vision)

Everything happens on a map of its own (REALROOM_MAPS_DIR, a temporary folder the demo_drive children inherit):
a 3 x 2 m room, the robot at (0.4, 1.0) facing +x, two chairs and a suitcase. The checks:
  - chat "đi thẳng 50 cm" -> a plan drawn 0.5 m ahead -> Run (dry) -> complete, the plan spent, the pose unmoved;
  - "đi tới cái ghế" asks which of the 2 chairs -> "2" -> a route to obj_2 from the robot's pose; driven, and
    refused by demo_drive once the robot moved since it was planned;
  - Stop mid-run -> aborted, the pose unmoved; a second Stop, and one after the run, do nothing; Stop once the
    robot is done -> still complete, run.json whole, no map built;
  - a second Run while one goes -> refused (RunBusy, HTTP 409), and the chat answers "busy";
  - "di chuyển tới bên cạnh cái ghế màu xanh" -> the green chair, no question: the chat names the target and the
    A* route; the dry run's message; said() -> "Đã tới" / "còn cách" / "Chưa tới" / cancelled / not run;
  - a run short of its target: leg_report (done / short with the measure and why / never run), the pose, go_on
    (status) -> ChatSession.go_on plans the rest of the way from the pose now; none without a target, another map;
  - "chạy" / "đi tiếp" are intents (not commands) unless a question waits; /api/chat "chạy" runs the plan (or says
    there is none), "đi tiếp" plans on from RunManager.unreached();
  - grounding with an LLM (CM's FakeClient): the object known -> no side / corner question, spots best first
    (middle before corners, nearest first); realroom_prompt swaps CM's side / corner rules, Vietnamese examples;
  - "lập bản đồ" (alone or with a move) -> refused: the chat moves on the map it has, maps are made at the
    terminal; build_map (the CLI's) records map_published / map_failed from drive_map's exit code;
  - --plan-json refuses steps outside the validator's limits, unknown legs, a route without its start pose;
  - /api/map/version changes when the pose does; /api/chat and /api/run refuse empty requests.
The route checks need CM's mv_client and libmv (project/tools/build_mv.sh), every run needs libmc
(project/tools/build_mc.sh): skipped without.
"""
import argparse
import json
import math
import os
import signal
import sys
import tempfile
import time

TMP = tempfile.mkdtemp(prefix="realroom_chat_")
os.environ["REALROOM_MAPS_DIR"] = os.path.join(TMP, "maps")    # before map_store is imported, for the children too

import app  # noqa: E402  (first: CM, which puts its own app.py on the path, comes in later)
import drive_chat  # noqa: E402
import map_store  # noqa: E402
import planner  # noqa: E402
from drive_chat import cfg, demo_drive, goto  # noqa: E402
from motion_plan import PlanError  # noqa: E402  (communication/, on the path through drive_chat)

RES = 0.05
ROOM_W, ROOM_L = 3.0, 2.0
ROBOT = {"x": 0.4, "y": 1.0, "theta": 0.0, "radius": 0.225}
CHAIR_1, CHAIR_2, SUITCASE = (1.5, 0.5), (2.4, 1.5), (1.5, 1.6)
BOX_HALF_M = 0.2                    # the objects are 0.4 m squares
MOVED_M = 0.1                       # the robot moved this far after a route was planned
MTIME_GAP_S = 0.01                  # between two writes of latest.json, so their mtimes differ
RUN_TIMEOUT_S = 90.0
POLL_S = 0.2
POSE_TOL = 1e-6
PATH_TOL_M = 1e-3
SPOT_REACH_M = 0.8                  # a spot beside obj_2 lies within this of its centre
FORWARD_M = 0.5
LONG_FORWARD = "đi thẳng 2 m"       # ~14 s: long enough to stop midway
WIRE_FORWARD = f"F {FORWARD_M:.3f} 0.150"
FIRST_LEG_PREFIX = "leg 1/"          # demo_drive.leg_label of a plan's first leg
DONE_PREFIX = demo_drive.STATUS_DONE.format(outcome="")   # "done: ": the drive has returned
RUNS_DIR = os.path.join(TMP, "runs")


def _box(oid, cls, x, y, color="unknown"):
    return {"id": oid, "class": cls, "confidence": 0.9, "color": color, "color_rgb": None, "color_share": None,
            "center": {"x": x, "y": y}, "yaw": 0.0,
            "polygon": [{"x": x + dx, "y": y + dy} for dx, dy in ((-BOX_HALF_M, -BOX_HALF_M), (BOX_HALF_M, -BOX_HALF_M),
                                                                 (BOX_HALF_M, BOX_HALF_M), (-BOX_HALF_M, BOX_HALF_M))],
            "free_sides": ["front", "back", "left", "right"], "near": []}


def _publish_room():
    """A drive_map-shaped run (scene, all-free known grid, meta) published into the temporary maps folder."""
    import numpy as np
    run_dir = os.path.join(TMP, "20260101_000000")
    map_dir = os.path.join(run_dir, map_store.RUN_MAP_DIR)
    os.makedirs(map_dir)
    scene = {"version": "2.0", "seed": None, "units": "m", "frame": "world", "source": "astra_map", "timestamp": 0.0,
             "room": {"width": ROOM_W, "length": ROOM_L}, "walls": [], "doors": [], "robot": dict(ROBOT),
             "objects": [_box("obj_1", "chair", *CHAIR_1, color="green"), _box("obj_2", "chair", *CHAIR_2, color="black"),
                         _box("obj_3", "suitcase", *SUITCASE)], "map_offset": {"x": 0.0, "y": 0.0}}
    with open(os.path.join(map_dir, map_store.RUN_SCENE), "w", encoding="utf-8") as f:
        json.dump(scene, f)
    cells = np.zeros((int(ROOM_L / RES) + 2, int(ROOM_W / RES) + 2), np.int8)     # a cell of margin all round
    np.save(os.path.join(map_dir, map_store.RUN_GRID), cells)
    with open(os.path.join(map_dir, map_store.RUN_META), "w", encoding="utf-8") as f:
        json.dump({"res_m": RES, "origin": [-RES, -RES]}, f)
    map_store.publish_run(run_dir)


def _has_mc():
    try:
        demo_drive.build_plan(demo_drive.map_scan_steps())
    except PlanError as exc:
        print(f"    skipped: {exc}")
        return False
    return True


def _has_mv():
    try:
        planner._mv_client().load_library()
    except Exception as exc:                          # no libmv: CM's MV build is missing
        print(f"    skipped: {exc}")
        return False
    return True


def _wait(runs, until, what):
    """The first status for which until(status) holds; AssertionError after RUN_TIMEOUT_S."""
    deadline = time.monotonic() + RUN_TIMEOUT_S
    while time.monotonic() < deadline:
        status = runs.status()
        if until(status):
            return status
        time.sleep(POLL_S)
    raise AssertionError(f"timed out waiting for {what}: {runs.status()}")


def _finished(runs):
    return _wait(runs, lambda s: s["state"] == drive_chat.STATE_FINISHED, "the run to finish")


def _pose():
    return map_store.robot_pose(map_store.load_latest())


def _run_json(run_dir):
    with open(os.path.join(run_dir, demo_drive.RUN_JSON), encoding="utf-8") as f:
        return json.load(f)


def check_steps_plan_and_dry_run():
    """A relative command: planned, drawn from the robot's pose, run dry to completion, then spent."""
    errors = []
    chat, runs = drive_chat.ChatSession("rule"), drive_chat.RunManager(RUNS_DIR, dry_run=True)
    reply = chat.say(f"đi thẳng {FORWARD_M * 100:.0f} cm")
    shown = reply["plan"] or {}
    if reply["type"] != drive_chat.REPLY_PLAN or [leg["wire"] for leg in shown.get("legs", [])] != [WIRE_FORWARD, "S"]:
        return [f"plan reply: {reply}"]
    expected = [[ROBOT["x"], ROBOT["y"]], [ROBOT["x"] + FORWARD_M, ROBOT["y"]]]
    if any(math.dist(a, b) > PATH_TOL_M for a, b in zip(shown["path"], expected)) or len(shown["path"]) != 2:
        errors.append(f"path {shown['path']}, expected {expected}")
    if shown["occupied_cells"] or shown["unseen_share"] != 0.0:
        errors.append(f"swept floor: {shown['occupied_cells']} occupied, {shown['unseen_share']} unseen on a free map")
    before = _pose()
    run_id = runs.start(chat.take_plan(), compensate=True)
    status = _finished(runs)
    if status["result"] != demo_drive.RESULT_COMPLETE or status["exit_code"] != demo_drive.EXIT_OK:
        errors.append(f"dry run: {status}")
    run = _run_json(os.path.join(RUNS_DIR, run_id))
    if run["command"] != f"đi thẳng {FORWARD_M * 100:.0f} cm" or run["mode"] != demo_drive.MODE_DRY_RUN:
        errors.append(f"run.json command {run['command']!r}, mode {run['mode']}")
    with open(os.path.join(RUNS_DIR, run_id, drive_chat.APPROVED_PLAN_FILE), encoding="utf-8") as f:
        approved = json.load(f)
    if [s["action"] for s in approved.get("steps", [])] != ["forward"]:
        errors.append(f"approved plan {approved}")
    if chat.take_plan() is not None:
        errors.append("the plan was not spent by the run")
    if any(abs(a - b) > POSE_TOL for a, b in zip(_pose(), before)):
        errors.append(f"a dry run moved the pose: {before} -> {_pose()}")
    return errors


def check_question_then_goal():
    """Two chairs: the chat asks which; the answer routes to it from the robot's pose. The route runs from there,
    and is refused by demo_drive once the robot moved since it was planned."""
    if not _has_mv():
        return []
    errors = []
    chat, runs = drive_chat.ChatSession("rule"), drive_chat.RunManager(RUNS_DIR, dry_run=True)
    reply = chat.say("đi tới cái ghế")
    if reply["type"] != drive_chat.REPLY_ASK or "2 chair" not in reply["text"]:
        return [f"no question about the two chairs: {reply}"]
    reply = chat.say("2")
    shown = reply["plan"] or {}
    if reply["type"] != drive_chat.REPLY_PLAN or "obj_2" not in shown.get("goal", ""):
        return [f"answer '2' did not plan to obj_2: {reply}"]
    end = shown["path"][-1]
    if math.dist(shown["path"][0], (ROBOT["x"], ROBOT["y"])) > PATH_TOL_M or math.dist(end, CHAIR_2) > SPOT_REACH_M:
        errors.append(f"route {shown['path']}")
    prepared = chat.plan
    if prepared.from_pose is None or prepared.from_pose["map_id"] != map_store.load_latest()["map_id"]:
        errors.append(f"route without its start pose: {prepared.from_pose}")
    runs.start(chat.take_plan(), compensate=False)
    status = _finished(runs)
    if status["result"] != demo_drive.RESULT_COMPLETE:
        errors.append(f"route dry run: {status}")
    before = _pose()
    map_store.save_pose((before[0] + MOVED_M, before[1], before[2]), "test: moved")
    runs.start(prepared, compensate=False)
    status = _finished(runs)
    if status["exit_code"] != demo_drive.EXIT_ERROR or status["result"] is not None or "di chuyển" not in status["log"]:
        errors.append(f"a route from an old pose was driven: {status}")
    map_store.save_pose(before, "test: back")
    return errors


def check_stop_midway():
    """Stop is SIGINT to the run's group: aborted, the pose unmoved; nothing to stop afterwards. The page's server
    ignores SIGINT here, as a background job does: its children inherit that, and Stop must still work."""
    errors = []
    chat, runs = drive_chat.ChatSession("rule"), drive_chat.RunManager(RUNS_DIR, dry_run=True)
    if chat.say(LONG_FORWARD)["type"] != drive_chat.REPLY_PLAN:
        return ["no plan for the long move"]
    before = _pose()
    previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        runs.start(chat.take_plan(), compensate=False)
    finally:
        signal.signal(signal.SIGINT, previous)
    _wait(runs, lambda s: s.get("line", "").startswith(FIRST_LEG_PREFIX), "the first leg")
    if not runs.stop() or not runs.stop():
        errors.append("stop refused while running")
    status = _finished(runs)
    if status["result"] != demo_drive.RESULT_ABORTED or status["exit_code"] != demo_drive.EXIT_FAILED:
        errors.append(f"stopped run: {status}")
    if runs.stop():
        errors.append("stop accepted with nothing running")
    if any(abs(a - b) > POSE_TOL for a, b in zip(_pose(), before)):
        errors.append(f"an aborted run moved the pose: {before} -> {_pose()}")
    return errors


def check_stop_after_the_drive():
    """Stop once the robot is done (the video's tail): noted, not raised - the run stays complete, run.json whole,
    and a map run would build no map."""
    errors = []
    chat, runs = drive_chat.ChatSession("rule"), drive_chat.RunManager(RUNS_DIR, dry_run=True)
    if chat.say("quay trái 90 độ")["type"] != drive_chat.REPLY_PLAN:
        return ["no plan for the turn"]
    run_id = runs.start(chat.take_plan(), compensate=False)
    _wait(runs, lambda s: s.get("line", "").startswith(DONE_PREFIX), "the drive's end")
    if not runs.stop():
        return [f"the run ended before Stop reached its {cfg.VIDEO_TAIL_S:g} s tail (a slow poll?)"]
    status = _finished(runs)
    run = _run_json(os.path.join(RUNS_DIR, run_id))
    if status["result"] != demo_drive.RESULT_COMPLETE or not run.get(demo_drive.STOP_REQUESTED):
        errors.append(f"stop in the tail: result {status['result']}, stop noted {run.get(demo_drive.STOP_REQUESTED)}")
    runlog = demo_drive.RunLog(os.path.join(RUNS_DIR, run_id))
    runlog.data[demo_drive.STOP_REQUESTED] = True
    args = argparse.Namespace(dry_run=False, compensate=False)
    if demo_drive._map_after(args, runlog, demo_drive.RESULT_COMPLETE) is not None:
        errors.append("a map was built after the user stopped")
    elif [e.get("reason") for e in runlog.events] != [demo_drive.MAP_SKIP_STOPPED]:
        errors.append(f"map skip events {runlog.events}")
    return errors


def check_one_run_at_a_time():
    """A second Run while one goes is refused (RunBusy; HTTP 409, the plan kept), and so is chatting."""
    errors = []
    app.chat = drive_chat.ChatSession("rule")
    app.runs = runs = drive_chat.RunManager(RUNS_DIR, dry_run=True)
    client = app.app.test_client()
    if client.post("/api/chat", json={"text": LONG_FORWARD}).get_json()["type"] != drive_chat.REPLY_PLAN:
        return ["no plan through /api/chat"]
    first = client.post("/api/run", json={"compensate": True})
    if first.status_code != 200 or not first.get_json()["dry_run"] or first.get_json()["compensate"]:
        errors.append(f"/api/run: {first.status_code} {first.get_json()}")
    try:
        runs.start(demo_drive.Prepared(None, None, "", "", False, None), compensate=False)
        errors.append("a second run started")
    except drive_chat.RunBusy:
        pass
    busy = client.post("/api/chat", json={"text": "quay trái 90 độ"}).get_json()
    if busy["type"] != app.REPLY_BUSY:
        errors.append(f"chat while running: {busy}")
    app.chat.plan = "a plan"                                   # whatever is there must stay for later
    second = client.post("/api/run", json={})
    if second.status_code != 409 or app.chat.plan != "a plan":
        errors.append(f"second /api/run: {second.status_code}, plan kept {app.chat.plan!r}")
    if not client.post("/api/run/stop").get_json()["ok"]:
        errors.append("/api/run/stop refused")
    _finished(runs)
    # Stopped this early, demo_drive may not have reached its first leg - or even its main(): cancelled, or aborted.
    status = client.get("/api/run/status").get_json()
    if status["result"] not in (demo_drive.RESULT_CANCELLED, demo_drive.RESULT_ABORTED) or status.get("end_pose"):
        errors.append(f"/api/run/status after an early stop: {status}")
    return errors


def check_no_mapping():
    """The chat never makes a map: 'lập bản đồ', alone or with a move, is refused with the terminal's way; a plain
    move plans as ever, not flagged for a map."""
    errors = []
    chat = drive_chat.ChatSession("rule")
    for text in ("lập bản đồ", "quay trái 360 độ rồi lập bản đồ"):
        reply = chat.say(text)
        if reply["type"] != drive_chat.REPLY_NONE or reply["text"] != drive_chat.NO_MAPPING or chat.plan is not None:
            errors.append(f"{text!r}: {reply}")
    reply = chat.say("đi thẳng 50 cm")
    if reply["type"] != drive_chat.REPLY_PLAN or chat.plan.wants_map or "map" in reply["plan"]:
        errors.append(f"plain move: {reply}")
    return errors


def check_color_goal():
    """'di chuyển tới bên cạnh cái ghế màu xanh' (the rules): the green chair of two, no question; the chat names
    the target in its words and the A* route; the route's target rides to the run, whose message the chat shows."""
    if not _has_mv():
        return []
    errors = []
    chat, runs = drive_chat.ChatSession("rule"), drive_chat.RunManager(RUNS_DIR, dry_run=True)
    reply = chat.say("di chuyển tới bên cạnh cái ghế màu xanh")
    target = (reply["plan"] or {}).get("target") or {}
    if reply["type"] != drive_chat.REPLY_PLAN or target.get("object_id") != "obj_1":
        return [f"not a plan to the green chair obj_1: {reply}"]
    if "ghế xanh lá (obj_1)" not in target.get("name", "") or f"Đích: {target['name']}" not in reply["text"] \
            or "Đường A*" not in reply["text"]:
        errors.append(f"the chat's words: {reply['text']!r}, target {target}")
    if math.dist(reply["plan"]["path"][-1], (target["x"], target["y"])) > PATH_TOL_M:
        errors.append(f"the route ends at {reply['plan']['path'][-1]}, the target at {target}")
    runs.start(chat.take_plan(), compensate=False)
    status = _finished(runs)
    if status["target"] != target or status["message"] != drive_chat.SAID_DRY:
        errors.append(f"dry run: target {status['target']}, message {status['message']!r}")
    reply = chat.say("đi tới cạnh cái ghế màu đỏ")
    if reply["type"] != drive_chat.REPLY_NONE or "không có" not in reply["text"]:
        errors.append(f"no red chair: {reply}")
    return errors


def check_leg_report():
    """leg_report on run 140606's shape: a right turn stuck after 3 legs all cut short, the rest never run."""
    legs = [(drive_chat.ROTATE, -2.1259), (drive_chat.FORWARD, 0.55), (drive_chat.ROTATE, 0.78)]
    run = {"requests": [{"kind": "ROTATE", "target": -2.1259, "measured": -1.98239, "outcome": "stuck",
                         "legs": [1, 2, 3]}],
           "executed_legs": [{"cut_short": True}, {"cut_short": True}, {"cut_short": True}]}
    want = ["✗ quay phải 122°: đo được -114° - bù thêm không nhích được nữa, firmware cắt ngắn 3/3 lần",
            "– đi thẳng 0.55 m: chưa chạy", "– quay trái 45°: chưa chạy"]
    got = drive_chat.leg_report(legs, run)
    errors = [] if got == want else [f"leg_report {got}"]
    ok_run = {"requests": [{"measured": -2.12, "outcome": "ok", "legs": [1]}], "executed_legs": [{"cut_short": False}]}
    if drive_chat.leg_report(legs[:1], ok_run) != ["✓ quay phải 122°: đo được -121°"]:
        errors.append(f"a leg done: {drive_chat.leg_report(legs[:1], ok_run)}")
    if drive_chat.leg_report(legs, {}) != []:
        errors.append("a run without per-leg records")
    return errors


def check_map_update_measured():
    """A --compensate run that stopped early (stuck) with every leg measured moves the pose as measured, not
    predicted; one ended by Stop or a lost link, or with a leg the camera could not measure, is predicted."""
    class _Log:
        def __init__(self, requests):
            self.data = {"run_id": "test", "measured_pose": [0.0, 0.0, 0.1], "requests": requests}
    errors = []
    before = _pose()
    cases = [([{"outcome": "stuck", "predicted": False}], False),
             ([{"outcome": "ok", "predicted": False}, {"outcome": "aborted", "predicted": False}], True),
             ([{"outcome": "robot_link failed", "predicted": False}], True),
             ([{"outcome": "weak odometry", "predicted": True}], True)]
    for requests, want in cases:
        log = _Log(requests)
        demo_drive._map_update(log, None, demo_drive.RESULT_FAILED, True)
        if log.data.get("end_pose", {}).get("predicted") is not want:
            errors.append(f"{requests} -> {log.data.get('end_pose')}, wanted predicted {want}")
    map_store.save_pose(before, "test: back")
    return errors


def check_intents():
    """'chạy' / 'đi tiếp' (whole messages, any case or punctuation) act on the plan; a command with the same
    word, or any message while a question waits, is not one."""
    chat = drive_chat.ChatSession("rule")
    cases = [("chạy", drive_chat.INTENT_RUN), ("Chạy đi!", drive_chat.INTENT_RUN), ("OK", drive_chat.INTENT_RUN),
             ("đi tiếp", drive_chat.INTENT_GO_ON), ("Tiếp tục.", drive_chat.INTENT_GO_ON),
             ("chạy thẳng 50 cm", None), ("đi tiếp 30 cm", None)]
    errors = [f"{text!r} -> {chat.intent(text)}, wanted {want}" for text, want in cases if chat.intent(text) != want]
    if chat.say("quay 90 độ")["type"] != drive_chat.REPLY_ASK or chat.intent("chạy") is not None:
        errors.append("'chạy' taken as an intent while a question waits")
    chat.reset()
    return errors


def check_go_on():
    """go_on plans from the pose NOW to the target a run stopped short of; /api/chat 'đi tiếp' / 'chạy' use it."""
    if not _has_mv():
        return []
    errors = []
    chat = drive_chat.ChatSession("rule")
    if chat.go_on(None)["text"] != drive_chat.NOTHING_TO_GO_ON:
        errors.append("go_on with no target")
    target = (chat.say("di chuyển tới bên cạnh cái ghế màu xanh")["plan"] or {}).get("target")
    if not target:
        return errors + ["no target to go on to"]
    if chat.go_on(dict(target, map_id="another"))["text"] != drive_chat.GO_ON_MAP_CHANGED:
        errors.append("go_on on another map")
    before = _pose()
    moved = (before[0] + MOVED_M, before[1] - MOVED_M, before[2] - 1.0)   # the run turned, then stopped
    map_store.save_pose(moved, "test: stopped short")
    reply = chat.go_on(target)
    shown = reply["plan"] or {}
    if reply["type"] != drive_chat.REPLY_PLAN or shown.get("target", {}).get("spot_id") != target["spot_id"]:
        errors.append(f"go_on: {reply}")
    elif math.dist(shown["path"][0], moved[:2]) > PATH_TOL_M or \
            math.dist(shown["path"][-1], (target["x"], target["y"])) > PATH_TOL_M:
        errors.append(f"go_on route {shown['path']} from {moved[:2]} to {target}")
    if drive_chat.GO_ON_NOTE not in reply["text"] or chat.plan is None or \
            chat.plan.from_pose["x"] != moved[0]:
        errors.append(f"go_on reply / plan: {reply['text']!r}, {chat.plan and chat.plan.from_pose}")
    # the endpoints: "chạy" runs the pending plan (a dry run), with none it says so; "đi tiếp" with no short run
    app.chat, app.runs = chat, drive_chat.RunManager(RUNS_DIR, dry_run=True)
    client = app.app.test_client()
    body = client.post("/api/chat", json={"text": "chạy"}).get_json()
    if body.get("type") != app.REPLY_RUN or not body.get("run", {}).get("run_id"):
        errors.append(f"'chạy' with a plan: {body}")
    _finished(app.runs)
    body = client.post("/api/chat", json={"text": "chạy"}).get_json()
    if body.get("text") != drive_chat.NO_PLAN_TO_RUN:
        errors.append(f"'chạy' with no plan: {body}")
    body = client.post("/api/chat", json={"text": "đi tiếp"}).get_json()
    if body.get("text") != drive_chat.NOTHING_TO_GO_ON:          # a dry run never leaves the pose short
        errors.append(f"'đi tiếp' after a dry run: {body}")
    map_store.save_pose(before, "test: back")
    return errors


def check_llm_grounding():
    """With an LLM that knows the object but not the side (CM's FakeClient), no question is asked: the spots come
    best first - middles before corners, the nearest middle first. realroom_prompt swaps CM's side / corner
    rules and carries Vietnamese examples."""
    from llm_client import FakeClient
    errors = []
    prompt = goto.realroom_prompt()
    rules = "\n".join(prompt["rules"])
    if "do NOT ask" not in rules or "Never ask where along a side" not in rules or "ask: corner" in rules:
        errors.append("realroom_prompt rules not swapped")
    if prompt["examples"] != goto.PROMPT_EXAMPLES:
        errors.append("realroom_prompt examples")
    scene = map_store.load_latest()
    llm = FakeClient([{"type": "ask", "question": "", "slots": {"target": "obj_3", "side": None, "spot": None},
                       "matches": ["obj_3:*"]}])
    asked = []
    spots, why = goto.ground(scene, "đi tới cạnh cái vali", llm, lambda q: asked.append(q) or "")
    if asked or not spots:
        return errors + [f"asked {asked}, spots {spots}, why {why}"]
    kinds = [s["spot"] for s in spots]
    centres = [s for s in spots if s["spot"] == goto.CENTER_SPOT]
    dists = [math.dist((s["x"], s["y"]), (ROBOT["x"], ROBOT["y"])) for s in centres]
    if {s["target_id"] for s in spots} != {"obj_3"} or kinds[:len(centres)] != [goto.CENTER_SPOT] * len(centres) \
            or dists != sorted(dists):
        errors.append(f"spots not best first: {[(s['id'], round(d, 2)) for s, d in zip(spots, dists)]}")
    return errors


def check_said():
    """said(): what the chat says when a run ends, from its result, end pose and the plan's target."""
    target = {"map_id": "m", "name": "cạnh sau của ghế (obj_1)", "x": 1.0, "y": 1.0}
    pose = {"map_id": "m", "x": 1.05, "y": 1.0, "theta": 0.0, "predicted": False}
    cases = [
        ({"result": "complete", "end_pose": pose}, target, "Đã tới cạnh sau của ghế (obj_1)"),
        ({"result": "complete", "end_pose": dict(pose, x=1.4)}, target, "còn cách cạnh sau của ghế (obj_1) 40 cm"),
        ({"result": "complete", "end_pose": dict(pose, predicted=True)}, target, "dự đoán"),
        ({"result": "complete", "end_pose": dict(pose, map_id="other")}, target, "Đã chạy xong. Robot ở (1.05, 1.00)"),
        ({"result": "complete", "end_pose": pose}, None, "Đã chạy xong."),
        ({"result": "aborted", "end_pose": pose}, target, "Chưa tới cạnh sau của ghế (obj_1): đã dừng"),
        ({"result": "failed", "end_pose": None}, target, "KHÔNG cập nhật"),
        ({"result": "failed", "end_pose": pose}, target, "Robot giờ ở (1.05, 1.00), hướng 0° (camera đo)"),
        ({"result": "cancelled"}, target, drive_chat.SAID_CANCELLED),
        ({"result": None, "log": "x\nLỗi: route planned from another pose\n", "exit_code": 2}, target,
         "Robot không chạy: Lỗi: route planned from another pose"),
    ]
    errors = [f"{reply} -> {drive_chat.said(reply, t, False)!r}, wanted {want!r}" for reply, t, want in cases
              if want not in drive_chat.said(reply, t, False)]
    if drive_chat.said({"result": "complete", "end_pose": None}, target, True) != drive_chat.SAID_DRY:
        errors.append("a dry run's message")
    return errors


def check_build_map():
    """build_map: drive_map's exit code 0 -> map_published (its args: --run <run> --publish), else map_failed;
    run.json saved either way."""
    errors = []
    for name, code in (("ok", 0), ("fail", 3)):
        run_dir = os.path.join(TMP, f"build_{name}")
        os.makedirs(run_dir)
        stub = os.path.join(TMP, f"drive_map_{name}.py")
        with open(stub, "w", encoding="utf-8") as f:
            f.write(f"import json, sys\njson.dump(sys.argv[1:], open({os.path.join(run_dir, 'args.json')!r}, 'w'))\n"
                    f"sys.exit({code})\n")
        runlog = demo_drive.RunLog(run_dir)
        published = demo_drive.build_map(runlog, stub)
        with open(os.path.join(run_dir, "args.json"), encoding="utf-8") as f:
            args = json.load(f)
        saved = _run_json(run_dir)
        events = [e["event"] for e in saved["events"] if e["event"].startswith("map_")]
        want = demo_drive.EVENT_MAP_PUBLISHED if code == 0 else demo_drive.EVENT_MAP_FAILED
        if published != (code == 0) or events != [want] or args != ["--run", run_dir, "--publish"]:
            errors.append(f"drive_map exit {code}: published {published}, events {events}, args {args}")
    return errors


def check_plan_json_limits():
    """--plan-json takes only what the validator would: a step over the limits, an unknown action, a MOVE leg,
    a route without its start pose are refused; a start pose holding a NaN counts as moved."""
    errors = []
    bad = {"too far": {"steps": [{"action": "forward", "amount": 5.0, "stated": True}]},
           "unknown action": {"steps": [{"action": "jump", "amount": 0.3, "stated": True}]},
           "no steps": {"steps": []},
           "MOVE leg": {"legs": [["MOVE", 0.3]], "from_pose": {"map_id": "x", "x": 0, "y": 0, "theta": 0}},
           "route without its pose": {"legs": [["FORWARD", 0.3]]},
           "not json": None}
    for what, data in bad.items():
        path = os.path.join(TMP, "bad_plan.json")
        with open(path, "w", encoding="utf-8") as f:
            f.write("{" if data is None else json.dumps(data))
        try:
            demo_drive.load_plan_json(path, cfg.CRUISE_SPEED_M_S, cfg.YAW_RATE_RAD_S)
            errors.append(f"{what}: accepted")
        except PlanError:
            pass
    scene = map_store.load_latest()
    x, y, theta = map_store.robot_pose(scene)
    here = {"map_id": scene["map_id"], "x": x, "y": y, "theta": theta}
    if demo_drive._moved_since(here) is not None or demo_drive._moved_since(dict(here, x=math.nan)) is None:
        errors.append("route start check: the robot's own pose refused, or a NaN pose taken for it")
    return errors


def check_endpoints():
    """/api/map/version follows the pose; empty chat and Run with no plan are refused."""
    errors = []
    app.chat = drive_chat.ChatSession("rule")
    app.runs = drive_chat.RunManager(RUNS_DIR, dry_run=True)
    client = app.app.test_client()
    first = client.get("/api/map/version").get_json()["version"]
    time.sleep(MTIME_GAP_S)
    pose = _pose()
    map_store.save_pose(pose, "test: same place")
    second = client.get("/api/map/version").get_json()["version"]
    if not first or first == second:
        errors.append(f"map version {first} -> {second} after a pose change")
    if client.post("/api/chat", json={"text": "  "}).status_code != 400:
        errors.append("empty chat accepted")
    if client.post("/api/run", json={}).status_code != 400:
        errors.append("Run with no plan accepted")
    if client.get("/api/run/status").get_json()["state"] != drive_chat.STATE_IDLE:
        errors.append("a new RunManager is not idle")
    return errors


def main():
    _publish_room()
    failed = 0
    checks = [check_plan_json_limits, check_build_map, check_endpoints, check_said, check_leg_report, check_intents,
              check_llm_grounding, check_map_update_measured]
    if _has_mc():
        checks += [check_steps_plan_and_dry_run, check_question_then_goal, check_stop_midway,
                   check_stop_after_the_drive, check_one_run_at_a_time, check_no_mapping, check_color_goal, check_go_on]
    for check in checks:
        try:
            errors = check()
        except AssertionError as exc:
            errors = [str(exc)]
        print(f"{'PASS' if not errors else 'FAIL'} {check.__name__}")
        for e in errors:
            print("   ", e)
        failed += bool(errors)
    print(f"(runs and maps in {TMP})")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
