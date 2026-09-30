"""The realroom page's chat and run control (tasks 2026-09-30_realroom-chat-drive, _realroom-chat-ui): the chat
moves the robot on the map already made - a place the user names is found on it and routed there by A*, or
relative moves are driven. A command is made ready exactly as at the terminal - communication/demo_drive.prepare:
the parser, its questions, a place grounded and routed on this map - then shown on the map, driven only when the
user presses Run, by demo_drive itself (--plan-json) in a process group of its own, so Stop is the terminal's
Ctrl-C; when it ends the chat says whether the robot got there. Maps are made at the terminal (demo_drive --map).

  ChatSession.say(text)   -> {type: ask | plan | none | error, text, plan}; after an "ask" the next message
                             is the answer (a thread per command waits on it, like input() at the terminal)
  ChatSession.intent(text) -> INTENT_RUN ("chạy") / INTENT_GO_ON ("đi tiếp") / None: words the app acts on
  ChatSession.go_on(target) -> a plan from where the robot is now to the target a run stopped short of
  RunManager.start(prepared, compensate) / stop() / status() / unreached()
Both hold one thing at a time: the page is one robot's remote, not a queue.
"""
import collections
import json
import logging
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime

import map_store
import planner

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COMM_DIR = os.path.join(REPO_ROOT, "communication")
if COMM_DIR not in sys.path:
    sys.path.insert(0, COMM_DIR)
import demo_drive  # noqa: E402  (communication module, path set just above)
import drive_config as cfg  # noqa: E402
import goto  # noqa: E402
from candidates import build_candidates  # noqa: E402  (CM module, on the path through goto)
from cmd_parser import (ACTION_BACKWARD, ACTION_FORWARD, ACTION_TURN_LEFT, ACTION_TURN_RIGHT,  # noqa: E402
                        LINEAR_ACTIONS, PROVIDER_AUTO, SAY_ACTION, CommandParser, ParseResult, ParserSetupError, fold,
                        split_map_request)
from motion_plan import FORWARD, ROTATE, STOP  # noqa: E402

DEMO_DRIVE_SCRIPT = os.path.join(COMM_DIR, "demo_drive.py")
REPLY_ASK, REPLY_PLAN, REPLY_NONE, REPLY_ERROR = "ask", "plan", "none", "error"
REPLY_TIMEOUT_S = 90.0              # an LLM reading with its retry, then the grounding dialogue's first turn
APPROVED_PLAN_FILE = "approved_plan.json"   # in the run's folder: what the page approved
CONSOLE_LOG = "console.log"                  # demo_drive's terminal output
LOG_TAIL_LINES = 14
STATE_IDLE, STATE_RUNNING, STATE_STOPPING, STATE_FINISHED = "idle", "running", "stopping", "finished"
MAP_EVENTS = (demo_drive.EVENT_MAP_PUBLISHED, demo_drive.EVENT_MAP_FAILED, demo_drive.EVENT_MAP_SKIPPED)
M_DIGITS, S_DIGITS, SHARE_DIGITS, ELAPSED_DIGITS = 3, 2, 3, 1
CANCELLED = "Đã huỷ."
ERROR_TEXT = "Lỗi: {error}"
TIMED_OUT = "Không trả lời kịp ({seconds:.0f} s): lệnh bị huỷ, hãy nói lại."
RUN_HINT = "Bấm Chạy để robot chạy."
WARN_OCCUPIED = "! đường quét qua {n} ô vật cản trên bản đồ - kiểm tra trước khi chạy"
WARN_UNSEEN = "! {share:.0%} sàn trên đường chưa ai nhìn thấy (planner cho phép tới {limit:.0%})"
NO_MAPPING = ("Khung chat này chỉ điều khiển robot đi trên bản đồ đã có. Lập bản đồ ở terminal: "
              "python3 communication/demo_drive.py --compensate --map \"<các bước quét>\"")
NOTE_RULES = "(LLM không đọc được lệnh: {why} - bộ luật đọc)"
PLAN_GOAL = "Đích: {name}\nĐường A*: {length:.2f} m, {legs} leg, khoảng {seconds:.0f} s"
PLAN_STEPS = "Các bước: {steps}\n{legs} leg, khoảng {seconds:.0f} s"
# What the chat says when a run ends (RunManager.status "message").
ARRIVE_TOL_M = 0.15             # nearer than this to the target spot: arrived
SAID_ARRIVED = "Đã tới {name}. Robot ở ({x:.2f}, {y:.2f}), cách đích {cm:.0f} cm."
SAID_SHORT = "Đã chạy hết đường nhưng còn cách {name} {cm:.0f} cm - robot ở ({x:.2f}, {y:.2f})."
SAID_DONE = "Đã chạy xong. Robot ở ({x:.2f}, {y:.2f}), hướng {deg:.0f}°."
SAID_DONE_NO_POSE = "Đã chạy xong."
SAID_PREDICTED = " (vị trí dự đoán theo lệnh, camera không đo)"
SAID_DRY = "Chạy thử xong (không robot, không camera): vị trí trên bản đồ không đổi."
SAID_CANCELLED = "Đã huỷ trước khi robot chạy."
SAID_STOPPED = "Chưa tới{what}: đã dừng giữa đường."
SAID_FAILED = "Chưa tới{what}: lần chạy dừng giữa kế hoạch ({result})."
SAID_NOT_RUN = "Robot không chạy: {why}"
SAID_WHERE = " Robot giờ ở ({x:.2f}, {y:.2f}), hướng {deg:.0f}°{note}."
SAID_MEASURED = " (camera đo)"
SAID_WHERE_UNKNOWN = " Vị trí robot trên bản đồ KHÔNG cập nhật: đặt robot lại chỗ cũ hoặc lập bản đồ lại."
ERROR_WORD = "Lỗi"              # demo_drive's refusals start with it ("Lỗi: ...")
# A whole message (folded, punctuation dropped) that is one of these acts on the plan instead of being a command.
INTENT_RUN, INTENT_GO_ON = "run", "go on"
RUN_WORDS = frozenset(("chay", "chay di", "chay luon", "bat dau", "xuat phat", "ok", "ok chay", "run", "go", "start"))
GO_ON_WORDS = frozenset(("di tiep", "tiep tuc", "tiep", "chay tiep", "continue", "go on"))
PUNCTUATION = ".,!?;:"
NO_PLAN_TO_RUN = "Chưa có kế hoạch để chạy: nói robot đi đâu trước."
NOTHING_TO_GO_ON = "Không có chỗ nào để đi tiếp: lần chạy trước đã tới nơi, hoặc chưa biết robot đang ở đâu."
GO_ON_MAP_CHANGED = "Bản đồ đã đổi từ lần chạy trước: nói lại chỗ cần tới."
GO_ON_NOTE = "Đi tiếp từ chỗ robot đang đứng:"
GO_ON_COMMAND = "đi tiếp tới {name}"
PROVIDER_GO_ON = "go on"         # ParseResult.provider of a go-on plan: no parser read it
# Per leg of a run that did not arrive (leg_report): done / not done / not run, the leg, what was measured.
MARK_DONE, MARK_SHORT, MARK_NOT_RUN = "✓", "✗", "–"
LEG_NOT_RUN = "{mark} {leg}: chưa chạy"
LEG_MEASURED = "{mark} {leg}: đo được {measured}"
LEG_WHY = " - {why}"
LEG_CUT = "firmware cắt ngắn {cut}/{runs} lần"
OUTCOME_WORDS = {"ok": "", "off target": "lệch đích", "weak odometry": "camera đo yếu, vị trí dự đoán",
                 "stuck": "bù thêm không nhích được nữa", "camera lost": "mất camera",
                 "robot_link failed": "lỗi nối ESP32", "aborted": "bị Dừng"}   # leg_executor's outcomes
OUTCOME_OK = "ok"
SAID_GO_ON_HINT = "Đã lập đường đi tiếp bên dưới; bấm Chạy, hoặc gõ \"chạy\"."
EXIT_TEXT = "demo_drive thoát {code}"

logger = logging.getLogger(__name__)


class RunBusy(RuntimeError):
    """A run the page started is still going."""


def _say_step(step):
    """A MotionStep in the user's words: 'đi thẳng 50 cm', 'quay trái 90 độ'."""
    if step.action in LINEAR_ACTIONS:
        return f"{SAY_ACTION[step.action]} {step.amount * cfg.CM_PER_M:g} cm"
    return f"{SAY_ACTION[step.action]} {step.amount:g} độ"


def preview(prepared, scene):
    """What the page shows of a Prepared before it runs: its legs and time, its path on the map (the robot's pose
    moved leg by leg, map frame) and the floor the robot's disc sweeps along it - occupied cells and the unseen
    share, the planner's checks. A route passed them to be planned; a command's steps are only warned about, as
    at the terminal. scene None (no map yet): no path."""
    plan = prepared.plan
    legs = [{"primitive": leg.plan_line(), "wire": leg.wire, "duration_s": round(leg.duration_s, S_DIGITS),
             "label": demo_drive.leg_label(plan, k)} for k, leg in enumerate(plan.legs)]
    shown = {"command": prepared.text, "provider": prepared.result.provider, "goal": prepared.goal,
             "target": prepared.target, "steps": [_say_step(step) for step in plan.steps], "legs": legs,
             "duration_s": round(plan.duration_s, S_DIGITS),
             "length_m": round(sum(abs(leg.value) for leg in plan.legs if leg.kind == FORWARD), M_DIGITS),
             "map_id": None, "path": [], "occupied_cells": 0, "unseen_share": None}
    if scene is None:
        return shown
    pose = map_store.robot_pose(scene)
    path = [pose[:2]]
    for leg in plan.legs:
        if leg.kind == ROTATE:
            pose = map_store.compose(pose, (0.0, 0.0, leg.value))
        elif leg.kind == FORWARD:
            pose = map_store.compose(pose, (leg.value, 0.0, 0.0))
            path.append(pose[:2])
    grid = map_store.load_grid(scene)
    values = grid.cells.reshape(-1)[planner.swept_cells(grid, path, scene["robot"]["radius"])]
    shown.update({"map_id": scene["map_id"], "path": [[round(x, M_DIGITS), round(y, M_DIGITS)] for x, y in path],
                  "occupied_cells": int((values == map_store.KNOWN_OCCUPIED).sum()),
                  "unseen_share": round(float((values == map_store.KNOWN_UNSEEN).mean()), SHARE_DIGITS)
                  if values.size else None})
    logger.debug("preview %r: %d points, %d occupied cells, unseen %s", prepared.text, len(path),
                 shown["occupied_cells"], shown["unseen_share"])
    return shown


def _plan_text(notes, shown, target):
    """The chat's words for a plan: where it goes (or its steps), how far and long, the warnings."""
    lines = list(notes)
    legs = len(shown["legs"]) - 1                   # the closing STOP: no leg to the user
    if target is not None:
        lines.append(PLAN_GOAL.format(name=target["name"], length=shown["length_m"], legs=legs,
                                      seconds=shown["duration_s"]))
    else:
        lines.append(PLAN_STEPS.format(steps=", ".join(shown["steps"]), legs=legs, seconds=shown["duration_s"]))
    if shown["occupied_cells"]:
        lines.append(WARN_OCCUPIED.format(n=shown["occupied_cells"]))
    if shown["unseen_share"] is not None and shown["unseen_share"] > planner.MAX_UNSEEN_SHARE:
        lines.append(WARN_UNSEEN.format(share=shown["unseen_share"], limit=planner.MAX_UNSEEN_SHARE))
    lines.append(RUN_HINT)
    return "\n".join(lines)


class _Conversation:
    """One command being made ready on its own thread: demo_drive.prepare, whose ask() puts the question out and
    waits for the page's next message. Replies (kind, text, Prepared | None) come out of `replies` in order."""

    def __init__(self, parser, text, speed, yaw_rate):
        self.answers = queue.Queue()
        self.replies = queue.Queue()
        self.notes = []                         # what prepare said (why nothing can be driven, ...)
        self._parser = parser
        self._told_rules = False                # the note that the rules read the command: said once
        self.thread = threading.Thread(target=self._run, args=(parser, text, speed, yaw_rate), daemon=True)
        self.thread.start()

    def _rules_note(self):
        """[the note that the rules, not the LLM, read this command] the first time it is so, else []."""
        if self._told_rules or not self._parser.llm_failure:
            return []
        self._told_rules = True
        return [NOTE_RULES.format(why=goto.short_error(self._parser.llm_failure))]

    def _ask(self, question):
        self.replies.put((REPLY_ASK, "\n".join(self._rules_note() + [question]), None))
        return self.answers.get()

    def _run(self, parser, text, speed, yaw_rate):
        try:
            prepared = demo_drive.prepare(parser, text, speed, yaw_rate, ask=self._ask, say=self.notes.append)
        except Exception as exc:                # MC / MV missing or refusing, an LLM error with no fallback
            logger.exception("command %r not prepared", text)
            self.replies.put((REPLY_ERROR, ERROR_TEXT.format(error=exc), None))
            return
        self.notes[:0] = self._rules_note()
        if prepared is None:
            self.replies.put((REPLY_NONE, "\n".join(self.notes) or CANCELLED, None))
        else:
            self.replies.put((REPLY_PLAN, "", prepared))

    def cancel(self):
        """An asking thread gets '' - the user gave up - and ends; a thread still reading ends on its own."""
        self.answers.put("")


class ChatSession:
    """The page's dialogue: one command at a time, a question's answer is the next message. plan: the last
    Prepared the dialogue produced, until it runs (take_plan) or a new command starts."""

    def __init__(self, provider=PROVIDER_AUTO, speed=cfg.CRUISE_SPEED_M_S, yaw_rate=cfg.YAW_RATE_RAD_S):
        self.provider, self.speed, self.yaw_rate = provider, speed, yaw_rate
        self.plan = None
        self._conversation = None               # the one waiting for an answer, else None
        self._lock = threading.Lock()

    def say(self, text):
        """The page's message -> reply {type, text, plan (preview(), for a plan)}."""
        with self._lock:
            conversation = self._conversation
            if conversation is not None:
                conversation.answers.put(text)
            else:
                self.plan = None                # a new command: the plan not run is dropped
                if split_map_request(text)[0]:
                    return {"type": REPLY_NONE, "text": NO_MAPPING, "plan": None}
                try:
                    # A parser per command: the LLM exchange and the rules' pending question are the command's own.
                    parser = CommandParser(self.provider)
                except ParserSetupError as exc:
                    return {"type": REPLY_ERROR, "text": ERROR_TEXT.format(error=exc), "plan": None}
                conversation = _Conversation(parser, text, self.speed, self.yaw_rate)
            try:
                kind, message, prepared = conversation.replies.get(timeout=REPLY_TIMEOUT_S)
            except queue.Empty:
                conversation.cancel()
                self._conversation = None
                return {"type": REPLY_ERROR, "text": TIMED_OUT.format(seconds=REPLY_TIMEOUT_S), "plan": None}
            self._conversation = conversation if kind == REPLY_ASK else None
            logger.debug("chat %r -> %s", text, kind)
            if kind != REPLY_PLAN:
                return {"type": kind, "text": message, "plan": None}
            self.plan = prepared
            shown = preview(prepared, map_store.load_latest())
            return {"type": REPLY_PLAN, "text": _plan_text(conversation.notes, shown, prepared.target), "plan": shown}

    def intent(self, text):
        """INTENT_RUN / INTENT_GO_ON when the whole message is one of RUN_WORDS / GO_ON_WORDS and no question
        waits for an answer (then it is the answer), else None: a command."""
        if self._conversation is not None:
            return None
        words = " ".join(fold(text).translate(str.maketrans("", "", PUNCTUATION)).split())
        if words in RUN_WORDS:
            return INTENT_RUN
        return INTENT_GO_ON if words in GO_ON_WORDS else None

    def go_on(self, target):
        """A plan from the robot's pose now to `target` (Prepared.target of a run that stopped short): its spot
        first, the object's other spots if no route reaches it. Reply as say()'s."""
        with self._lock:
            if self._conversation is not None:
                self._conversation.cancel()
            self._conversation, self.plan = None, None
            if target is None:
                return {"type": REPLY_NONE, "text": NOTHING_TO_GO_ON, "plan": None}
            scene = map_store.load_latest()
            spot = None
            if scene is not None and scene["map_id"] == target["map_id"]:
                spot = next((s for s in build_candidates(scene) if s["id"] == target["spot_id"]), None)
            if spot is None:
                return {"type": REPLY_NONE, "text": GO_ON_MAP_CHANGED, "plan": None}
            notes = [GO_ON_NOTE]
            planned = demo_drive.plan_to_spots(scene, goto.with_alternatives(scene, spot), self.speed, self.yaw_rate,
                                               say=notes.append)
            if planned is None:
                return {"type": REPLY_NONE, "text": "\n".join(notes), "plan": None}
            plan, goal, from_pose, new_target = planned
            result = ParseResult(goal=target["name"], provider=PROVIDER_GO_ON)
            self.plan = demo_drive.Prepared(result, plan, GO_ON_COMMAND.format(name=new_target["name"]), goal, False,
                                            from_pose, new_target)
            logger.debug("go on to %s: %d legs", new_target["spot_id"], len(plan.legs))
            shown = preview(self.plan, scene)
            return {"type": REPLY_PLAN, "text": _plan_text(notes, shown, new_target), "plan": shown}

    def reset(self):
        """Forgets the command in progress and the plan not run."""
        with self._lock:
            if self._conversation is not None:
                self._conversation.cancel()
            self._conversation, self.plan = None, None

    def take_plan(self):
        """The plan to run, now spent: a second press of Run never drives it twice. None when there is none."""
        with self._lock:
            plan, self.plan = self.plan, None
            return plan

    def give_back(self, plan):
        """A plan take_plan handed out that could not start: it is the plan to run again, unless a newer one
        came meanwhile."""
        with self._lock:
            if self.plan is None:
                self.plan = plan


def _tail(path, lines):
    """The last lines of a text file ('' when there is none), streamed: polled every second while it grows."""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return "".join(collections.deque(f, maxlen=lines))
    except OSError:
        return ""


def _read_text(path):
    """A small text file's content ('' when there is none)."""
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read().strip()
    except OSError:
        return ""


def _read_run(run_dir):
    """The run's run.json, or None (not written yet: demo_drive refused the plan before running it)."""
    try:
        with open(os.path.join(run_dir, demo_drive.RUN_JSON), encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _leg_words(kind, value):
    """A leg in the chat's words: 'quay phải 122°', 'đi thẳng 0.55 m' (value: rad / m, signed)."""
    if kind == ROTATE:
        return f"{SAY_ACTION[ACTION_TURN_LEFT if value > 0 else ACTION_TURN_RIGHT]} {abs(math.degrees(value)):.0f}°"
    return f"{SAY_ACTION[ACTION_FORWARD if value > 0 else ACTION_BACKWARD]} {abs(value):.2f} m"


def _measured_words(kind, value):
    """What the camera measured along a leg, signed as the leg: '-114°' (left +), '+0.52 m' (forward +)."""
    return f"{math.degrees(value):+.0f}°" if kind == ROTATE else f"{value:+.2f} m"


def leg_report(legs, run):
    """What each leg of the plan did, from a --compensate run's run.json (requests, executed_legs, in the plan's
    order): done / short, with what the camera measured and why it stopped, then the legs never run. [] for a
    run with no per-leg record (not --compensate)."""
    requests, executed = run.get("requests") or [], run.get("executed_legs") or []
    if not requests:
        return []
    lines = []
    for k, (kind, value) in enumerate(legs):
        if k >= len(requests):
            lines.append(LEG_NOT_RUN.format(mark=MARK_NOT_RUN, leg=_leg_words(kind, value)))
            continue
        request = requests[k]
        done = request.get("outcome") == OUTCOME_OK
        line = LEG_MEASURED.format(mark=MARK_DONE if done else MARK_SHORT, leg=_leg_words(kind, value),
                                   measured=_measured_words(kind, request.get("measured", 0.0)))
        why = [OUTCOME_WORDS.get(request.get("outcome"), request.get("outcome", ""))]
        numbers = request.get("legs") or []
        cut = sum(1 for n in numbers if 0 < n <= len(executed) and executed[n - 1].get("cut_short"))
        if cut:
            why.append(LEG_CUT.format(cut=cut, runs=len(numbers)))
        why = [w for w in why if w]
        lines.append(line + (LEG_WHY.format(why=", ".join(why)) if why else ""))
    logger.debug("leg report: %d legs planned, %d requested", len(legs), len(requests))
    return lines


def _deg(theta):
    """A heading (rad) in degrees, (-180, 180]."""
    return math.degrees(math.atan2(math.sin(theta), math.cos(theta)))


def _off_target_m(reply, target):
    """How far the run's end pose is from the target's spot (m), or None: no target, no end pose, another map."""
    pose = reply.get("end_pose")
    if not target or not pose or target.get("map_id") != pose.get("map_id"):
        return None
    return math.hypot(pose["x"] - target["x"], pose["y"] - target["y"])


def said(reply, target, dry_run):
    """What the chat says when a run ends: arrived at the target (within ARRIVE_TOL_M of its spot, on the map it
    was planned on) or how far off, stopped, failed, or never run. reply: RunManager.status() of a finished run."""
    result, pose = reply.get("result"), reply.get("end_pose")
    what = f" {target['name']}" if target else ""
    if result is None:                              # demo_drive ended before its run began: its log says why
        lines = [line for line in reply.get("log", "").splitlines() if line.startswith(ERROR_WORD)]
        return SAID_NOT_RUN.format(why=lines[-1] if lines else EXIT_TEXT.format(code=reply.get("exit_code")))
    if result == demo_drive.RESULT_CANCELLED:
        return SAID_CANCELLED
    if result == demo_drive.RESULT_COMPLETE and dry_run:
        return SAID_DRY
    if result != demo_drive.RESULT_COMPLETE:
        text = SAID_STOPPED.format(what=what) if result == demo_drive.RESULT_ABORTED else \
            SAID_FAILED.format(what=what, result=result)
        if pose:
            return text + SAID_WHERE.format(x=pose["x"], y=pose["y"], deg=_deg(pose["theta"]),
                                            note=SAID_PREDICTED if pose.get("predicted") else SAID_MEASURED)
        return text + ("" if dry_run else SAID_WHERE_UNKNOWN)   # a dry run never moves the pose
    if not pose:
        return SAID_DONE_NO_POSE + SAID_WHERE_UNKNOWN
    x, y = pose["x"], pose["y"]
    note = SAID_PREDICTED if pose.get("predicted") else ""
    off_m = _off_target_m(reply, target)
    if off_m is None:
        return SAID_DONE.format(x=x, y=y, deg=_deg(pose["theta"])) + note
    logger.debug("run ended %.3f m from %s (tolerance %.2f m)", off_m, target.get("spot_id"), ARRIVE_TOL_M)
    text = SAID_ARRIVED if off_m <= ARRIVE_TOL_M else SAID_SHORT
    return text.format(name=target["name"], x=x, y=y, cm=off_m * cfg.CM_PER_M) + note


class RunManager:
    """demo_drive runs started from the page, one at a time: `demo_drive --yes --plan-json <run>/approved_plan.json
    --run-dir <run> [--compensate]` in a process group of its own. stop() sends that group SIGINT - the terminal's
    Ctrl-C, which robot_link turns into abort + S, after which demo_drive closes the video and saves run.json.
    The recorder is in a session of its own and is stopped by demo_drive, as at the terminal. dry_run: the plan's
    timing is played with no robot and no camera (the page tried without the robot; the tests)."""

    def __init__(self, output_dir=cfg.OUTPUT_DIR, dry_run=False):
        self.output_dir, self.dry_run = output_dir, dry_run
        self._lock = threading.Lock()
        self._proc = None
        self._log = None
        self._run_dir = None
        self._started = None
        self._stopping = False
        self._target = None                     # the running plan's Prepared.target
        self._legs = []                         # its legs [(ROTATE | FORWARD, value)], the closing STOP left out

    @property
    def running(self):
        return self._proc is not None and self._proc.poll() is None

    def _new_run_dir(self):
        """vision/output/<date_time>, created; a second run in the same second gets a suffix."""
        stem = os.path.join(self.output_dir, datetime.now().strftime(demo_drive.RUN_ID_FORMAT))
        path, count = stem, 1
        while True:
            try:
                os.makedirs(path)
                return path
            except FileExistsError:
                count += 1
                path = f"{stem}_{count}"

    def start(self, prepared, compensate):
        """Starts demo_drive on prepared. Returns the run's id; RunBusy while a run is going."""
        with self._lock:
            if self.running:
                raise RunBusy(f"run {os.path.basename(self._run_dir)} is still going")
            if self._log is not None:
                self._log.close()
            run_dir = self._new_run_dir()
            plan_path = os.path.join(run_dir, APPROVED_PLAN_FILE)
            with open(plan_path, "w", encoding="utf-8") as f:
                json.dump(demo_drive.plan_json(prepared), f, ensure_ascii=False, indent=1)
            cmd = [sys.executable, DEMO_DRIVE_SCRIPT, "--yes", "--plan-json", plan_path, "--run-dir", run_dir]
            if self.dry_run:
                cmd += ["--dry-run", "--no-vision"]    # --compensate measures with the camera: not here
            elif compensate:
                cmd.append("--compensate")
            self._log = open(os.path.join(run_dir, CONSOLE_LOG), "w", encoding="utf-8")
            try:
                self._proc = subprocess.Popen(cmd, cwd=COMM_DIR, stdin=subprocess.DEVNULL, stdout=self._log,
                                              stderr=subprocess.STDOUT, start_new_session=True)
            except OSError:
                self._log.close()
                self._log, self._proc = None, None
                raise
            self._run_dir, self._started, self._stopping = run_dir, time.time(), False
            self._target = prepared.target
            self._legs = [(leg.kind, leg.value) for leg in prepared.plan.legs if leg.kind != STOP]
            logger.info("run %s started: %s", os.path.basename(run_dir), " ".join(cmd))
            return os.path.basename(run_dir)

    def stop(self):
        """SIGINT to the run's process group, once. False when nothing runs."""
        with self._lock:
            if not self.running:
                return False
            if not self._stopping:
                try:
                    os.killpg(self._proc.pid, signal.SIGINT)   # start_new_session: its pid is the group's id
                except ProcessLookupError:                     # it ended just now
                    return False
                self._stopping = True
                logger.info("run %s: stop sent", os.path.basename(self._run_dir))
            return True

    def status(self):
        """{state, run_id, line (the run's status.txt), log (its console's last lines), elapsed_s, target (the
        plan's Prepared.target | None); once finished: exit_code, result, end_pose, map (the last map event:
        {event, ...} | None), video, message (what the chat says: said())}."""
        with self._lock:
            if self._proc is None:
                return {"state": STATE_IDLE, "run_id": None}
            exit_code = self._proc.poll()
            if exit_code is None:
                state = STATE_STOPPING if self._stopping else STATE_RUNNING
            else:
                state = STATE_FINISHED
                if self._log is not None:
                    self._log.close()
                    self._log = None
            run_dir, stopped, target, legs = self._run_dir, self._stopping, self._target, self._legs
            reply = {"state": state, "run_id": os.path.basename(run_dir), "target": target,
                     "elapsed_s": round(time.time() - self._started, ELAPSED_DIGITS),
                     "line": _read_text(os.path.join(run_dir, cfg.RECORDER_STATUS_FILE)),
                     "log": _tail(os.path.join(run_dir, CONSOLE_LOG), LOG_TAIL_LINES), "exit_code": exit_code}
        if state == STATE_FINISHED:
            run = _read_run(run_dir)
            # No run.json: demo_drive ended before its run began (it saves one whatever happens once it has) - so
            # nothing was driven. Stopped then (still starting up: killed by the SIGINT, or interrupted in its
            # imports), the run was cancelled; otherwise the log's last lines say why (a plan refused).
            result = demo_drive.RESULT_CANCELLED if run is None and stopped else None
            run = run or {}
            events = [e for e in run.get("events", []) if e.get("event") in MAP_EVENTS]
            reply.update({"result": result or run.get("result"), "end_pose": run.get("end_pose"),
                          "video": run.get("video"), "map": events[-1] if events else None})
            off_m = _off_target_m(reply, target)
            # Short of the target, where the robot now is known: the chat offers the rest of the way (go_on).
            reply["go_on"] = bool(off_m is not None and off_m > ARRIVE_TOL_M and not self.dry_run
                                  and reply["result"] != demo_drive.RESULT_CANCELLED)
            lines = [said(reply, target, self.dry_run)]
            if reply["result"] != demo_drive.RESULT_COMPLETE or reply["go_on"]:
                lines += leg_report(legs, run)
            if reply["go_on"] and reply["result"] != demo_drive.RESULT_ABORTED:   # Stop: the user's; no offer
                lines.append(SAID_GO_ON_HINT)
            reply["message"] = "\n".join(lines)
        return reply

    def unreached(self):
        """The target of the last run, when it finished short of it with the robot's pose known (status go_on),
        else None: what 'đi tiếp' goes on to."""
        status = self.status()
        return status.get("target") if status.get("go_on") else None
