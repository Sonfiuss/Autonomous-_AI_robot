"""Text command -> LLM -> wheels, with the camera filming: the whole chain in one demo.

  python3 communication/demo_drive.py "đi thẳng 30 cm. sau đó rẽ trái, đi tiếp 60 cm"
  python3 communication/demo_drive.py "đi tới cái ghế"    # a place: the real room's map (realroom/) plans it
  python3 communication/demo_drive.py --dry-run "..."     # everything except the UART
  python3 communication/demo_drive.py                      # asks for the command
  python3 communication/demo_drive.py --compensate "lùi 40 cm"   # each leg measured by the camera, topped up

Printed stage by stage before anything moves:
  [1] parser   the raw reply of the LLM (or of the rule parser)
  [2] steps    after the validator, every default spelled out
  [3] legs     primitive -> the UART line SEQ sends -> duration
  [4] wheels   per leg and wheel: DM556 pulses (sign = DIR), peak pulse rate, peak wheel speed
The recorder (vision/record.py) starts loading while the plan is shown. After Enter,
`robot_link --run-plan` drives the plan one leg at a time (the next leg only after the firmware's K),
and the recorder is stopped VIDEO_TAIL_S after the robot. Ctrl-C reaches robot_link directly, which
aborts and sends S; the video is still closed properly. Everything lands in vision/output/<run_id>/:
plan.txt, run.json, detections.mp4, detections.jsonl, recorder.log, robot_link.log.

A place to go to (the parser's "goal") is grounded on the real room's map and routed there (goto.py); the
legs then run like any command's. With a map, run.json records the robot's pose on it before and after,
and the map is moved by what the run did (_map_update).

--compensate (task 2026-09-27_explore-map Pha 1' C): leg_executor drives the legs one at a time instead,
each commanded at target / k of the floor (slip_model), measured by the floor odometry of the recording
and topped up while it is off by more than 3 cm / 2 deg. It prints what every leg really did and the slip
table; run.json adds executed_legs and requests. Needs the recorder, so not with --no-vision / --dry-run.

The room's map (task 2026-09-30_realroom-chat-drive): "lập bản đồ" / "quét phòng" in the command, or --map,
builds the map of this run once it is over (vision/drive_map.py --run <run> --publish, map_build.log) and
makes it the real room's - realroom's page shows it at once. Alone it is the scan (cmd_parser.MAP_SCAN, turn in
place left 90 / right 180 / left 90); with another move, that move is driven and filmed instead.
  python3 communication/demo_drive.py --compensate "lập bản đồ"
  python3 communication/demo_drive.py --compensate --map "quay trái 360 độ"
--plan-json FILE drives a plan approved elsewhere (plan_json(): realroom's web chat writes it) with no parser
and no question; a route in it is refused once the robot or the map changed since it was planned. --run-dir
names the run's folder, so whoever started the run can follow it (status.txt, run.json).
"""
import argparse
import collections
import fcntl
import json
import logging
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

import drive_config as cfg
from cmd_parser import (ACTIONS, LINEAR_ACTIONS, LLM_PROVIDERS, PROVIDER_AUTO, PROVIDER_MAP_SCAN, PROVIDER_RULE,
                        CommandParser, MotionStep, ParserSetupError, ParseResult, check_intents, map_scan_steps,
                        split_map_request, track_steps)
from motion_plan import FORWARD, ROTATE, STOP, PlanError, build_plan, plan_from_legs
from robot_link_cli import COMPLETE_MARK, FAILED_MARK, LINE_BUFFERED, PROGRESS_RE, flush_link
from slip_model import SlipModel, kind_of

cfg.add_realroom_to_path()
import map_store  # noqa: E402  (realroom module, path set just above)

RESULT_COMPLETE = "complete"
RESULT_FAILED = "failed"
RESULT_ABORTED = "aborted"
RESULT_CANCELLED = "cancelled"
MODE_DRY_RUN = "dry-run"
MODE_ROBOT = "robot"
RUN_ID_FORMAT = "%Y%m%d_%H%M%S"
RUN_JSON = "run.json"
PLAN_FILE = "plan.txt"
RECORDER_LOG = "recorder.log"
ROBOT_LINK_LOG = "robot_link.log"
MAP_BUILD_LOG = "map_build.log"
STATUS_WAITING = "waiting to start"
STATUS_DONE = "done: {outcome}"
STATUS_MAP_BUILDING, STATUS_MAP_PUBLISHED, STATUS_MAP_FAILED = "building map", "map published", "map failed"
EVENT_MAP_PUBLISHED, EVENT_MAP_FAILED, EVENT_MAP_SKIPPED = "map_published", "map_failed", "map_skipped"
MAP_SKIP_DRY_RUN = "dry run: nothing moved"
MAP_SKIP_INCOMPLETE = "the run did not complete"
MAP_SKIP_STOPPED = "stopped by the user"
STOP_REQUESTED = "stop_requested"                       # run.json: Ctrl-C / Stop came during the run
MAP_FAIL_TIMEOUT, MAP_FAIL_STOPPED = "timed out", "stopped"
TIME_DIGITS = 3                                         # wall-clock seconds in run.json (ms)
POSE_MATCH_TOL = 1e-3                                   # m and rad: latest.json keeps a pose to 1e-4
EXIT_OK = 0
EXIT_FAILED = 1                                         # the run started and did not complete
EXIT_ERROR = 2                                          # nothing ran: setup or input problem
POLL_S = 0.2
QUIT_WORDS = ("q", "quit", "exit")
RULE = "-" * 78

# One command made ready to drive. result: the ParseResult (provider and raw go to run.json); plan: MotionPlan;
# text: every message of it joined; goal: what a place resolved to ("" for steps); wants_map: build and publish
# the run's map afterwards; from_pose: {map_id, x, y, theta} a route was planned from (None for steps) - it may
# only be driven from there; target: where a route ends, {map_id, spot_id, object_id, name (the chat's words:
# goto.describe), x, y, theta} (None for steps) - to say whether the robot got there.
Prepared = collections.namedtuple("Prepared", "result plan text goal wants_map from_pose target", defaults=(None,))

logger = logging.getLogger("demo_drive")


class RunLog:
    """The run folder, its timeline, and the status line the recorder draws on the video."""

    def __init__(self, run_dir):
        self.run_dir = run_dir
        self.events = []
        self.status_path = os.path.join(run_dir, cfg.RECORDER_STATUS_FILE)
        self.data = {"run_id": os.path.basename(run_dir), "events": self.events}

    def path(self, name):
        return os.path.join(self.run_dir, name)

    def event(self, name, **detail):
        self.events.append(dict({"t": round(time.time(), TIME_DIGITS), "event": name}, **detail))

    def status(self, text):
        """ASCII only: the recorder draws it with a Hershey font."""
        tmp = self.status_path + ".tmp"
        with open(tmp, "w", encoding="ascii", errors="replace") as f:
            f.write(text)
        os.replace(tmp, self.status_path)      # atomic: the recorder never reads half a line
        self.event("status", text=text)

    def save(self):
        with open(self.path(RUN_JSON), "w", encoding="utf-8") as f:
            json.dump(self.data, f, ensure_ascii=False, indent=2)


class RecorderProcess:
    """vision/record.py in its own session: terminal Ctrl-C does not reach it, stop() does."""

    def __init__(self, runlog, extra_args):
        self.runlog = runlog
        self.ready_path = runlog.path(cfg.RECORDER_READY_FILE)
        self.args = [sys.executable, cfg.RECORDER_SCRIPT, "--out", runlog.run_dir,
                     "--status-file", runlog.status_path, "--ready-file", self.ready_path] + extra_args
        self.proc = None
        self._log = None

    def start(self):
        self._log = open(self.runlog.path(RECORDER_LOG), "w", encoding="utf-8")
        try:
            self.proc = subprocess.Popen(self.args, cwd=cfg.VISION_DIR, stdout=self._log,
                                         stderr=subprocess.STDOUT, start_new_session=True)
        except OSError:
            self._log.close()
            raise
        self.runlog.event("recorder_started", pid=self.proc.pid)

    def video_path(self):
        """The recorder writes the video's path into its ready file; None if it never got a frame."""
        try:
            with open(self.ready_path, encoding="utf-8") as f:
                return f.read().strip() or None
        except OSError:
            return None

    def wait_ready(self, timeout_s):
        """True once the first frame is in the video; False if it died or timed out."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if os.path.exists(self.ready_path):
                self.runlog.event("recorder_ready")
                return True
            if self.proc.poll() is not None:
                return False
            time.sleep(POLL_S)
        return False

    def stop(self):
        if self.proc is None:
            return None
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)   # the recorder closes the video, then exits
            try:
                self.proc.wait(timeout=cfg.RECORDER_STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                logger.warning("recorder did not stop in %.0f s: killing it", cfg.RECORDER_STOP_TIMEOUT_S)
                self.proc.kill()
                self.proc.wait()
        self._log.close()
        self.runlog.event("recorder_stopped", exit_code=self.proc.returncode)
        return self.proc.returncode


# ------------------------------------------------------------------ stages
def leg_label(plan, index):
    """'leg 2/5  T 90.000 0.800  (turn_left 90.0 deg)': the status line of a leg (vision/drive_timeline reads it)."""
    leg = plan.legs[index]
    if leg.kind == STOP:
        what = "stop"
    elif leg.step_index < 0:                   # a route to a place: its legs come from no step of the command
        what = "route"
    else:
        what = plan.steps[leg.step_index].describe()
    return f"leg {index + 1}/{len(plan.legs)}  {leg.wire}  ({what})"


def _print_stages(prepared, model=""):
    result, plan, text, goal = prepared.result, prepared.plan, prepared.text, prepared.goal
    print(RULE)
    print(f"[1] Parser ({result.provider}{', ' + model if model else ''}):")
    print("    " + json.dumps(result.raw, ensure_ascii=False))
    if goal:
        print(f"[2] Đi tới '{result.goal}' trên bản đồ thật: {goal}")
    else:
        print("[2] Các bước sau validator (hướng / vị trí tích luỹ theo lệnh, từ chỗ bắt đầu):")
    for i, (step, (heading, x, y)) in enumerate(zip(plan.steps, track_steps(plan.steps)), 1):
        print(f"    {i}. {step.describe():<28} -> hướng {heading:+.0f}°, x {x * cfg.CM_PER_M:+.0f} cm, "
              f"y {y * cfg.CM_PER_M:+.0f} cm")
    for ok, number, words in check_intents(plan.steps, text):
        print(f"    ✓ '{words}': sau bước {number} robot về lại ban đầu" if ok else
              f"    ! '{words}': không bước nào đưa robot về lại ban đầu - kiểm tra lại lệnh")
    print(f"[3] Primitive -> lệnh UART (SEQ gửi từng dòng, chờ K):   "
          f"tốc độ {plan.cruise_speed:g} m/s, quay {plan.yaw_rate:g} rad/s")
    print(f"    {'#':>2}  {'primitive':<20} {'UART':<18} {'thời gian':>9}")
    for i, leg in enumerate(plan.legs, 1):
        print(f"    {i:>2}  {leg.plan_line():<20} {leg.wire:<18} {leg.duration_s:8.2f}s")
    print(f"    tổng {plan.duration_s:.1f} s")
    print("[4] Thông số bánh mỗi leg (MC = cùng chuỗi RM với firmware; bước có dấu = mức DIR):")
    wheels = len(next((leg.wheels for leg in plan.legs if leg.wheels), ()))
    header = " | ".join(f"W{k + 1}: {'bước':>7} {'Hz đỉnh':>7} {'rad/s':>6}" for k in range(wheels))
    print(f"    {'#':>2}  {'UART':<18} {header}")
    for i, leg in enumerate(plan.legs, 1):
        if not leg.wheels:
            continue
        cells = " | ".join(f"    {w.steps:+7d} {w.peak_hz:7.0f} {w.peak_rad_s:+6.2f}" for w in leg.wheels)
        print(f"    {i:>2}  {leg.wire:<18} {cells}")
    if prepared.wants_map:
        print("[5] Chạy xong: dựng bản đồ từ video của lần chạy này và dùng nó làm bản đồ phòng thật "
              "(thay bản đồ hiện tại)")
    print(RULE)


# ------------------------------------------------------------------ steps of a run
def _ask(prompt):
    """input() that treats a closed stdin as 'no answer'."""
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def console_ask(question):
    """A question to the user at the terminal -> the answer ('' = gave up)."""
    return _ask(f"? {question}\n> ")


def _parse_until_steps(parser, text, ask):
    """Asks the parser's questions back through ask(question) -> answer until there are steps or a place to
    go to. Returns (ParseResult, the answers given), or (None, ...) when the user gave up."""
    answers = []
    while True:
        result = parser.parse(text)
        if result.steps or result.goal:
            return result, answers
        text = ask(result.question).strip()
        if not text or text.lower() in QUIT_WORDS:
            return None, answers
        answers.append(text)


def _drive_dry(plan, runlog):
    """Plays the plan's timing without a robot, so the video and run.json look like a real run."""
    for index, leg in enumerate(plan.legs):
        runlog.status(leg_label(plan, index))
        print(f"  [dry-run] {leg_label(plan, index)}")
        time.sleep(leg.duration_s)
    return RESULT_COMPLETE, EXIT_OK


def _drive_compensated(plan, args, runlog):
    """leg_executor over the plan's legs, one at a time; stops at the first leg that cannot go on."""
    import leg_executor                    # vision/ + OpenCV: only a compensated run pulls them in
    executor = leg_executor.LegExecutor(args.port, runlog, plan.cruise_speed, plan.yaw_rate)
    result, outcome = RESULT_COMPLETE, None
    for leg in plan.legs:
        if leg.kind == STOP:
            continue
        outcome = executor.run(leg.kind, leg.value, leg.step_index).outcome
        # A stuck leg ends the command too: the steps after it assume the robot got where it was sent.
        if outcome in leg_executor.STOPPING or outcome == leg_executor.STUCK:
            result = RESULT_ABORTED if outcome == leg_executor.ABORTED else RESULT_FAILED
            break
    x, y, theta = executor.pose
    print(f"  vị trí đo được: x {x * cfg.CM_PER_M:+.1f} cm, y {y * cfg.CM_PER_M:+.1f} cm, hướng {math.degrees(theta):+.1f}°")
    print("\n".join(executor.slip.table()))
    # A leg that failed, lost the camera or was aborted moved the robot by an unknown amount (the ESP32 may
    # finish it alone): no measured pose then, so the map is not moved (_map_update warns instead).
    if outcome not in leg_executor.STOPPING:
        runlog.data["measured_pose"] = [round(v, leg_executor.RAD_DIGITS) for v in executor.pose]
    return result, None                    # several robot_link calls: their exit codes are in the events


def _drive_robot(plan_path, plan, args, runlog):
    """robot_link --run-plan, its output teed to the console and robot_link.log."""
    flush_link(args.port, runlog)
    cmd = list(LINE_BUFFERED) + [cfg.ROBOT_LINK_BIN, "--port", args.port, "--run-plan", plan_path,
                                 "--speed", str(plan.cruise_speed), "--yaw-rate", str(plan.yaw_rate)]
    runlog.event("robot_link_started", cmd=cmd)
    result = RESULT_FAILED
    with open(runlog.path(ROBOT_LINK_LOG), "w", encoding="utf-8") as log:
        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    universal_newlines=True, bufsize=1)
        except OSError as exc:                 # stdbuf or robot_link missing: nothing was sent
            print(f"Lỗi: không chạy được robot_link: {exc}")
            runlog.event("robot_link_failure", line=str(exc))
            return RESULT_FAILED, None
        try:
            for line in proc.stdout:
                log.write(line)
                print("  [robot_link] " + line.rstrip())
                # The failure line names a primitive too, so the end marks are matched first.
                match = PROGRESS_RE.search(line)
                if COMPLETE_MARK in line:
                    result = RESULT_COMPLETE
                elif FAILED_MARK in line:
                    runlog.event("robot_link_failure", line=line.strip())
                elif match and int(match.group(1)) <= len(plan.legs):
                    runlog.status(leg_label(plan, int(match.group(1)) - 1))
            proc.wait()
        except KeyboardInterrupt:
            # The same Ctrl-C reached robot_link: it aborts the plan and sends S. Let it finish that.
            result = RESULT_ABORTED
            print("\n  Ctrl-C: robot_link đang dừng robot (S)...")
            try:
                remaining, _ = proc.communicate(timeout=cfg.ROBOT_LINK_GRACE_S)
                log.write(remaining or "")
            except subprocess.TimeoutExpired:
                logger.error("robot_link did not exit after Ctrl-C: killing it")
                proc.kill()
                proc.wait()
    if proc.returncode != 0 and result == RESULT_COMPLETE:
        result = RESULT_FAILED
    runlog.event("robot_link_exited", exit_code=proc.returncode)
    return result, proc.returncode


def _map_start(runlog):
    """Records where the real room's map has the robot as this run starts (run.json start_pose); None and
    nothing recorded when no map was published."""
    scene = map_store.load_latest()
    if scene is None:
        return None
    x, y, theta = map_store.robot_pose(scene)
    runlog.data["start_pose"] = {"map_id": scene["map_id"], "x": x, "y": y, "theta": theta}
    return scene


def _predicted_motion(plan):
    """(dx, dy, dtheta) the plan's legs come to at the floor's slip ratios - where a run nobody measured ended."""
    slip, motion = SlipModel(), (0.0, 0.0, 0.0)
    for leg in plan.legs:
        if leg.kind == STOP:
            continue
        amount = leg.value * slip.k(kind_of(leg.kind, leg.value))
        motion = map_store.compose(motion, (0.0, 0.0, amount) if leg.kind == ROTATE else (amount, 0.0, 0.0))
    return motion


def _map_update(runlog, plan, outcome, compensated):
    """Moves the robot in the real room's map by what this run did: the camera's measure when compensated
    (flagged predicted only where a leg went unmeasured), else the plan at the slip ratios (flagged predicted). A run that stopped early is not placed - where it
    stopped is unknown (the link can die while the ESP32 drives on) - and says so."""
    if outcome != RESULT_COMPLETE and not (compensated and "measured_pose" in runlog.data):
        print("  ! vị trí robot trong bản đồ thật KHÔNG cập nhật (lệnh không chạy hết): "
              "đặt lại robot hoặc dựng lại bản đồ")
        return
    if compensated:
        import leg_executor                # already loaded: the compensated run drove through it
        motion = tuple(runlog.data["measured_pose"])
        # Measured, even for a run that stopped early (stuck, off target), when every leg it drove was acked and
        # measured; predicted when one was not (weak odometry). A request that ended aborted / failed may have moved
        # the robot unmeasured - _drive_compensated keeps no measured_pose then, so it never gets here; checked
        # all the same.
        requests = runlog.data.get("requests", [])
        predicted = any(r.get("predicted") or r.get("outcome") in leg_executor.UNMEASURED_ENDS for r in requests)
    else:
        motion, predicted = _predicted_motion(plan), True
    source = f"run {runlog.data['run_id']}" + ("" if compensated else " (lệnh x k)")
    scene = map_store.move_robot(motion, source, predicted)
    x, y, theta = map_store.robot_pose(scene)
    runlog.data["end_pose"] = {"map_id": scene["map_id"], "x": x, "y": y, "theta": theta, "predicted": predicted}
    print(f"  bản đồ {scene['map_id']}: robot giờ ở x {x:.2f} y {y:.2f} m, hướng "
          f"{math.degrees(math.atan2(math.sin(theta), math.cos(theta))):+.0f}°" + (" (dự đoán)" if predicted else ""))


def _plan_goal(words, parser, speed, yaw_rate, ask, say):
    """A place to go to -> (MotionPlan, what it goes to, the pose it starts from, the target: Prepared.target) on
    the real room's map, or None (said why)."""
    import goto                            # CM grounding + realroom planner: only a place to go pulls them in
    scene = map_store.load_latest()
    if scene is None:
        say("Chưa có bản đồ thật: dựng một bản đồ trước ('lập bản đồ', hoặc "
            "python3 vision/drive_map.py --run vision/output/<run> --publish).")
        return None
    spots, why = goto.ground(scene, words, parser.llm, ask, say)
    if spots is None:
        say(f"Không đi được tới '{words}': {why}")
        return None
    return plan_to_spots(scene, spots, speed, yaw_rate, say)


def plan_to_spots(scene, spots, speed, yaw_rate, say=print):
    """The route to the first of spots (best first) the planner reaches on scene -> (MotionPlan, what it goes
    to, the pose it starts from, Prepared.target), or None (said why). A spot passed over for one further down
    the list is said, with why. Also realroom's 'go on' after a run that stopped short."""
    import goto
    spot, route, tried = goto.route_first(scene, spots)
    if spot is None:
        say("Không có đường tới " + "; ".join(f"{goto.describe(scene, s)} ({why})" for s, why in tried))
        return None
    if tried:
        say(f"Không có đường tới {goto.describe(scene, tried[0][0])} ({tried[0][1]}): đi tới "
            f"{goto.describe(scene, spot)}.")
    x, y, theta = map_store.robot_pose(scene)
    what = (f"{spot['description']} ({spot['id']}), bản đồ {scene['map_id']}: từ ({x:.2f}, {y:.2f}) tới "
            f"({spot['x']:.2f}, {spot['y']:.2f}), đường {route.length_m:.2f} m, sàn chưa thấy {route.unseen_share:.0%}")
    from_pose = {"map_id": scene["map_id"], "x": x, "y": y, "theta": theta}
    target = {"map_id": scene["map_id"], "spot_id": spot["id"], "object_id": spot["target_id"],
              "name": goto.describe(scene, spot), "x": spot["x"], "y": spot["y"], "theta": spot["theta"]}
    return plan_from_legs(route.legs, speed, yaw_rate), what, from_pose, target


def prepare(parser, text, speed, yaw_rate, wants_map=False, ask=console_ask, say=print):
    """A command -> Prepared: a map request split off (split_map_request - alone, it is the scan), the parser's
    and the map's questions asked through ask(question) -> answer ('' = gave up), a place grounded and routed on
    the real room's map. None when the user gave up or there is nowhere to go (said why through say). Raises
    PlanError when MC refuses a leg. The terminal and realroom's web chat both come through here."""
    asked_map, rest = split_map_request(text)
    wants_map = wants_map or asked_map
    logger.debug("map requested: %s (in the words: %s), command left: %r", wants_map, asked_map, rest)
    if wants_map and not rest:
        steps = map_scan_steps()
        result = ParseResult(steps=steps, provider=PROVIDER_MAP_SCAN, raw={"map_scan": [s.to_dict() for s in steps]})
        return Prepared(result, build_plan(steps, speed, yaw_rate), text, "", True, None)
    result, answers = _parse_until_steps(parser, rest, ask)
    if result is None:
        return None
    text = " / ".join([text] + answers)
    if not result.goal:
        return Prepared(result, build_plan(result.steps, speed, yaw_rate), text, "", wants_map, None)
    planned = _plan_goal(result.goal, parser, speed, yaw_rate, ask, say)
    if planned is None:
        return None
    plan, goal, from_pose, target = planned
    return Prepared(result, plan, text, goal, wants_map, from_pose, target)


def plan_json(prepared):
    """What --plan-json reads back (load_plan_json): the command and its parse, the steps - or, for a route,
    its legs and the pose they start from - and whether to build the map afterwards."""
    result = prepared.result
    data = {"command": prepared.text, "provider": result.provider, "raw": result.raw, "goal": result.goal,
            "goal_text": prepared.goal, "map": prepared.wants_map, "from_pose": prepared.from_pose,
            "target": prepared.target}
    if result.goal:
        data["legs"] = [[leg.kind, leg.value] for leg in prepared.plan.legs if leg.kind != STOP]
    else:
        data["steps"] = [step.to_dict() for step in result.steps]
    return data


def _checked_step(item):
    """A plan file's step -> MotionStep, inside the validator's limits (ValueError otherwise)."""
    step = MotionStep(item["action"], float(item["amount"]), bool(item["stated"]))
    low, high = ((cfg.MIN_LINEAR_M, cfg.MAX_LINEAR_M) if step.action in LINEAR_ACTIONS
                 else (cfg.MIN_TURN_DEG, cfg.MAX_TURN_DEG))
    if step.action not in ACTIONS or not low <= step.amount <= high:
        raise ValueError(f"step {item} outside the validator's limits")
    return step


def _checked_leg(item):
    """A plan file's route leg -> (ROTATE | FORWARD, value) (ValueError otherwise)."""
    kind, value = item[0], float(item[1])
    if kind not in (ROTATE, FORWARD) or not math.isfinite(value):
        raise ValueError(f"leg {item} is not ROTATE / FORWARD by a number")
    return kind, value


def load_plan_json(path, speed, yaw_rate):
    """--plan-json: a plan approved elsewhere (plan_json()) -> Prepared, planned again by MC. Raises PlanError
    for a file that is not such a plan."""
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        if "legs" in data:
            legs, steps = [_checked_leg(item) for item in data["legs"]], ()
        else:
            legs, steps = None, tuple(_checked_step(item) for item in data["steps"])
            if not 0 < len(steps) <= cfg.MAX_STEPS:
                raise ValueError(f"{len(steps)} steps")
        from_pose = data.get("from_pose")
        if legs is not None:
            if from_pose is None:
                raise ValueError("a route without the pose it was planned from")
            from_pose = dict(from_pose, **{key: float(from_pose[key]) for key in map_store.POSE_KEYS},
                             map_id=str(from_pose["map_id"]))
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:   # JSONDecodeError is a ValueError
        raise PlanError(f"{path}: not a plan ({exc})") from exc
    result = ParseResult(steps=steps, provider=data.get("provider", ""), raw=data.get("raw"), goal=data.get("goal", ""))
    plan = plan_from_legs(legs, speed, yaw_rate) if legs is not None else build_plan(steps, speed, yaw_rate)
    return Prepared(result, plan, data.get("command", ""), data.get("goal_text", ""), bool(data.get("map")),
                    from_pose, data.get("target"))


def _moved_since(from_pose):
    """Why a route planned from from_pose may not be driven now - the map changed, or the robot moved since -
    or None."""
    scene = map_store.load_latest()
    if scene is None:
        return "không còn bản đồ thật"
    if scene["map_id"] != from_pose["map_id"]:
        return f"bản đồ đã đổi ({from_pose['map_id']} -> {scene['map_id']})"
    x, y, theta = map_store.robot_pose(scene)
    logger.debug("route planned from %s, the robot now at (%.4f, %.4f, %.4f)", from_pose, x, y, theta)
    turn = math.atan2(math.sin(theta - from_pose["theta"]), math.cos(theta - from_pose["theta"]))
    # Written so that a NaN in either pose counts as moved: every comparison with NaN is False.
    if not (math.hypot(x - from_pose["x"], y - from_pose["y"]) <= POSE_MATCH_TOL and abs(turn) <= POSE_MATCH_TOL):
        return "robot đã di chuyển từ khi lập kế hoạch"
    return None


def _preflight(args):
    """Refuses to start a real run that cannot work, before the camera is even opened."""
    if args.compensate and (args.dry_run or args.no_vision):
        return "--compensate đo từng leg bằng camera: không dùng được với --dry-run / --no-vision"
    if args.dry_run:
        return None
    if not os.path.isfile(cfg.ROBOT_LINK_BIN):
        return f"không có {cfg.ROBOT_LINK_BIN}; build: bash motivation/jetson/tools/build_jetson.sh"
    if not os.path.exists(args.port):
        return f"không thấy cổng {args.port} (ESP32 chưa cắm?); dùng --port hoặc --dry-run"
    return None


def _recorder_args(args):
    extra = []
    if args.no_depth:
        extra.append("--no-depth")
    if args.no_floor:
        extra.append("--no-floor")
    if args.color_index is not None:
        extra += ["--color-index", str(args.color_index)]
    return extra


def _map_problem(args, prepared):
    """Why a map request cannot be met (its map is built from the recording), or None."""
    if prepared.wants_map and args.no_vision and not args.dry_run:
        return "lập bản đồ cần camera quay lại lần chạy: bỏ --no-vision"
    return None


def _lock_robot():
    """The robot for this process alone: the open file holding ROBOT_LOCK_FILE's lock until the process ends,
    or None when another demo_drive - from a terminal or realroom's page - has it (both would write to the ESP32).
    OSError when the lock file cannot be opened."""
    os.makedirs(os.path.dirname(cfg.ROBOT_LOCK_FILE), exist_ok=True)
    handle = open(cfg.ROBOT_LOCK_FILE, "w", encoding="utf-8")
    try:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        handle.close()
        return None
    return handle


def _note_stops(runlog):
    """From here on Ctrl-C (realroom's Stop) is noted in run.json's STOP_REQUESTED, not raised: the robot is done
    (or already stopping), and the pose, the video and run.json must be finished whole - raised in the video's
    tail it once moved the pose a second time, in the recorder's stop it lost run.json. _map_after then builds
    no map. _execute restores the KeyboardInterrupt once run.json is saved. The handler only sets the flag: a print
    from it while the main code is printing raises (a reentrant stdout write)."""
    def note(signum, frame):
        runlog.data[STOP_REQUESTED] = True
    signal.signal(signal.SIGINT, note)


def _execute(args, prepared):
    """Films and drives one plan. Returns (the result word, the run's RunLog)."""
    result, plan, text = prepared.result, prepared.plan, prepared.text
    run_dir = args.run_dir or os.path.join(cfg.OUTPUT_DIR, datetime.now().strftime(RUN_ID_FORMAT))
    os.makedirs(run_dir, exist_ok=True)
    runlog = RunLog(run_dir)
    runlog.data.update({"command": text, "provider": result.provider, "parser_raw": result.raw,
                        "mode": MODE_DRY_RUN if args.dry_run else MODE_ROBOT, "port": args.port,
                        "build_map": prepared.wants_map, "plan": plan.to_dict()})
    plan_path = runlog.path(PLAN_FILE)
    with open(plan_path, "w", encoding="utf-8") as f:
        f.write(plan.plan_file_text(text))
    runlog.status(STATUS_WAITING)
    on_map = not args.dry_run and _map_start(runlog) is not None

    recorder = None
    if not args.no_vision:
        recorder = RecorderProcess(runlog, _recorder_args(args))
        recorder.start()
        print(f"Camera + YOLO đang khởi động (log: {runlog.path(RECORDER_LOG)})")

    outcome, exit_code, driving = RESULT_CANCELLED, None, False
    try:
        answer = "" if args.yes else _ask("Nhấn Enter để chạy, 'q' để huỷ: ")
        if not args.yes and answer.lower() in QUIT_WORDS:
            print("Đã huỷ.")
        elif recorder is not None and not recorder.wait_ready(cfg.RECORDER_READY_TIMEOUT_S):
            outcome = RESULT_FAILED
            print(f"Recorder không lên (xem {runlog.path(RECORDER_LOG)}); robot không chạy. "
                  "Dùng --no-vision để chạy không quay.")
        else:
            runlog.event("drive_started")
            driving = True
            if args.dry_run:
                outcome, exit_code = _drive_dry(plan, runlog)
            elif args.compensate:
                outcome, exit_code = _drive_compensated(plan, args, runlog)
            else:
                outcome, exit_code = _drive_robot(plan_path, plan, args, runlog)
            _note_stops(runlog)
            driving = False
            runlog.status(STATUS_DONE.format(outcome=outcome))
            if on_map:
                _map_update(runlog, plan, outcome, args.compensate)
            time.sleep(cfg.VIDEO_TAIL_S)
    except KeyboardInterrupt:
        _note_stops(runlog)
        runlog.data[STOP_REQUESTED] = True
        outcome = RESULT_ABORTED if driving else RESULT_CANCELLED
        runlog.status(STATUS_DONE.format(outcome=outcome))
        print("\nĐã dừng.")
        if on_map and driving:
            _map_update(runlog, plan, outcome, args.compensate)
    finally:
        runlog.data["result"] = outcome
        runlog.data["robot_link_exit"] = exit_code
        runlog.data["video"] = None
        if recorder is not None:
            runlog.data["recorder_exit"] = recorder.stop()
            runlog.data["video"] = recorder.video_path()
        runlog.save()
        signal.signal(signal.SIGINT, signal.default_int_handler)   # the map build may be stopped again
    if runlog.data.get(STOP_REQUESTED) and outcome not in (RESULT_ABORTED, RESULT_CANCELLED):
        print("Ctrl-C khi robot đã xong: không huỷ gì - video đã đóng, run.json đã ghi.")
    print(f"Kết quả: {outcome}.  Thư mục: {run_dir}")
    if runlog.data["video"]:
        print(f"Video: {runlog.data['video']}")
    return outcome, runlog


def build_map(runlog, script=cfg.DRIVE_MAP_SCRIPT):
    """drive_map --publish on this run's recording: its map becomes the real room's, the robot where the run
    ended on it. Logged to map_build.log; run.json event map_published or map_failed. True when published."""
    runlog.status(STATUS_MAP_BUILDING)
    print(f"Đang dựng bản đồ từ video (log: {runlog.path(MAP_BUILD_LOG)})...")
    cmd = [sys.executable, script, "--run", runlog.run_dir, "--publish"]
    map_id = os.path.basename(runlog.run_dir.rstrip("/"))           # map_store.publish_run names the map so
    logger.debug("map build: %s", " ".join(cmd))
    exit_code, why = None, None
    with open(runlog.path(MAP_BUILD_LOG), "w", encoding="utf-8") as log:
        try:
            exit_code = subprocess.run(cmd, cwd=cfg.VISION_DIR, stdout=log, stderr=subprocess.STDOUT,
                                       timeout=cfg.MAP_BUILD_TIMEOUT_S).returncode
        except subprocess.TimeoutExpired:
            why = MAP_FAIL_TIMEOUT
        except KeyboardInterrupt:              # the same Ctrl-C (or the page's Stop) reached drive_map too
            why = MAP_FAIL_STOPPED
        except OSError as exc:
            why = str(exc)
    if exit_code == 0:
        runlog.event(EVENT_MAP_PUBLISHED, map_id=map_id)
        runlog.status(STATUS_MAP_PUBLISHED)
        print(f"Bản đồ mới: {map_id} - giờ là bản đồ phòng thật.")
    else:
        runlog.event(EVENT_MAP_FAILED, exit_code=exit_code, reason=why)
        runlog.status(STATUS_MAP_FAILED)
        why = why or f"drive_map thoát {exit_code}"
        print(f"Không dựng được bản đồ ({why}; xem {runlog.path(MAP_BUILD_LOG)}).")
    runlog.save()
    return exit_code == 0


def _map_after(args, runlog, outcome):
    """After a map run: build_map, unless nothing moved (a dry run's map would replace the room's with a still
    view) or the run ended where nobody knows (the rule _map_update places the robot by). True published,
    False failed, None not built."""
    if args.dry_run:
        why = MAP_SKIP_DRY_RUN
    elif runlog.data.get(STOP_REQUESTED):
        why = MAP_SKIP_STOPPED
    elif outcome != RESULT_COMPLETE and not (args.compensate and "measured_pose" in runlog.data):
        why = MAP_SKIP_INCOMPLETE
    else:
        return build_map(runlog)
    print(f"Không dựng bản đồ: {why}.")
    runlog.event(EVENT_MAP_SKIPPED, reason=why)
    runlog.save()
    return None


def _parse_args():
    parser = argparse.ArgumentParser(description="Text command -> LLM -> robot wheels, with detection video.")
    parser.add_argument("command", nargs="*", help="the command; asked for when omitted")
    parser.add_argument("--provider", default=PROVIDER_AUTO, choices=(PROVIDER_AUTO, PROVIDER_RULE) + LLM_PROVIDERS,
                        help="auto = CM's configured LLM (CM_PROVIDER, keys in project/src/CM/.env), "
                             "rule parser when it is unavailable")
    parser.add_argument("--dry-run", action="store_true", help="everything except the UART")
    parser.add_argument("--yes", action="store_true", help="do not wait for Enter before driving")
    parser.add_argument("--port", default=cfg.DEFAULT_PORT)
    parser.add_argument("--speed", type=float, default=cfg.CRUISE_SPEED_M_S, help="cruise speed, m/s")
    parser.add_argument("--yaw-rate", type=float, default=cfg.YAW_RATE_RAD_S, help="turn rate, rad/s")
    parser.add_argument("--compensate", action="store_true",
                        help="one leg at a time, each measured by the camera and topped up (leg_executor)")
    parser.add_argument("--no-vision", action="store_true", help="do not record")
    parser.add_argument("--no-depth", action="store_true", help="record without the Astra depth")
    parser.add_argument("--no-floor", action="store_true", help="record without the Depth Anything floor")
    parser.add_argument("--color-index", type=int, help="OpenCV index of the Astra RGB camera")
    parser.add_argument("--map", action="store_true",
                        help="build the run's map afterwards and make it the real room's (alone: the scan)")
    parser.add_argument("--plan-json", help="drive this approved plan (plan_json(); realroom's web chat) - no parser")
    parser.add_argument("--run-dir", help="the run's folder (default: vision/output/<date_time>)")
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    args = parser.parse_args()
    if not 0.0 < args.speed <= cfg.MAX_CRUISE_SPEED_M_S:
        parser.error(f"--speed must be in (0, {cfg.MAX_CRUISE_SPEED_M_S}]")
    if not 0.0 < args.yaw_rate <= cfg.MAX_YAW_RATE_RAD_S:
        parser.error(f"--yaw-rate must be in (0, {cfg.MAX_YAW_RATE_RAD_S}]")
    if args.plan_json and args.command:
        parser.error("--plan-json is the whole command: give no other")
    return args


def _prepare_command(args):
    """The terminal's command, or the approved --plan-json -> (Prepared, the LLM model that read it, None), or
    (None, "", exit code): EXIT_OK when the user gave up or nothing can be driven (said why), EXIT_ERROR on a
    setup problem or a file that is no plan."""
    try:
        if args.plan_json:
            prepared = load_plan_json(args.plan_json, args.speed, args.yaw_rate)
            return prepared._replace(wants_map=prepared.wants_map or args.map), "", None
        parser = CommandParser(args.provider)
        text = " ".join(args.command).strip()
        if not text and not args.map:          # --map alone needs no words: it is the scan
            text = _ask("Lệnh: ")
            if not text:
                return None, "", EXIT_OK
        prepared = prepare(parser, text, args.speed, args.yaw_rate, args.map)
    except (ParserSetupError, PlanError) as exc:
        print(f"Lỗi: {exc}")
        return None, "", EXIT_ERROR
    if prepared is None:
        return None, "", EXIT_OK
    return prepared, parser.model if prepared.result.provider == parser.provider else "", None


def main():
    # Ctrl-C - or realroom's Stop, a SIGINT to this process group - is a KeyboardInterrupt even when this was
    # started with SIGINT ignored (a background job): ignored, it would pass on to robot_link and Stop do nothing.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    args = _parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    problem = _preflight(args)
    if problem:
        print(f"Lỗi: {problem}")
        return EXIT_ERROR
    prepared, model, exit_code = _prepare_command(args)
    if prepared is None:
        return exit_code
    problem = _map_problem(args, prepared)
    if problem:
        print(f"Lỗi: {problem}")
        return EXIT_ERROR
    _print_stages(prepared, model)
    try:
        lock = None if args.dry_run else _lock_robot()
    except OSError as exc:
        print(f"Lỗi: không mở được khoá robot {cfg.ROBOT_LOCK_FILE}: {exc}")
        return EXIT_ERROR
    if not args.dry_run and lock is None:
        print("Lỗi: robot đang chạy một lệnh khác (demo_drive ở terminal khác, hoặc trang realroom).")
        return EXIT_ERROR
    # Checked holding the lock: no other run can move the robot between this check and the drive.
    moved = prepared.from_pose and _moved_since(prepared.from_pose)
    if moved:
        print(f"Lỗi: đường đi lập từ vị trí khác - {moved}: lập lại kế hoạch.")
        return EXIT_ERROR
    outcome, runlog = _execute(args, prepared)
    mapped = _map_after(args, runlog, outcome) if prepared.wants_map else None
    if outcome not in (RESULT_COMPLETE, RESULT_CANCELLED) or mapped is False:
        return EXIT_FAILED
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
