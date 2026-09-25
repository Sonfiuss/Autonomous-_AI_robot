"""Text command -> LLM -> wheels, with the camera filming: the whole chain in one demo, no map.

  python3 communication/demo_drive.py "đi thẳng 30 cm. sau đó rẽ trái, đi tiếp 60 cm"
  python3 communication/demo_drive.py --dry-run "..."     # everything except the UART
  python3 communication/demo_drive.py                      # asks for the command

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
"""
import argparse
import json
import logging
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime

import drive_config as cfg
from cmd_parser import LLM_PROVIDERS, PROVIDER_AUTO, PROVIDER_RULE, CommandParser, ParserSetupError
from motion_plan import STOP, PlanError, build_plan

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
STATUS_WAITING = "waiting to start"
STATUS_DONE = "done: {outcome}"
TIME_DIGITS = 3                                         # wall-clock seconds in run.json (ms)
EXIT_OK = 0
EXIT_FAILED = 1                                         # the run started and did not complete
EXIT_ERROR = 2                                          # nothing ran: setup or input problem
PROGRESS_RE = re.compile(r"primitive (\d+)/(\d+)")     # MissionRunner's per-primitive line
COMPLETE_MARK = "plan complete"
FAILED_MARK = "plan failed"
LINE_BUFFERED = ("stdbuf", "-oL")                       # robot_link's stdout is a pipe: flush per line
POLL_S = 0.2
QUIT_WORDS = ("q", "quit", "exit")
RULE = "-" * 78

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
def _leg_label(plan, index):
    leg = plan.legs[index]
    what = "stop" if leg.kind == STOP else plan.steps[leg.step_index].describe()
    return f"leg {index + 1}/{len(plan.legs)}  {leg.wire}  ({what})"


def _print_stages(result, plan):
    print(RULE)
    print(f"[1] Parser ({result.provider}):")
    print("    " + json.dumps(result.raw, ensure_ascii=False))
    print("[2] Các bước sau validator:")
    for i, step in enumerate(plan.steps, 1):
        print(f"    {i}. {step.describe()}")
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
    print(RULE)


# ------------------------------------------------------------------ steps of a run
def _ask(prompt):
    """input() that treats a closed stdin as 'no answer'."""
    try:
        return input(prompt).strip()
    except EOFError:
        return ""


def _parse_until_steps(parser, text):
    """Asks the parser's questions back to the user until there are steps.
    Returns (ParseResult, every message joined), or (None, ...) when the user gave up."""
    said = [text]
    while True:
        result = parser.parse(text)
        if result.steps:
            return result, " / ".join(said)
        print(f"? {result.question}")
        text = _ask("> ")
        if not text or text.lower() in QUIT_WORDS:
            return None, " / ".join(said)
        said.append(text)


def _drive_dry(plan, runlog):
    """Plays the plan's timing without a robot, so the video and run.json look like a real run."""
    for index, leg in enumerate(plan.legs):
        runlog.status(_leg_label(plan, index))
        print(f"  [dry-run] {_leg_label(plan, index)}")
        time.sleep(leg.duration_s)
    return RESULT_COMPLETE, EXIT_OK


def _drive_robot(plan_path, plan, args, runlog):
    """robot_link --run-plan, its output teed to the console and robot_link.log."""
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
                    runlog.status(_leg_label(plan, int(match.group(1)) - 1))
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


def _preflight(args):
    """Refuses to start a real run that cannot work, before the camera is even opened."""
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


def _execute(args, text, result, plan):
    """Films and drives one plan. Returns the result word."""
    run_dir = os.path.join(cfg.OUTPUT_DIR, datetime.now().strftime(RUN_ID_FORMAT))
    os.makedirs(run_dir, exist_ok=True)
    runlog = RunLog(run_dir)
    runlog.data.update({"command": text, "provider": result.provider, "parser_raw": result.raw,
                        "mode": MODE_DRY_RUN if args.dry_run else MODE_ROBOT, "port": args.port,
                        "plan": plan.to_dict()})
    plan_path = runlog.path(PLAN_FILE)
    with open(plan_path, "w", encoding="utf-8") as f:
        f.write(plan.plan_file_text(text))
    runlog.status(STATUS_WAITING)

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
            else:
                outcome, exit_code = _drive_robot(plan_path, plan, args, runlog)
            runlog.status(STATUS_DONE.format(outcome=outcome))
            time.sleep(cfg.VIDEO_TAIL_S)
    except KeyboardInterrupt:
        outcome = RESULT_ABORTED if driving else RESULT_CANCELLED
        runlog.status(STATUS_DONE.format(outcome=outcome))
        print("\nĐã dừng.")
    finally:
        runlog.data["result"] = outcome
        runlog.data["robot_link_exit"] = exit_code
        runlog.data["video"] = None
        if recorder is not None:
            runlog.data["recorder_exit"] = recorder.stop()
            runlog.data["video"] = recorder.video_path()
        runlog.save()
    print(f"Kết quả: {outcome}.  Thư mục: {run_dir}")
    if runlog.data["video"]:
        print(f"Video: {runlog.data['video']}")
    return outcome


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
    parser.add_argument("--no-vision", action="store_true", help="do not record")
    parser.add_argument("--no-depth", action="store_true", help="record without the Astra depth")
    parser.add_argument("--no-floor", action="store_true", help="record without the Depth Anything floor")
    parser.add_argument("--color-index", type=int, help="OpenCV index of the Astra RGB camera")
    parser.add_argument("-v", "--verbose", action="store_true", help="debug logging")
    args = parser.parse_args()
    if not 0.0 < args.speed <= cfg.MAX_CRUISE_SPEED_M_S:
        parser.error(f"--speed must be in (0, {cfg.MAX_CRUISE_SPEED_M_S}]")
    if not 0.0 < args.yaw_rate <= cfg.MAX_YAW_RATE_RAD_S:
        parser.error(f"--yaw-rate must be in (0, {cfg.MAX_YAW_RATE_RAD_S}]")
    return args


def main():
    args = _parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    problem = _preflight(args)
    if problem:
        print(f"Lỗi: {problem}")
        return EXIT_ERROR
    try:
        parser = CommandParser(args.provider)
    except ParserSetupError as exc:
        print(f"Lỗi: {exc}")
        return EXIT_ERROR
    text = " ".join(args.command).strip() or _ask("Lệnh: ")
    if not text:
        return EXIT_OK
    result, text = _parse_until_steps(parser, text)
    if result is None:
        return EXIT_OK
    try:
        plan = build_plan(result.steps, args.speed, args.yaw_rate)
    except PlanError as exc:
        print(f"Lỗi: {exc}")
        return EXIT_ERROR
    _print_stages(result, plan)
    outcome = _execute(args, text, result, plan)
    return EXIT_OK if outcome in (RESULT_COMPLETE, RESULT_CANCELLED) else EXIT_FAILED


if __name__ == "__main__":
    sys.exit(main())
