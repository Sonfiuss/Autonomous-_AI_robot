"""Drives one leg at a time and checks every leg against the camera (task 2026-09-27_explore-map Pha 1' C).
The pose it reports is the measured one: the user trusts the images over the wheels (2026-09-28).

For each requested leg (FORWARD m / ROTATE rad):
  1. command = what is left of the target / k (slip_model) of that kind of motion
  2. `robot_link --stop`, then `robot_link --run-plan` with that one leg (its sequencer still checks the
     firmware's faults, reboots and ack timeout; the cost is its READY_WAIT_MS before the leg)
  3. after the leg's K and SETTLE_S standing still: vision/leg_odometry.measure_leg over the call
  4. still off by more than TOL_M / TOL_RAD: back to 1 for the rest, at most MAX_COMPENSATIONS times
Every measured leg goes to the slip model, which learns only the trustworthy ones.

It stops compensating on weak odometry (nothing to compensate against: the leg is then placed where the
command x k predicts, flagged - plan step B), a stuck leg (moved under
slip_model.STUCK_RATIO of its command: something in the way) and a lost camera (raw frames stopped: the
USB hub reset of 2026-09-28 - the robot then stops altogether). A leg the firmware acked well before its
planned duration (vision/drive_timeline.cut_short: the backward legs of 2026-09-28) is compensated like any
short leg but not learned.

run.json gets "executed_legs" (every leg actually sent, in order; vision/drive_timeline reads it instead of
plan.legs), "requests" (per requested leg: target, measured, error, outcome) and the events leg_acked /
leg_measured. Status lines are "leg N/N ..." per executed leg, as drive_timeline expects.
"""
import collections
import json
import logging
import math
import os
import subprocess
import time

import drive_config as cfg
import slip_model
from motion_plan import ROTATE, STOP, PlanError, plan_leg
from robot_link_cli import COMPLETE_MARK, FAILED_MARK, LINE_BUFFERED, PROGRESS_RE, flush_link

cfg.add_vision_to_path()
cfg.add_realroom_to_path()
import drive_timeline  # noqa: E402  (vision module, path set just above)
import leg_odometry  # noqa: E402
from floor_geometry import CameraMount  # noqa: E402
from frame_motion import vo_scale, vo_turn_scale  # noqa: E402
from map_store import compose  # noqa: E402  (realroom: motion b in the frame a ends in, after a)
from object_distance import Intrinsics  # noqa: E402

TOL_M = 0.03                         # the plan's acceptance: 30 +- 3 cm on the tape
TOL_RAD = math.radians(2.0)
MAX_COMPENSATIONS = 2                # a leg cut to 60 % still lands within 3 cm: 40 -> 24 -> 34 -> 38 cm
SETTLE_S = 0.5                       # standing still after the K before the span closes
MEASURE_WAIT_S = 3.0                 # for the raw writer to put the span's last frame on disk
CAMERA_LOST_S = 1.0                  # the newest raw frame this far before the span's end: the camera is gone
LEG_PLAN_PATTERN = "leg_{n:02d}.txt"
ROBOT_LINK_LOG = "robot_link.log"
RECORDING_FILE = "recording.json"
ACKED_PRIMITIVE = 2                  # "primitive 2/2" (the closing STOP) is printed when leg 1's K came in
ROLE_MAIN, ROLE_COMPENSATION = "main", "compensation"
CALL_COMPLETE, CALL_FAILED, CALL_ABORTED = "complete", "failed", "aborted"   # how one robot_link call ended
# Outcomes of one requested leg.
OK, OFF, WEAK, STUCK, CAMERA_LOST, FAILED, ABORTED = ("ok", "off target", "weak odometry", "stuck", "camera lost",
                                                     "robot_link failed", "aborted")
UNMEASURED_ENDS = (FAILED, ABORTED)   # a request that ended so may have moved the robot unmeasured (demo_drive)
STOPPING = (CAMERA_LOST, FAILED, ABORTED)                # the run cannot go on after these
M_DIGITS, RAD_DIGITS, T_DIGITS, K_DIGITS, SHARE_DIGITS = 4, 5, 3, 4, 3

# One robot_link call. outcome: CALL_COMPLETE / CALL_FAILED / CALL_ABORTED; t_start: when it was
# launched; t_ack: when its K came in (None: never); firmware_s: wire line to K (None: unknown); motion: the
# LegMotion measured (None: not measured); along: its motion along the leg; placed: the (dx, dy, dtheta) the
# pose moves by - measured, or predicted when the odometry was weak (None: unknown, the leg failed).
LegRun = collections.namedtuple("LegRun", "number role leg k outcome t_start t_ack firmware_s cut_short motion along "
                                          "placed predicted")
# One requested leg: kind FORWARD / ROTATE, target (m / rad), runs, measured along the target (m / rad),
# motion (dx, dy, dtheta) in the body frame it started from, outcome (OK ... ABORTED).
LegResult = collections.namedtuple("LegResult", "kind target runs measured motion outcome")

logger = logging.getLogger(__name__)


def load_camera(run_dir):
    """(Intrinsics, CameraMount) the recorder of run_dir writes into its recording.json at start."""
    with open(os.path.join(run_dir, RECORDING_FILE), encoding="utf-8") as f:
        recording = json.load(f)
    return Intrinsics(**recording["intrinsics"]), CameraMount(**recording["mount"])


class LegExecutor:
    """Runs requested legs through robot_link one at a time, measured by the recorder of runlog.run_dir.
    pose: the robot centre (x, y, theta) in the frame it had when the executor was made, as measured."""

    def __init__(self, port, runlog, cruise_speed, yaw_rate, slip=None, say=print):
        self.port, self.runlog = port, runlog
        self.cruise_speed, self.yaw_rate = cruise_speed, yaw_rate
        self.slip = slip if slip is not None else slip_model.SlipModel()
        self.say = say
        self.intrinsics, self.mount = load_camera(runlog.run_dir)
        self.scale = vo_scale(self.mount)
        self.turn_scale = vo_turn_scale(self.mount)   # the odometry reads turns ~10 % long (2026-09-29)
        self.ready_wait = drive_timeline.ready_wait_s()
        self.pose = (0.0, 0.0, 0.0)
        self.executed = runlog.data.setdefault("executed_legs", [])
        self.requests = runlog.data.setdefault("requests", [])
        runlog.data["vo_scale"] = self.scale
        runlog.data["vo_turn_scale"] = self.turn_scale

    # ------------------------------------------------------------------ one requested leg
    def run(self, kind, target, step_index=-1):
        """Drives `target` (m for FORWARD, rad for ROTATE) with measurement and compensation. Returns a
        LegResult; self.pose moves by what was measured."""
        runs, motion, outcome = [], (0.0, 0.0, 0.0), OFF
        for attempt in range(1 + MAX_COMPENSATIONS):
            rest = target - self._along(kind, motion)
            try:
                leg_run = self._drive(kind, rest, ROLE_COMPENSATION if attempt else ROLE_MAIN, step_index)
            except PlanError as exc:             # MC refused the leg: nothing was sent, the legs before it count
                logger.error("leg for %s %+.4f not planned: %s", kind, rest, exc)
                outcome = FAILED
                break
            runs.append(leg_run)
            if leg_run.placed is not None:
                motion = compose(motion, leg_run.placed)
            stop = self._verdict(kind, leg_run)
            if stop is not None:
                outcome = stop
                break
            outcome = OK if abs(target - self._along(kind, motion)) <= self._tolerance(kind) else OFF
            if outcome == OK:
                break
        measured = self._along(kind, motion)
        self.pose = compose(self.pose, motion)
        result = LegResult(kind, target, runs, measured, motion, outcome)
        self.requests.append({"step_index": step_index, "kind": kind, "target": round(target, RAD_DIGITS),
                              "measured": round(measured, RAD_DIGITS), "error": round(target - measured, RAD_DIGITS),
                              "motion": [round(v, RAD_DIGITS) for v in motion], "legs": [r.number for r in runs],
                              "predicted": any(r.predicted for r in runs), "outcome": outcome})
        self.say(self.describe(result))
        return result

    @staticmethod
    def _verdict(kind, leg_run):
        """The outcome that ends a requested leg after this run, or None to carry on. Stuck is judged on
        legs long enough to learn from: on a 3 cm compensation the odometry's noise is a large share."""
        if leg_run.outcome == CALL_ABORTED:
            return ABORTED
        if leg_run.motion is None or leg_run.outcome != CALL_COMPLETE:
            return FAILED
        m = leg_run.motion
        if m.weak and not m.t_last >= leg_run.t_ack + SETTLE_S - CAMERA_LOST_S:   # nan: no frame at all
            return CAMERA_LOST
        if m.weak:
            return WEAK
        long_enough = abs(leg_run.leg.value) >= slip_model.LEARN_MIN[slip_model.kind_of(kind, leg_run.leg.value)]
        if long_enough and not leg_run.along / leg_run.leg.value >= slip_model.STUCK_RATIO:
            return STUCK
        return None

    @staticmethod
    def _along(kind, motion):
        return motion[2] if kind == ROTATE else motion[0]

    @staticmethod
    def _predicted(kind, leg, k, firmware_s, cut):
        """(dx, dy, dtheta) of a leg the odometry could not measure: the command x k along it, times the
        share of its duration the firmware ran when it cut the leg short (2026-09-28: a leg run 1.6x fast
        covered 1 / 1.6 of the command)."""
        share = min(firmware_s / leg.duration_s, 1.0) if cut else 1.0
        amount = leg.value * k * share
        return (0.0, 0.0, amount) if kind == ROTATE else (amount, 0.0, 0.0)

    @staticmethod
    def _tolerance(kind):
        return TOL_RAD if kind == ROTATE else TOL_M

    # ------------------------------------------------------------------ one robot_link call
    def _drive(self, kind, rest, role, step_index):
        """Sends `rest` / k as one leg, waits for it, measures it, teaches the slip model. LegRun."""
        slip_kind = slip_model.kind_of(kind, rest)
        k = self.slip.k(slip_kind)
        leg = plan_leg(step_index, kind, rest / k, self.cruise_speed, self.yaw_rate)
        number = len(self.executed) + 1
        self.executed.append({"primitive": leg.plan_line(), "wire": leg.wire,
                              "duration_s": round(leg.duration_s, T_DIGITS), "waits_ready": True, "role": role,
                              "step_index": step_index, "k": round(k, K_DIGITS)})
        outcome, t_start, t_ack = self._call(number, leg, role)
        firmware_s = None if t_ack is None or self.ready_wait is None else t_ack - t_start - self.ready_wait
        cut = firmware_s is not None and firmware_s < drive_timeline.FIRMWARE_SHORT_SHARE * leg.duration_s
        if cut:
            logger.warning("leg %d: acked %.2f s after the wire line, planned %.2f s - cut short by the firmware",
                           number, firmware_s, leg.duration_s)
        motion, along, placed, predicted = None, 0.0, None, False
        if outcome == CALL_COMPLETE and t_ack is not None:
            self.runlog.status(f"measuring leg {number}")
            time.sleep(SETTLE_S)
            motion = leg_odometry.measure_leg(self.runlog.run_dir, t_start, t_ack + SETTLE_S, self.intrinsics,
                                              self.mount, self.scale, MEASURE_WAIT_S, turn_scale=self.turn_scale)
            placed = (motion.dx, motion.dy, motion.dtheta)
            along = self._along(kind, placed)
            if motion.weak:
                placed, predicted = self._predicted(kind, leg, k, firmware_s, cut), True
            speed = self.yaw_rate if kind == ROTATE else self.cruise_speed
            self.slip.add(slip_kind, leg.value, along, cut_short=cut, weak=motion.weak,
                          run=os.path.basename(self.runlog.run_dir), leg=number, pairs=round(motion.pairs, SHARE_DIGITS),
                          speed=speed)
            self.runlog.event("leg_measured", leg=number, dx=round(motion.dx, M_DIGITS), dy=round(motion.dy, M_DIGITS),
                              dtheta=round(motion.dtheta, RAD_DIGITS), pairs=round(motion.pairs, SHARE_DIGITS),
                              weak=motion.weak,
                              frames=motion.frames)
        self.executed[-1].update({"outcome": outcome, "cut_short": cut, "predicted": predicted,
                                  "firmware_s": None if firmware_s is None else round(firmware_s, T_DIGITS),
                                  "measured": None if motion is None else round(along, RAD_DIGITS)})
        return LegRun(number, role, leg, k, outcome, t_start, t_ack, firmware_s, cut, motion, along, placed, predicted)

    def _call(self, number, leg, role):
        """robot_link --run-plan on a one-leg plan (the leg + STOP). (outcome, t_start, t_ack)."""
        path = self.runlog.path(LEG_PLAN_PATTERN.format(n=number))
        with open(path, "w", encoding="utf-8") as f:
            f.write(f"# leg_executor leg {number} ({role})\n{leg.plan_line()}\n{STOP}\n")
        flush_link(self.port, self.runlog)
        cmd = list(LINE_BUFFERED) + [cfg.ROBOT_LINK_BIN, "--port", self.port, "--run-plan", path,
                                     "--speed", str(self.cruise_speed), "--yaw-rate", str(self.yaw_rate)]
        outcome, t_ack = CALL_FAILED, None
        t_start = time.time()
        self.runlog.event("robot_link_started", cmd=cmd, leg=number)
        with open(self.runlog.path(ROBOT_LINK_LOG), "a", encoding="utf-8") as log:
            try:
                proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        universal_newlines=True, bufsize=1)
            except OSError as exc:                 # stdbuf or robot_link missing: nothing was sent
                self.runlog.event("robot_link_failure", line=str(exc), leg=number)
                return outcome, t_start, None
            try:
                for line in proc.stdout:
                    log.write(line)
                    self.say("  [robot_link] " + line.rstrip())
                    match = PROGRESS_RE.search(line)
                    if COMPLETE_MARK in line:
                        outcome = CALL_COMPLETE
                    elif FAILED_MARK in line:
                        self.runlog.event("robot_link_failure", line=line.strip(), leg=number)
                    elif match and int(match.group(1)) == ACKED_PRIMITIVE:
                        t_ack = time.time()
                        self.runlog.event(drive_timeline.ACKED_EVENT, leg=number)
                    elif match:
                        self.runlog.status(f"leg {number}/{number}  {leg.wire}  ({role})")
                proc.wait()
            except KeyboardInterrupt:
                # The same Ctrl-C reached robot_link: it aborts the plan and sends S. Let it finish that.
                outcome = CALL_ABORTED
                try:
                    remaining, _ = proc.communicate(timeout=cfg.ROBOT_LINK_GRACE_S)
                    log.write(remaining or "")
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        if proc.returncode != 0 and outcome == CALL_COMPLETE:
            outcome = CALL_FAILED
        self.runlog.event("robot_link_exited", exit_code=proc.returncode, leg=number)
        return outcome, t_start, t_ack

    # ------------------------------------------------------------------ reporting
    def describe(self, result):
        """One line per requested leg and one per robot_link call, in the user's language."""
        unit, conv = ("°", math.degrees) if result.kind == ROTATE else ("cm", lambda v: v * cfg.CM_PER_M)
        lines = [f"  {result.kind} đích {conv(result.target):+.1f} {unit} -> đo {conv(result.measured):+.1f} {unit} "
                 f"(lệch {conv(result.target - result.measured):+.1f}) : {result.outcome}"]
        for r in result.runs:
            fw = "?" if r.firmware_s is None else f"{r.firmware_s:.2f}"
            m = r.motion
            seen = "không đo" if m is None else f"đo {conv(r.along):+.1f} {unit}, {m.pairs:.0%} cặp"
            if r.predicted:
                seen += f" YẾU -> dùng dự đoán {conv(self._along(result.kind, r.placed)):+.1f} {unit}"
            lines.append(f"    leg {r.number} ({r.role}, k {r.k:.3f}): lệnh {conv(r.leg.value):+.1f} {unit}, {seen}; "
                         f"firmware {fw} / {r.leg.duration_s:.2f} s{' BỊ CẮT' if r.cut_short else ''}")
        return "\n".join(lines)
