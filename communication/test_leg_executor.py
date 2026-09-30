"""Offline tests for leg_executor and slip_model (task 2026-09-27_explore-map Pha 1' C, D): a simulated
robot stands in for robot_link and the camera. Every leg it is sent moves it by the command times the
floor's true k - times CUT_SHARE, acked that early, when the firmware cuts it short as it did the
2026-09-28 backward legs.

Run:  python3 communication/test_leg_executor.py
Needs libmc.so (plan_leg asks MC for each leg's duration); no camera, robot or port.
"""
import json
import math
import os
import tempfile

import leg_executor
import slip_model
from motion_plan import FORWARD, ROTATE, PlanError

TRUE_K_FORWARD = 0.93
TRUE_K_TURN = 0.90
CUT_SHARE = 0.6                      # a cut-short leg: this share of the command, acked this early
READY_WAIT_S = 2.0
LOST_CAMERA_AGE_S = 3.0              # the newest frame of a lost camera: this long before the span's end
CALL_SPACING_S = 1000.0              # robot_link call N starts at N x this: calls never overlap in time
MEASURED_PAIRS, WEAK_PAIRS = 1.0, 0.3
FAKE_FRAMES, FAKE_INLIERS = 100, 250
PREDICT_REL_TOL = 0.01               # a predicted placement against command x k (x the cut share)
RECORDING = {"intrinsics": {"fx": 570.0, "fy": 570.0, "cx": 319.5, "cy": 239.5},
             "mount": {"height_m": 0.24, "pitch_deg": -0.92, "forward_m": 0.185, "left_m": 0.062}}


class FakeRunLog:
    """demo_drive.RunLog's surface: run_dir, data, event, status, path."""

    def __init__(self, run_dir):
        self.run_dir, self.data, self.events, self.statuses = run_dir, {}, [], []

    def path(self, name):
        return os.path.join(self.run_dir, name)

    def event(self, name, **detail):
        self.events.append(dict(detail, event=name))

    def status(self, text):
        self.statuses.append(text)


class FakeRobot:
    """robot_link + camera. cut: leg numbers the firmware cuts short; stuck_after: from this leg on
    the robot moves STUCK_SHARE of its command; camera: "ok" | "weak" | "lost"."""
    STUCK_SHARE = 0.1

    def __init__(self, cut=(), stuck_after=None, camera="ok"):
        self.cut, self.stuck_after, self.camera = set(cut), stuck_after, camera
        self.sent = []                       # (number, kind, commanded value)
        self.last = None                     # (kind, real motion) of the leg just driven
        self.turn_scales = []                # the turn_scale every measurement was asked for

    def call(self, number, leg, role):
        share = TRUE_K_TURN if leg.kind == ROTATE else TRUE_K_FORWARD
        fw_share = CUT_SHARE if number in self.cut else 1.0
        if self.stuck_after is not None and number >= self.stuck_after:
            share = self.STUCK_SHARE
        self.sent.append((number, leg.kind, leg.value))
        self.last = (leg.kind, leg.value * share * fw_share)
        t_start = CALL_SPACING_S * number
        return "complete", t_start, t_start + READY_WAIT_S + leg.duration_s * fw_share

    def measure(self, run_dir, t0, t1, intrinsics, mount, scale=1.0, wait_s=0.0, turn_scale=1.0):
        """The real motion of the leg just driven: the camera reads it right (turn_scale is only recorded)."""
        self.turn_scales.append(turn_scale)
        kind, real = self.last
        motion = (0.0, 0.0, real) if kind == ROTATE else (real, 0.0, 0.0)
        weak = self.camera != "ok"
        t_last = t1 - LOST_CAMERA_AGE_S if self.camera == "lost" else t1
        return leg_executor.leg_odometry.LegMotion(*motion, WEAK_PAIRS if weak else MEASURED_PAIRS, weak, FAKE_FRAMES,
                                                   0 if weak else FAKE_INLIERS, t0, t_last)


def _executor(tmp, robot, slip=None):
    run_dir = os.path.join(tmp, "run")
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, leg_executor.RECORDING_FILE), "w", encoding="utf-8") as f:
        json.dump(RECORDING, f)
    slip = slip or slip_model.SlipModel(os.path.join(tmp, "slip.json"))
    executor = leg_executor.LegExecutor("/dev/null", FakeRunLog(run_dir), 0.15, 0.8, slip, say=lambda text: None)
    executor.ready_wait = READY_WAIT_S
    executor._call = robot.call
    leg_executor.leg_odometry.measure_leg = robot.measure
    leg_executor.SETTLE_S = 0.0
    return executor


def test_forward_lands_within_tolerance():
    with tempfile.TemporaryDirectory() as tmp:
        robot = FakeRobot()
        result = _executor(tmp, robot).run(FORWARD, 0.40)
        assert result.outcome == leg_executor.OK, result
        assert abs(result.measured - 0.40) <= leg_executor.TOL_M, result.measured
        assert len(robot.sent) == 1, robot.sent           # default k 0.93 = the floor's: no top-up needed


def test_cut_short_backward_converges_and_is_not_learned():
    """40 cm backward, the firmware cutting every leg to 60 %: two top-ups land it within 3 cm (the
    plan's 40 -> 24 -> 34 -> 38), and none of the three samples teaches the model."""
    with tempfile.TemporaryDirectory() as tmp:
        robot = FakeRobot(cut=(1, 2, 3))
        executor = _executor(tmp, robot)
        result = executor.run(FORWARD, -0.40)
        assert result.outcome == leg_executor.OK, (result.outcome, result.measured)
        assert len(robot.sent) == 1 + leg_executor.MAX_COMPENSATIONS, robot.sent
        assert all(r.cut_short for r in result.runs), [r.firmware_s for r in result.runs]
        samples = executor.slip.samples[slip_model.BACKWARD]
        assert [s["why"] for s in samples] == [slip_model.WHY_CUT] * 3, samples
        assert executor.slip.k(slip_model.BACKWARD) == slip_model.DEFAULT_K[slip_model.BACKWARD]
        assert [leg["role"] for leg in executor.executed] == [leg_executor.ROLE_MAIN] + \
            [leg_executor.ROLE_COMPENSATION] * 2, executor.executed


def test_turn_is_topped_up():
    with tempfile.TemporaryDirectory() as tmp:
        robot = FakeRobot()
        result = _executor(tmp, robot).run(ROTATE, math.radians(90.0))
        assert result.outcome == leg_executor.OK, result
        assert abs(result.measured - math.radians(90.0)) <= leg_executor.TOL_RAD
        assert len(robot.sent) == 2, robot.sent           # 90 -> 81 deg, then the 9 deg top-up


def test_learned_k_feeds_the_next_command():
    """A model that learned k 0.80 on another floor commands 0.5 m as 0.625 m; the 0.93 floor then
    overshoots and the top-up backs up. Every leg lands in the file under its own kind."""
    with tempfile.TemporaryDirectory() as tmp:
        slip = slip_model.SlipModel(os.path.join(tmp, "slip.json"))
        for _ in range(slip_model.MIN_SAMPLES):
            slip.add(slip_model.FORWARD, 0.5, 0.40)
        robot = FakeRobot()
        result = _executor(tmp, robot, slip).run(FORWARD, 0.5)
        assert abs(robot.sent[0][2] - 0.5 / 0.8) < 1e-9, robot.sent
        assert result.outcome == leg_executor.OK and robot.sent[1][2] < 0.0, (result.outcome, robot.sent)
        reloaded = slip_model.SlipModel(os.path.join(tmp, "slip.json"))
        assert len(reloaded.samples[slip_model.FORWARD]) == slip_model.MIN_SAMPLES + 1
        assert len(reloaded.samples[slip_model.BACKWARD]) == len(robot.sent) - 1


def test_stuck_stops_topping_up():
    with tempfile.TemporaryDirectory() as tmp:
        robot = FakeRobot(stuck_after=1)
        executor = _executor(tmp, robot)
        result = executor.run(FORWARD, 0.40)
        assert result.outcome == leg_executor.STUCK and len(robot.sent) == 1, (result.outcome, robot.sent)
        assert executor.slip.samples[slip_model.FORWARD][-1]["why"] == slip_model.WHY_STUCK


def test_weak_and_lost_camera():
    """Weak odometry and a lost camera stop the top-ups; the leg is then placed where command x k puts it
    (plan step B), times the share of its duration the firmware ran when it cut the leg short."""
    for camera, cut, outcome, share in (("weak", (), leg_executor.WEAK, 1.0),
                                        ("weak", (1,), leg_executor.WEAK, CUT_SHARE),
                                        ("lost", (), leg_executor.CAMERA_LOST, 1.0)):
        with tempfile.TemporaryDirectory() as tmp:
            robot = FakeRobot(cut=cut, camera=camera)
            executor = _executor(tmp, robot)
            result = executor.run(FORWARD, 0.40)
            assert result.outcome == outcome and len(robot.sent) == 1, (camera, result.outcome, robot.sent)
            predicted = robot.sent[0][2] * slip_model.DEFAULT_K[slip_model.FORWARD] * share
            assert abs(result.measured - predicted) < PREDICT_REL_TOL * abs(predicted), \
                (camera, cut, result.measured, predicted)
            assert executor.requests[-1]["predicted"] and result.runs[0].predicted
    assert leg_executor.CAMERA_LOST in leg_executor.STOPPING and leg_executor.WEAK not in leg_executor.STOPPING


def test_unplannable_top_up_keeps_the_pose():
    """MC refusing a top-up leg fails the request without losing the main leg the robot already drove."""
    with tempfile.TemporaryDirectory() as tmp:
        robot = FakeRobot(cut=(1,))
        executor = _executor(tmp, robot)
        real_plan_leg = leg_executor.plan_leg

        def refuse_top_ups(step_index, kind, value, cruise_speed, yaw_rate):
            if robot.sent:
                raise PlanError("refused for the test")
            return real_plan_leg(step_index, kind, value, cruise_speed, yaw_rate)

        leg_executor.plan_leg = refuse_top_ups
        try:
            result = executor.run(FORWARD, 0.40)
        finally:
            leg_executor.plan_leg = real_plan_leg
        assert result.outcome == leg_executor.FAILED and len(robot.sent) == 1, (result.outcome, robot.sent)
        assert abs(executor.pose[0] - robot.last[1]) < 1e-9 and executor.requests[-1]["legs"] == [1], executor.pose


def test_pose_composes_legs():
    """Turn left 90 then forward 0.40: the measured pose is 0.40 along +y."""
    with tempfile.TemporaryDirectory() as tmp:
        executor = _executor(tmp, FakeRobot())
        executor.run(ROTATE, math.radians(90.0))
        executor.run(FORWARD, 0.40)
        x, y, theta = executor.pose
        assert abs(x) < 0.03 and abs(y - 0.40) <= leg_executor.TOL_M and abs(theta - math.pi / 2) <= leg_executor.TOL_RAD, \
            executor.pose


def test_turn_scale_reaches_the_odometry():
    """The executor measures every leg at the vo_turn_scale of its recording's mount, and logs it."""
    with tempfile.TemporaryDirectory() as tmp:
        robot = FakeRobot()
        executor = _executor(tmp, robot)
        want = leg_executor.vo_turn_scale(executor.mount)
        executor.run(ROTATE, math.radians(90.0))
        assert robot.turn_scales and all(s == want for s in robot.turn_scales), (robot.turn_scales, want)
        assert executor.runlog.data["vo_turn_scale"] == want, executor.runlog.data


if __name__ == "__main__":
    try:
        test_forward_lands_within_tolerance()
        test_cut_short_backward_converges_and_is_not_learned()
        test_turn_is_topped_up()
        test_learned_k_feeds_the_next_command()
        test_stuck_stops_topping_up()
        test_weak_and_lost_camera()
        test_unplannable_top_up_keeps_the_pose()
        test_pose_composes_legs()
        test_turn_scale_reaches_the_odometry()
    except PlanError as exc:
        print(f"SKIPPED (needs libmc): {exc}")
        raise SystemExit(0)
    print("ALL OK")
