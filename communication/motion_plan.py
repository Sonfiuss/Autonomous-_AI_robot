"""Validated MotionSteps -> primitives -> wire legs -> per-wheel numbers, one stage at a time.

  primitives  FORWARD m / ROTATE rad / STOP: the plan-file format `robot_link --run-plan` reads
  wire legs   the F / T / S lines SEQ will put on the UART for them
  wheels      per leg and per wheel: signed pulse count, peak pulse rate, peak wheel speed, duration.
              From MC, which runs the same RM chain (kinematics, trapezoid, step/dir) as the firmware,
              so these are the numbers the ESP32 will generate, not an estimate of them.

SEQ stays the authority on the wire; the wire column is a preview built with its rules: one leg per
FORWARD/ROTATE (no MOVE is ever produced here, so no decomposition), `S` for STOP, 3 decimals, and a
ROTATE on a zero-bias plan is sent as-is. Turns are cut into chunks below 180 deg because SEQ wraps
headings into (-180, 180]: an unchunked 180 deg right turn would be driven as a left one.
"""
import logging
import math
from dataclasses import dataclass

import drive_config as cfg
from cmd_parser import ACTION_BACKWARD, ACTION_FORWARD, ACTION_TURN_AROUND, ACTION_TURN_LEFT, ACTION_TURN_RIGHT

logger = logging.getLogger(__name__)

FORWARD = "FORWARD"
ROTATE = "ROTATE"
STOP = "STOP"
WIRE_STOP = "S"
PRIMITIVE_DIGITS = 6             # plan-file precision; the wire rounds to WIRE_DECIMALS anyway
JSON_DIGITS = 3                  # seconds in run.json (ms)
LINEAR_SIGN = {ACTION_FORWARD: 1.0, ACTION_BACKWARD: -1.0}
TURN_SIGN = {ACTION_TURN_LEFT: 1.0, ACTION_TURN_RIGHT: -1.0, ACTION_TURN_AROUND: 1.0}   # CCW +; a U-turn goes left


class PlanError(RuntimeError):
    """MC is missing or refused a leg."""


@dataclass(frozen=True)
class WheelLeg:
    """What one wheel's DM556 receives during one leg."""
    steps: int            # signed pulse count; the sign is the DIR level
    peak_hz: float        # highest pulse rate of the leg
    peak_rad_s: float     # wheel speed at that moment, signed

    def to_dict(self):
        return {"steps": self.steps, "peak_hz": self.peak_hz, "peak_rad_s": self.peak_rad_s}


@dataclass(frozen=True)
class Leg:
    """One primitive, the wire line it becomes, and what the wheels do during it."""
    step_index: int       # the MotionStep it came from; -1 for the closing STOP
    kind: str             # FORWARD | ROTATE | STOP
    value: float          # metres (FORWARD, signed) or radians (ROTATE, CCW positive)
    wire: str
    duration_s: float
    wheels: tuple         # WheelLeg per wheel; empty for STOP

    def plan_line(self):
        if self.kind == STOP:
            return STOP
        return f"{self.kind} {self.value:.{PRIMITIVE_DIGITS}f}"

    def to_dict(self):
        return {"step_index": self.step_index, "primitive": self.plan_line(), "wire": self.wire,
                "duration_s": round(self.duration_s, JSON_DIGITS), "wheels": [w.to_dict() for w in self.wheels]}


@dataclass(frozen=True)
class MotionPlan:
    """Every stage of one command, from the validated steps down to the wheels."""
    steps: tuple          # the validated MotionSteps
    legs: tuple           # one per primitive, closing STOP included
    cruise_speed: float
    yaw_rate: float

    @property
    def duration_s(self):
        return sum(leg.duration_s for leg in self.legs)

    def plan_file_text(self, command):
        """The file handed to `robot_link --run-plan`. Comment lines are skipped by its reader."""
        header = [f"# demo_drive plan: {command.replace(chr(10), ' ')}",
                  f"# speed {self.cruise_speed:g} m/s, yaw rate {self.yaw_rate:g} rad/s"]
        return "\n".join(header + [leg.plan_line() for leg in self.legs]) + "\n"

    def to_dict(self):
        return {"steps": [s.to_dict() for s in self.steps], "legs": [leg.to_dict() for leg in self.legs],
                "cruise_speed": self.cruise_speed, "yaw_rate": self.yaw_rate,
                "duration_s": round(self.duration_s, JSON_DIGITS)}


def _primitives(steps):
    """(step_index, kind, value) per primitive, closing STOP included."""
    out = []
    for index, step in enumerate(steps):
        if step.action in LINEAR_SIGN:
            out.append((index, FORWARD, LINEAR_SIGN[step.action] * step.amount))
        else:
            chunks = math.ceil(step.amount / cfg.TURN_CHUNK_MAX_DEG)
            chunk_rad = TURN_SIGN[step.action] * math.radians(step.amount / chunks)
            out += [(index, ROTATE, chunk_rad)] * chunks
    out.append((-1, STOP, 0.0))
    return out


def _wire_line(kind, value, cruise_speed, yaw_rate):
    """The line LINK formats for this leg (link::formatCommand)."""
    d = cfg.WIRE_DECIMALS
    if kind == FORWARD:
        return f"F {value:.{d}f} {cruise_speed:.{d}f}"
    if kind == ROTATE:
        return f"T {math.degrees(value):.{d}f} {yaw_rate:.{d}f}"
    return WIRE_STOP


def _wheel_legs(result):
    """MC's per-tick rows -> one WheelLeg per wheel."""
    rows, tick = result["steps"], result["tick_s"]
    wheels = []
    for k in range(len(rows[0]["hz"])):
        pulses = sum(row["dir"][k] * row["hz"][k] * tick for row in rows)
        peak = max(rows, key=lambda row: row["hz"][k])
        wheels.append(WheelLeg(int(round(pulses)), peak["hz"][k], peak["w"][k]))
    return tuple(wheels)


def build_plan(steps, cruise_speed=cfg.CRUISE_SPEED_M_S, yaw_rate=cfg.YAW_RATE_RAD_S):
    """MotionSteps -> MotionPlan. Raises PlanError when MC is missing or refuses a leg."""
    cfg.add_cm_to_path()
    try:
        import mc_client                         # CM's ctypes bridge to libmc.so
        mc_client.load_library()
    except (ImportError, RuntimeError) as exc:   # McError is a RuntimeError
        raise PlanError(f"MC unavailable ({exc}); build it: bash project/tools/build_mc.sh") from exc

    legs = []
    for step_index, kind, value in _primitives(steps):
        wire = _wire_line(kind, value, cruise_speed, yaw_rate)
        if kind == STOP:
            legs.append(Leg(step_index, kind, value, wire, 0.0, ()))
            continue
        result = mc_client.run([{"type": kind, "a": value, "b": 0.0}],
                               cruise_speed=cruise_speed, yaw_rate=yaw_rate)
        if not result["ok"] or not result["steps"]:
            raise PlanError(f"MC refused {kind} {value:g}: {result.get('reason', 'no ticks')}")
        legs.append(Leg(step_index, kind, value, wire, result["duration_s"], _wheel_legs(result)))
        logger.debug("leg %s %.4f -> %s, %.2f s", kind, value, wire, result["duration_s"])
    return MotionPlan(tuple(steps), tuple(legs), cruise_speed, yaw_rate)
