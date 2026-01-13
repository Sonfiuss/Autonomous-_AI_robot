"""Forward and inverse kinematics utilities for an omni robot."""

from __future__ import annotations

import math
from typing import List, Tuple

from config import settings


WheelSpeeds = List[float]
WheelAngles = List[float]


def forward_kinematics(wheel_speeds: WheelSpeeds, wheel_angles: WheelAngles, robot_radius: float) -> Tuple[float, float, float]:
    """Compute robot velocity (vx, vy, omega) in robot frame using OpenBase kiwi model."""
    if len(wheel_speeds) != 3:
        raise ValueError("Expect exactly three wheel speeds")

    # Convert wheel angular speeds (rad/s) to ground speeds (m/s)
    r = settings.WHEEL_RADIUS
    V1, V2, V3 = (ws * r for ws in wheel_speeds)  # order: left (240°), back (0°), right (120°)

    vx = (2.0 / 3.0) * V2 - (1.0 / 3.0) * (V1 + V3)
    vy = (math.sqrt(3.0) / 3.0) * (V3 - V1)
    omega = (1.0 / (3.0 * robot_radius)) * (V1 + V2 + V3)
    return vx, vy, omega


def inverse_kinematics(vx: float, vy: float, omega: float, wheel_angles: WheelAngles, robot_radius: float) -> WheelSpeeds:
    """Compute wheel angular speeds (rad/s) from robot velocity using OpenBase kiwi model."""
    r = settings.WHEEL_RADIUS

    V1 = robot_radius * omega - 0.5 * vx - (math.sqrt(3.0) / 2.0) * vy
    V2 = robot_radius * omega + vx
    V3 = robot_radius * omega - 0.5 * vx + (math.sqrt(3.0) / 2.0) * vy

    return [V1 / r, V2 / r, V3 / r]


def wheel_omegas_to_pulse_hz(omegas: WheelSpeeds, microsteps: int, steps_per_rev: int | None = None, gear_ratio: float | None = None) -> List[tuple[float, int]]:
    """Convert wheel angular speeds (rad/s) to step pulse frequency (Hz) and direction.

    Args:
        omegas: list of wheel angular speeds (rad/s)
        microsteps: microstepping factor (e.g., 16)
        steps_per_rev: motor full steps per rev (defaults to settings.STEPS_PER_REV)
        gear_ratio: wheel rev per motor rev (defaults to settings.GEAR_RATIO)

    Returns:
        List of tuples (pulse_hz, direction), direction in { -1, 0, 1 }
    """
    spr = steps_per_rev or settings.STEPS_PER_REV
    gear = gear_ratio or settings.GEAR_RATIO
    scale = (spr * microsteps) / (2.0 * math.pi * gear)
    result: List[tuple[float, int]] = []
    for w in omegas:
        direction = 0
        if w > 0:
            direction = 1
        elif w < 0:
            direction = -1
        pulse_hz = abs(w) * scale
        result.append((pulse_hz, direction))
    return result


def pulses_to_wheel_omegas(pulses_hz: WheelSpeeds, microsteps: int, steps_per_rev: int | None = None, gear_ratio: float | None = None) -> WheelSpeeds:
    """Convert step pulse frequency (Hz) to wheel angular speed (rad/s).

    Args:
        pulses_hz: list of pulse frequencies (Hz), sign indicates direction
        microsteps: microstepping factor
        steps_per_rev: motor full steps per rev (defaults to settings.STEPS_PER_REV)
        gear_ratio: wheel rev per motor rev (defaults to settings.GEAR_RATIO)
    """
    spr = steps_per_rev or settings.STEPS_PER_REV
    gear = gear_ratio or settings.GEAR_RATIO
    scale = (2.0 * math.pi * gear) / (spr * microsteps)
    return [p * scale for p in pulses_hz]


def rotate_vector(vx: float, vy: float, angle: float) -> Tuple[float, float]:
    ca, sa = math.cos(angle), math.sin(angle)
    return vx * ca - vy * sa, vx * sa + vy * ca


def limit_speed(speed: float, max_speed: float) -> float:
    return max(-max_speed, min(max_speed, speed))
