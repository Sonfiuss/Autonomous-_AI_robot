"""Utility math helpers for the simulator."""

import math


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    wrapped = (angle + math.pi) % (2 * math.pi) - math.pi
    return wrapped


def deg_to_rad(deg: float) -> float:
    return math.radians(deg)


def rad_to_deg(rad: float) -> float:
    return math.degrees(rad)


def calculate_distance(p1: tuple, p2: tuple) -> float:
    dx = p1[0] - p2[0]
    dy = p1[1] - p2[1]
    return math.hypot(dx, dy)


def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, value))
