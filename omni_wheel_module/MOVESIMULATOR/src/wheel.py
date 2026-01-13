"""Wheel model handling speed updates."""

from __future__ import annotations

from dataclasses import dataclass, field

from config import settings
from utils.math_utils import clamp


@dataclass
class Wheel:
    angle: float
    max_speed: float
    speed: float = 0.0
    target_speed: float = 0.0

    def update(self, dt: float, acceleration: bool, reverse: bool) -> None:
        if acceleration and not reverse:
            self.target_speed = self.max_speed
        elif reverse and not acceleration:
            self.target_speed = -self.max_speed
        elif reverse and acceleration:
            self.target_speed = 0.0
        else:
            self.target_speed = 0.0

        if self.speed < self.target_speed:
            self.speed += settings.ACCELERATION * dt
        elif self.speed > self.target_speed:
            self.speed -= settings.DECELERATION * dt

        self.speed = clamp(self.speed, -self.max_speed, self.max_speed)

    def get_velocity_vector(self) -> tuple[float, float]:
        from math import sin, cos

        linear_speed = self.speed * settings.WHEEL_RADIUS
        return linear_speed * sin(self.angle), linear_speed * cos(self.angle)

    def reset(self) -> None:
        self.speed = 0.0
        self.target_speed = 0.0
