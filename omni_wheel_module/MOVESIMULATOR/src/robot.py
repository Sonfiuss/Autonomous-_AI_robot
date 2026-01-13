"""Robot model that aggregates wheels and applies kinematics."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

from config import settings
from utils.math_utils import deg_to_rad, normalize_angle
from .wheel import Wheel
from . import kinematics


@dataclass
class Robot:
    start_pos: Tuple[float, float]
    position: List[float] = field(default_factory=lambda: [0.0, 0.0])
    orientation: float = 0.0
    wheels: List[Wheel] = field(default_factory=list)

    def __post_init__(self) -> None:
        self.position = [self.start_pos[0], self.start_pos[1]]
        self.wheels = [
            Wheel(angle=deg_to_rad(angle_deg), max_speed=settings.MAX_WHEEL_SPEED)
            for angle_deg in settings.WHEEL_ANGLES
        ]

    def update(self, dt: float, wheel_commands: List[dict]) -> None:
        for wheel, command in zip(self.wheels, wheel_commands):
            wheel.update(dt, command.get("accelerate", False), command.get("reverse", False))

        wheel_speeds = [w.speed for w in self.wheels]
        wheel_angles = [w.angle for w in self.wheels]
        vx, vy, omega = kinematics.forward_kinematics(wheel_speeds, wheel_angles, settings.ROBOT_RADIUS)

        gx, gy = kinematics.rotate_vector(vx, vy, self.orientation)
        self.position[0] += gx * dt
        self.position[1] += gy * dt
        self.orientation = normalize_angle(self.orientation + omega * dt)

    def get_global_velocity(self) -> Tuple[float, float, float]:
        wheel_speeds = [w.speed for w in self.wheels]
        wheel_angles = [w.angle for w in self.wheels]
        vx, vy, omega = kinematics.forward_kinematics(wheel_speeds, wheel_angles, settings.ROBOT_RADIUS)
        gx, gy = kinematics.rotate_vector(vx, vy, self.orientation)
        return gx, gy, omega

    def get_wheel_positions(self) -> List[Tuple[float, float]]:
        from math import cos, sin

        positions = []
        for wheel in self.wheels:
            offset_angle = wheel.angle + self.orientation
            x = self.position[0] + settings.ROBOT_RADIUS * cos(offset_angle)
            y = self.position[1] + settings.ROBOT_RADIUS * sin(offset_angle)
            positions.append((x, y))
        return positions

    def reset(self) -> None:
        self.position = [self.start_pos[0], self.start_pos[1]]
        self.orientation = 0.0
        for wheel in self.wheels:
            wheel.reset()
