"""Matplotlib-based renderer for the simulation."""

from __future__ import annotations

import math
from typing import Optional

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from config import settings
from src.robot import Robot
from .trajectory import TrajectoryTracker


class SimulationRenderer:
    def __init__(self, grid_size: float) -> None:
        self.grid_size = grid_size
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.robot_body: Optional[Circle] = None
        self.trajectory_plot = None
        self.velocity_arrow = None

    def setup_plot(self) -> None:
        self.ax.set_xlim(-self.grid_size, self.grid_size)
        self.ax.set_ylim(-self.grid_size, self.grid_size)
        self.ax.set_aspect("equal")
        self.ax.grid(True, linestyle="--", alpha=0.3)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("Omni Wheel Robot Simulator")

    def draw_robot(self, robot: Robot) -> None:
        if self.robot_body is None:
            self.robot_body = Circle(robot.position, settings.ROBOT_RADIUS, color="#4C72B0", alpha=0.4)
            self.ax.add_patch(self.robot_body)
        else:
            self.robot_body.center = robot.position

        heading_x = robot.position[0] + settings.ROBOT_RADIUS * math.cos(robot.orientation)
        heading_y = robot.position[1] + settings.ROBOT_RADIUS * math.sin(robot.orientation)
        self.ax.plot([robot.position[0], heading_x], [robot.position[1], heading_y], color="#1B3B6F")

        wheel_positions = robot.get_wheel_positions()
        for wx, wy in wheel_positions:
            self.ax.plot(wx, wy, "ko", markersize=5)

    def draw_velocity_vector(self, robot: Robot, scale: float = 1.0) -> None:
        gx, gy, omega = robot.get_global_velocity()
        if self.velocity_arrow:
            self.velocity_arrow.remove()
            self.velocity_arrow = None
        self.velocity_arrow = self.ax.arrow(
            robot.position[0],
            robot.position[1],
            gx * scale,
            gy * scale,
            head_width=0.05,
            length_includes_head=True,
            color="#E76F51",
        )

    def draw_trajectory(self, trajectory: TrajectoryTracker) -> None:
        xs, ys = trajectory.get_trajectory()
        if self.trajectory_plot:
            self.trajectory_plot.remove()
        self.trajectory_plot, = self.ax.plot(xs, ys, color="#2A9D8F", linewidth=1.5)

    def update_frame(self, robot: Robot, trajectory: TrajectoryTracker) -> None:
        self.ax.cla()
        self.setup_plot()
        self.draw_robot(robot)
        self.draw_velocity_vector(robot, scale=0.5)
        self.draw_trajectory(trajectory)
        self.show_legend(robot)

    def show_legend(self, robot: Robot) -> None:
        wheel_speeds = [w.speed for w in robot.wheels]
        gx, gy, omega = robot.get_global_velocity()
        text = (
            f"vx: {gx:.2f} m/s\n"
            f"vy: {gy:.2f} m/s\n"
            f"omega: {omega:.2f} rad/s\n"
            f"wheels: {[f'{s:.2f}' for s in wheel_speeds]}"
        )
        self.ax.text(
            0.02,
            0.98,
            text,
            transform=self.ax.transAxes,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.7),
        )
