"""
Drive Controller API

Python API điều khiển hệ thống di chuyển omni wheel qua serial.
"""

import math
from .serial_interface import SerialInterface


class DriveController:
    """Điều khiển hệ thống di chuyển omni wheel."""

    def __init__(self, port: str, baudrate: int = 115200):
        self.serial = SerialInterface(port, baudrate)

    def connect(self) -> bool:
        return self.serial.connect()

    def disconnect(self):
        self.serial.disconnect()

    def set_velocity(self, vx: float, vy: float, omega: float) -> bool:
        """
        Đặt vận tốc robot (omni-directional).

        Args:
            vx: Vận tốc theo trục X (mm/s)
            vy: Vận tốc theo trục Y (mm/s)
            omega: Vận tốc góc (rad/s)
        """
        cmd = f"VEL {vx:.2f} {vy:.2f} {omega:.4f}"
        response = self.serial.send_and_wait(cmd)
        return response is not None and "ACK" in response

    def move_forward(self, speed: float) -> bool:
        """Di chuyển thẳng phía trước."""
        return self.set_velocity(speed, 0, 0)

    def move_lateral(self, speed: float) -> bool:
        """Di chuyển ngang (strafe)."""
        return self.set_velocity(0, speed, 0)

    def rotate(self, omega: float) -> bool:
        """Xoay tại chỗ."""
        return self.set_velocity(0, 0, omega)

    def move_direction(self, speed: float, angle_deg: float) -> bool:
        """
        Di chuyển theo hướng bất kỳ.

        Args:
            speed: Tốc độ (mm/s)
            angle_deg: Hướng di chuyển (độ, 0 = phía trước, 90 = phải)
        """
        angle_rad = math.radians(angle_deg)
        vx = speed * math.cos(angle_rad)
        vy = speed * math.sin(angle_rad)
        return self.set_velocity(vx, vy, 0)

    def stop(self) -> bool:
        """Dừng robot."""
        return self.set_velocity(0, 0, 0)

    def emergency_stop(self):
        """Dừng khẩn cấp."""
        self.serial.send("ESTOP")
