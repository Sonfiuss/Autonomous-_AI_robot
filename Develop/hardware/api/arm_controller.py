"""
Arm Controller API

Python API điều khiển cánh tay robot qua serial.
"""

from .serial_interface import SerialInterface


class ArmController:
    """Điều khiển cánh tay robot."""

    def __init__(self, port: str, baudrate: int = 115200):
        self.serial = SerialInterface(port, baudrate)

    def connect(self) -> bool:
        return self.serial.connect()

    def disconnect(self):
        self.serial.disconnect()

    def move_joint(self, joint_id: int, angle: float) -> bool:
        """
        Di chuyển 1 khớp đến góc chỉ định.

        Args:
            joint_id: ID của khớp (0-5)
            angle: Góc mục tiêu (độ)
        """
        cmd = f"JOINT {joint_id} {angle}"
        response = self.serial.send_and_wait(cmd)
        return response is not None and "ACK" in response

    def move_to_position(self, x: float, y: float, z: float) -> bool:
        """
        Di chuyển end-effector đến tọa độ (x, y, z).

        Args:
            x, y, z: Tọa độ mục tiêu (mm)
        """
        cmd = f"MOVETO {x} {y} {z}"
        response = self.serial.send_and_wait(cmd)
        return response is not None and "ACK" in response

    def gripper(self, state: str) -> bool:
        """
        Điều khiển gripper.

        Args:
            state: "open" hoặc "close"
        """
        cmd = f"GRIPPER {state.upper()}"
        response = self.serial.send_and_wait(cmd)
        return response is not None and "ACK" in response

    def home(self) -> bool:
        """Đưa tất cả khớp về vị trí home."""
        response = self.serial.send_and_wait("HOME")
        return response is not None and "ACK" in response

    def emergency_stop(self):
        """Dừng khẩn cấp."""
        self.serial.send("ESTOP")
