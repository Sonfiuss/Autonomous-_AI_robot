"""
Sensor Reader API

Đọc dữ liệu từ các cảm biến qua serial (từ STM32 sensor hub hoặc Arduino).
"""

import json
from typing import Optional
from .serial_interface import SerialInterface


class SensorReader:
    """Đọc dữ liệu cảm biến từ vi điều khiển."""

    def __init__(self, port: str, baudrate: int = 115200):
        self.serial = SerialInterface(port, baudrate)
        self._latest_data = {}

    def connect(self) -> bool:
        return self.serial.connect()

    def disconnect(self):
        self.serial.disconnect()

    def read_sensors(self) -> Optional[dict]:
        """
        Đọc tất cả dữ liệu cảm biến.

        Returns:
            dict: {"ultrasonic": [...], "imu": {...}, "encoder": [...], ...}
        """
        response = self.serial.send_and_wait("READ_ALL")
        if response:
            try:
                self._latest_data = json.loads(response)
                return self._latest_data
            except json.JSONDecodeError:
                return None
        return None

    def read_ultrasonic(self) -> Optional[list]:
        """Đọc cảm biến siêu âm (khoảng cách cm)."""
        response = self.serial.send_and_wait("READ_US")
        if response:
            try:
                return json.loads(response)
            except json.JSONDecodeError:
                return None
        return None

    def read_imu(self) -> Optional[dict]:
        """
        Đọc dữ liệu IMU.

        Returns:
            dict: {"accel": [ax, ay, az], "gyro": [gx, gy, gz], "heading": float}
        """
        response = self.serial.send_and_wait("READ_IMU")
        if response:
            try:
                return json.loads(response)
            except json.JSONDecodeError:
                return None
        return None

    def read_encoders(self) -> Optional[list]:
        """Đọc giá trị encoder từ các motor."""
        response = self.serial.send_and_wait("READ_ENC")
        if response:
            try:
                return json.loads(response)
            except json.JSONDecodeError:
                return None
        return None

    @property
    def latest_data(self) -> dict:
        """Trả về dữ liệu cảm biến gần nhất đã đọc được."""
        return self._latest_data
