"""
Serial Interface

Lớp cơ sở giao tiếp serial với vi điều khiển (Arduino/STM32).
"""

import serial
import time
import json
from typing import Optional


class SerialInterface:
    """Giao tiếp serial cơ bản với vi điều khiển."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 1.0):
        """
        Args:
            port: COM port (e.g., "COM3" trên Windows, "/dev/ttyUSB0" trên Linux)
            baudrate: Tốc độ baud
            timeout: Timeout đọc (giây)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None

    def connect(self) -> bool:
        """Kết nối serial port."""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
            time.sleep(2)  # Chờ Arduino reset
            return True
        except serial.SerialException as e:
            print(f"Lỗi kết nối serial: {e}")
            return False

    def disconnect(self):
        """Ngắt kết nối."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

    def send(self, data: str) -> bool:
        """Gửi chuỗi dữ liệu qua serial."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        try:
            self.serial_conn.write((data + "\n").encode("utf-8"))
            return True
        except serial.SerialException:
            return False

    def send_json(self, data: dict) -> bool:
        """Gửi dữ liệu dạng JSON."""
        return self.send(json.dumps(data))

    def read_line(self) -> Optional[str]:
        """Đọc 1 dòng từ serial."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        try:
            line = self.serial_conn.readline().decode("utf-8").strip()
            return line if line else None
        except serial.SerialException:
            return None

    def send_and_wait(self, data: str, timeout: float = 2.0) -> Optional[str]:
        """Gửi lệnh và chờ phản hồi."""
        self.send(data)
        start = time.time()
        while time.time() - start < timeout:
            response = self.read_line()
            if response:
                return response
        return None

    @property
    def is_connected(self) -> bool:
        return self.serial_conn is not None and self.serial_conn.is_open

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
