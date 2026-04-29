"""
Shared Configuration

Config chung cho tất cả modules.
"""

import os
import json
from pathlib import Path


class Config:
    """Quản lý cấu hình chung cho project."""

    # Đường dẫn gốc
    PROJECT_ROOT = Path(__file__).parent.parent.parent  # Autonomous_AI_robot/
    DEVELOP_ROOT = Path(__file__).parent.parent          # Develop/

    # Module paths
    AI_DIR = DEVELOP_ROOT / "ai"
    HARDWARE_DIR = DEVELOP_ROOT / "hardware"
    UI_DIR = DEVELOP_ROOT / "ui"
    SIMULATION_DIR = DEVELOP_ROOT / "simulation"

    # Legacy module paths (code gốc bên ngoài Develop/)
    VISION_DIR = PROJECT_ROOT / "Vision"
    ARD_ARMS_DIR = PROJECT_ROOT / "ARD_ARMS"
    OMNI_WHEEL_DIR = PROJECT_ROOT / "omni_wheel_module"
    STM32_DIR = PROJECT_ROOT / "VS_STM32"

    # Serial ports (mặc định, có thể override qua env hoặc config file)
    DRIVE_PORT = os.getenv("DRIVE_PORT", "COM3")
    ARM_PORT = os.getenv("ARM_PORT", "COM4")
    SENSOR_PORT = os.getenv("SENSOR_PORT", "COM5")
    BAUDRATE = int(os.getenv("BAUDRATE", "115200"))

    # Camera
    CAMERA_SOURCE = os.getenv("CAMERA_SOURCE", "0")  # Index hoặc RTSP URL

    # Server
    UI_HOST = os.getenv("UI_HOST", "0.0.0.0")
    UI_PORT = int(os.getenv("UI_PORT", "5000"))

    @classmethod
    def load_from_file(cls, filepath: str):
        """Load config từ JSON file."""
        with open(filepath, "r") as f:
            data = json.load(f)
        for key, value in data.items():
            if hasattr(cls, key.upper()):
                setattr(cls, key.upper(), value)

    @classmethod
    def to_dict(cls) -> dict:
        """Export config ra dict."""
        return {
            k: str(v) for k, v in vars(cls).items()
            if not k.startswith("_") and k.isupper()
        }
