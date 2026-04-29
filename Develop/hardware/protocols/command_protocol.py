"""
Command Protocol

Định nghĩa protocol giao tiếp giữa Python API và firmware.
"""

# === Command Format ===
# Gửi: "<CMD> [PARAMS]\n"
# Nhận: "ACK: <CMD>" hoặc "ERR: <message>"
#
# === Drive Commands ===
# VEL <vx> <vy> <omega>    - Đặt vận tốc
# ESTOP                     - Dừng khẩn cấp
#
# === Arm Commands ===
# JOINT <id> <angle>        - Di chuyển khớp
# MOVETO <x> <y> <z>        - Di chuyển end-effector
# GRIPPER <OPEN|CLOSE>      - Điều khiển gripper
# HOME                      - Về vị trí home
# ESTOP                     - Dừng khẩn cấp
#
# === Sensor Commands ===
# READ_ALL                  - Đọc tất cả cảm biến (JSON response)
# READ_US                   - Đọc cảm biến siêu âm
# READ_IMU                  - Đọc IMU
# READ_ENC                  - Đọc encoder


class CommandType:
    """Enum-like cho các loại lệnh."""
    # Drive
    VELOCITY = "VEL"
    EMERGENCY_STOP = "ESTOP"

    # Arm
    JOINT = "JOINT"
    MOVE_TO = "MOVETO"
    GRIPPER = "GRIPPER"
    HOME = "HOME"

    # Sensor
    READ_ALL = "READ_ALL"
    READ_ULTRASONIC = "READ_US"
    READ_IMU = "READ_IMU"
    READ_ENCODER = "READ_ENC"


class ResponseType:
    """Enum-like cho các loại phản hồi."""
    ACK = "ACK"
    ERROR = "ERR"
    DATA = "DATA"


def parse_response(raw: str) -> dict:
    """
    Parse phản hồi từ firmware.

    Returns:
        dict: {"type": "ACK"|"ERR"|"DATA", "content": str}
    """
    if raw.startswith("ACK"):
        return {"type": ResponseType.ACK, "content": raw[5:]}
    elif raw.startswith("ERR"):
        return {"type": ResponseType.ERROR, "content": raw[5:]}
    else:
        return {"type": ResponseType.DATA, "content": raw}
