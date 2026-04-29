"""
Shared Constants

Hằng số dùng chung giữa các modules.
"""

# === Robot Dimensions (mm) ===
ROBOT_BASE_RADIUS = 250       # Bán kính base
ROBOT_BASE_HEIGHT = 150       # Chiều cao base
WHEEL_RADIUS = 50             # Bán kính bánh xe
WHEEL_COUNT = 3               # Số bánh omni wheel

# === Motion Limits ===
MAX_LINEAR_SPEED = 1000       # mm/s
MAX_ANGULAR_SPEED = 3.14      # rad/s (~ 180°/s)
MAX_ACCELERATION = 500        # mm/s²

# === Arm Limits ===
ARM_JOINT_COUNT = 6
ARM_MAX_REACH = 500           # mm
ARM_JOINT_LIMITS = [          # (min_deg, max_deg) cho từng khớp
    (-180, 180),
    (-90, 90),
    (-120, 120),
    (-180, 180),
    (-90, 90),
    (-180, 180),
]

# === Sensor Constants ===
ULTRASONIC_MAX_RANGE = 4000   # mm
IMU_UPDATE_RATE = 100         # Hz
ENCODER_TICKS_PER_REV = 2000

# === Communication ===
DEFAULT_BAUDRATE = 115200
SERIAL_TIMEOUT = 1.0          # seconds
COMMAND_TIMEOUT = 2.0         # seconds

# === AI ===
CAMERA_RESOLUTION = (640, 480)
CAMERA_FPS = 30
DETECTION_CONFIDENCE_THRESHOLD = 0.5
