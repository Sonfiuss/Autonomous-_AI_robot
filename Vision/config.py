"""
Configuration file for Hand Angle Detection and Servo Control System
"""

# ============================================
# CAMERA SETTINGS
# ============================================
CAMERA_INDEX = 0
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_FPS = 30

# ============================================
# MEDIAPIPE SETTINGS
# ============================================
MEDIAPIPE_STATIC_IMAGE_MODE = False
MEDIAPIPE_MAX_HANDS = 2
MEDIAPIPE_MIN_DETECTION_CONFIDENCE = 0.7
MEDIAPIPE_MIN_TRACKING_CONFIDENCE = 0.7

# ============================================
# HAND ANGLE DETECTION
# ============================================
ANGLE_PINCH_THRESHOLD = 30  # degrees
ANGLE_MIN = 0
ANGLE_MAX = 180

# ============================================
# SERVO CONTROL SETTINGS
# ============================================
SERVO_DEFAULT_PORT = 'COM3'
SERVO_BAUDRATE = 115200
SERVO_NUM_SERVOS = 6
SERVO_TIMEOUT = 0.1  # seconds
SERVO_QUEUE_SIZE = 10

# Default servo angle mapping
SERVO_MIN_ANGLE = 30
SERVO_MAX_ANGLE = 150
SERVO_HOME_ANGLE = 90
SERVO_INVERT = False

# ============================================
# VISUALIZATION SETTINGS
# ============================================
WINDOW_NAME = "Hand Angle Detector"
FONT = 1  # cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.7
FONT_THICKNESS = 2

# Colors (BGR format)
COLOR_WRIST = (255, 255, 0)      # Cyan
COLOR_THUMB = (255, 0, 0)        # Blue
COLOR_INDEX = (0, 255, 0)        # Green
COLOR_LINE = (255, 0, 255)       # Magenta
COLOR_ANGLE = (0, 255, 255)      # Yellow
COLOR_TEXT = (0, 0, 255)         # Red
COLOR_PINCH = (0, 255, 0)        # Green
COLOR_OPEN = (0, 165, 255)       # Orange

# ============================================
# KEYBOARD SHORTCUTS
# ============================================
KEY_QUIT = ord('q')
KEY_SAVE = ord('s')
KEY_HOME = ord('h')
KEY_CALIBRATE = ord('c')

# ============================================
# LOGGING
# ============================================
LOG_LEVEL = "INFO"  # DEBUG, INFO, WARNING, ERROR
ENABLE_FPS_DISPLAY = True
ENABLE_STATISTICS = False
