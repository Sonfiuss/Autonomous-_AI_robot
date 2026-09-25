"""Constants for the text-command drive demo. Import-only, no side effects."""
import os
import sys

COMM_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(COMM_DIR, ".."))
CM_DIR = os.path.join(REPO_ROOT, "project", "src", "CM")        # llm_client.py + mc_client.py
PROMPT_FILE = os.path.join(COMM_DIR, "prompts", "drive_command.json")
VISION_DIR = os.path.join(REPO_ROOT, "vision")
RECORDER_SCRIPT = os.path.join(VISION_DIR, "record.py")
OUTPUT_DIR = os.path.join(VISION_DIR, "output")                  # one sub-folder per run
ROBOT_LINK_BIN = os.path.join(REPO_ROOT, "motivation", "jetson", "build", "robot_link")
DEFAULT_PORT = "/dev/ttyUSB0"

# ---- command validator (L2 -> code). A step outside these is refused, never clipped.
MAX_STEPS = 10                   # steps in one command
MIN_LINEAR_M = 0.01
MAX_LINEAR_M = 3.0               # the demo floor is a room, and odometry is open loop
MIN_TURN_DEG = 1.0
MAX_TURN_DEG = 360.0
DEFAULT_TURN_DEG = 90.0          # "re trai" with no angle: filled in HERE, never by the LLM
TURN_AROUND_DEG = 180.0          # "quay dau"
NUMBER_MATCH_TOL = 1e-6          # an LLM value must equal a number the user wrote, to this tolerance
MAX_VALIDATION_RETRIES = 1       # re-ask the LLM once with the validator error before giving up

LINEAR_UNITS_M = {"mm": 0.001, "cm": 0.01, "m": 1.0}
ANGLE_UNIT = "deg"
ANGLE_UNITS = (ANGLE_UNIT,)
CM_PER_M = 100.0

# ---- motion (primitives -> wire). 0 would mean "compiled default" to MC and SEQ; these are the
# same defaults, spelled out so the preview and robot_link are handed identical numbers.
CRUISE_SPEED_M_S = 0.15          # mc::cfg::CRUISE_SPEED_M_S
YAW_RATE_RAD_S = 0.8             # mc::cfg::YAW_RATE_RAD_S
MAX_CRUISE_SPEED_M_S = 0.5       # --speed ceiling for the demo (chassis limit ~0.62 forward)
MAX_YAW_RATE_RAD_S = 2.0         # --yaw-rate ceiling for the demo (chassis limit ~2.57)
# SEQ wraps every heading into (-180, 180] deg: a single 180 deg RIGHT turn would come out as a left
# one, and 360 as nothing. Turns are therefore cut into chunks strictly below 180.
TURN_CHUNK_MAX_DEG = 170.0
WIRE_DECIMALS = 3                # link::cfg::FLOAT_DECIMALS

# ---- run orchestration
RECORDER_READY_FILE = "recorder.ready"      # written by vision/record.py once its first frame is saved
RECORDER_STATUS_FILE = "status.txt"         # the line the recorder draws on every frame
RECORDER_READY_TIMEOUT_S = 120.0            # YOLO load + CUDA init + first camera frame
RECORDER_STOP_TIMEOUT_S = 15.0              # SIGINT -> video closed; then SIGKILL
VIDEO_TAIL_S = 2.0                          # keep filming after the robot stops
ROBOT_LINK_GRACE_S = 10.0                   # after Ctrl-C: time robot_link gets to send S and exit


def add_cm_to_path():
    """Makes CM's flat modules (llm_client, mc_client and their `config`) importable. Called by the
    modules that need them, at the point they need them, so importing this file stays side-effect free."""
    if CM_DIR not in sys.path:
        sys.path.insert(0, CM_DIR)
