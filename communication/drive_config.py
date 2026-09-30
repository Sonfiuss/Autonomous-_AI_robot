"""Constants for the text-command drive demo. Import-only, no side effects."""
import os
import sys

COMM_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(COMM_DIR, ".."))
CM_DIR = os.path.join(REPO_ROOT, "project", "src", "CM")        # llm_client.py + mc_client.py
PROMPT_FILE = os.path.join(COMM_DIR, "prompts", "drive_command.json")
VISION_DIR = os.path.join(REPO_ROOT, "vision")
REALROOM_DIR = os.path.join(REPO_ROOT, "realroom")             # the real room's map + the robot's pose in it
RECORDER_SCRIPT = os.path.join(VISION_DIR, "record.py")
DRIVE_MAP_SCRIPT = os.path.join(VISION_DIR, "drive_map.py")      # a run's recording -> the room's map
OUTPUT_DIR = os.path.join(VISION_DIR, "output")                  # one sub-folder per run
ROBOT_LOCK_FILE = os.path.join(OUTPUT_DIR, ".robot.lock")         # flock: one demo_drive drives the robot at a time
ROBOT_LINK_BIN = os.path.join(REPO_ROOT, "motivation", "jetson", "build", "robot_link")
DEFAULT_PORT = "/dev/ttyUSB0"

# ---- command validator (L2 -> code). A step outside these is refused, never clipped.
MAX_STEPS = 10                   # steps in one command
MIN_LINEAR_M = 0.01
MAX_LINEAR_M = 3.0               # the demo floor is a room, and odometry is open loop
MIN_TURN_DEG = 1.0
MAX_TURN_DEG = 360.0
DEFAULT_TURN_DEG = 90.0          # "re trai" with no angle: filled in HERE, never by the LLM
TURN_AROUND_DEG = 180.0          # "quay dau" with no angle
# A turn with no side ("quay tai cho 180 do", "rotate 360 degrees") goes LEFT when it is one of these: a half
# or full turn ends facing the same way either side. Any other angle is asked, never guessed.
SIDELESS_TURN_DEG = (180.0, 360.0)
NUMBER_MATCH_TOL = 1e-6          # an LLM value must equal a number the user wrote, to this tolerance
MAX_VALIDATION_RETRIES = 1       # re-ask the LLM once with the validator error before giving up

LINEAR_UNITS_M = {"mm": 0.001, "cm": 0.01, "m": 1.0}
ANGLE_UNIT = "deg"
REV_UNIT = "rev"                 # "vong", "turn", "circle": converted HERE, never by the LLM
ANGLE_UNITS_DEG = {ANGLE_UNIT: 1.0, REV_UNIT: 360.0}
DEFAULT_REVS = 1.0               # "quay mot vong tron" / "a full circle" with no number: one turn
CM_PER_M = 100.0

# ---- motion (primitives -> wire). 0 would mean "compiled default" to MC and SEQ; these are the
# same defaults, spelled out so the preview and robot_link are handed identical numbers.
CRUISE_SPEED_M_S = 0.15          # mc::cfg::CRUISE_SPEED_M_S
YAW_RATE_RAD_S = 0.8             # mc::cfg::YAW_RATE_RAD_S
MAX_CRUISE_SPEED_M_S = 0.4       # --speed ceiling for the demo (chassis limit 0.45 forward at r 0.040 m)
MAX_YAW_RATE_RAD_S = 1.75        # --yaw-rate ceiling for the demo (chassis limit 1.77 at L 0.2217 m)
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
MAP_BUILD_TIMEOUT_S = 600.0                 # drive_map on a run (a 35 s run took 17 s on the Orin, 2026-09-30)
# `robot_link --stop` before every plan: a partial line left in the ESP32's RX buffer once glued onto
# the plan's first T and failed it with E2 (2026-09-26). The S line is newline-terminated, so it ends
# that fragment (at worst the fragment + S is refused alone) and the plan's first line starts clean.
ROBOT_LINK_STOP_TIMEOUT_S = 5.0             # --stop = S + robot_link's settle (a few hundred ms)

# Gemini out of quota or overloaded (HTTP 429 / 503; the free tier allows a few requests a minute) -> this model,
# whose quota is its own, before the rules. Read from the environment (CM's .env) when the parser starts;
# "" = no second model. The lite model refuses CM's thinking_budget (400): it is sent this body instead.
GEMINI_FALLBACK_ENV = "CM_GEMINI_FALLBACK_MODEL"
GEMINI_FALLBACK_MODEL = "gemini-3.5-flash-lite"
GEMINI_FALLBACK_EXTRA_BODY = {}
GEMINI_PRIMARY_RETRIES = 0       # the SDK's retries of the first model when a second one is there: switch, not wait
QUOTA_COOLDOWN_S = 600.0         # a model out of quota is not asked again for this long (per process)


def add_cm_to_path():
    """Makes CM's flat modules (llm_client, mc_client and their `config`) importable. Called by the
    modules that need them, at the point they need them, so importing this file stays side-effect free."""
    if CM_DIR not in sys.path:
        sys.path.insert(0, CM_DIR)


def add_vision_to_path():
    """Makes vision/'s flat modules (leg_odometry, drive_timeline, frame_motion...) importable; same rule
    as add_cm_to_path."""
    if VISION_DIR not in sys.path:
        sys.path.insert(0, VISION_DIR)


def add_realroom_to_path():
    """Makes realroom/'s flat modules (map_store, planner) importable; same rule as add_cm_to_path."""
    if REALROOM_DIR not in sys.path:
        sys.path.insert(0, REALROOM_DIR)
