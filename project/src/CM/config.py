"""Constants for the CM grounding module. Import-only, no side effects."""
import os

from dotenv import load_dotenv

CM_DIR = os.path.dirname(os.path.abspath(__file__))
ENV_FILE = os.path.join(CM_DIR, ".env")                        # API keys live here (git-ignored)
load_dotenv(ENV_FILE)                                          # does not override vars already set in the shell
PROJECT_DIR = os.path.abspath(os.path.join(CM_DIR, "..", ".."))               # project/ (C++ sources + build)
REPO_ROOT = os.path.abspath(os.path.join(CM_DIR, "..", "..", ".."))
ROOM_DIR = os.path.join(REPO_ROOT, "simulation", "room")      # room_generator.py lives here
SCENES_DIR = os.path.join(ROOM_DIR, "scenes")                  # shared with the room simulator
PROMPT_FILE = os.path.join(CM_DIR, "prompts", "grounding.json")
STATIC_DIR = os.path.join(CM_DIR, "static")

PORT = 5001

# ---- candidate geometry (L3)
SPOT_CLEARANCE_M = 0.05          # gap between robot hull and the object side
CORNER_MIN_SIDE_M = 0.45         # sides shorter than this get only a "center" spot (2 x hull radius)
DOOR_INSIDE_OFFSET_M = 0.15      # door spot: hull radius + this, inside the room
ROUND_DIGITS = 3

# ---- LLM (L2)
PROVIDER = os.environ.get("CM_PROVIDER", "gemini")             # gemini | openai | anthropic | fake
OPENAI_MODEL = os.environ.get("CM_OPENAI_MODEL", "gpt-4o-mini")
GEMINI_MODEL = os.environ.get("CM_GEMINI_MODEL", "gemini-2.5-flash")
GEMINI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai/"   # OpenAI-compatible endpoint
GEMINI_MAX_TOKENS = 8192         # Gemini 2.5 bills thinking tokens against max_tokens -> keep headroom
GEMINI_EXTRA_BODY = {"extra_body": {"google": {"thinking_config": {"thinking_budget": 0}}}}   # no thinking (Gemini compat nests it)
ANTHROPIC_MODEL = os.environ.get("CM_ANTHROPIC_MODEL", "claude-opus-5")
LLM_MAX_TOKENS = 2048            # replies are one short JSON object
LLM_TIMEOUT_S = 60.0
MAX_VALIDATION_RETRIES = 1       # re-ask the LLM once with the validator error before giving up

# ---- MV path planner (L4, C library called through ctypes; see mv_client.py)
MV_MAX_PATH = 4098               # output cells; MV grid is at most 240 x 240 at 0.05 m
MV_MAX_PRIMS = 2 * MV_MAX_PATH + 2
MV_MAX_GRID_BYTES = 240 * 240    # matches mv::MAX_CELLS in project/config/constants.h
MV_CELL_M = 0.05                 # matches mv::GRID_RES_M (cell edge), for tests and grid overlays
MV_DEFAULT_ROBOT_RADIUS_M = 0.225   # used only when the scene carries no robot radius
MV_ROUND_DIGITS = 3              # metres in the JSON reply (mm precision)
MV_FINE_ROUND_DIGITS = 4         # radians and cell size need one digit more than metres
MV_HOLONOMIC = os.environ.get("CM_MV_HOLONOMIC", "0") == "1"   # 1: MOVE dx dy instead of ROTATE/FORWARD

# ---- MC motion executor (L5, C library called through ctypes; see mc_client.py)
MC_MAX_STEPS = 4096              # matches mc::cfg::MAX_TRAJECTORY_STEPS (81.9 s at 50 Hz)
MC_UI_MAX_ROWS = 600             # rows sent to the browser; longer runs are strided down
MC_HZ_ROUND_DIGITS = 1           # pulse rates are whole-ish numbers, one decimal is plenty
