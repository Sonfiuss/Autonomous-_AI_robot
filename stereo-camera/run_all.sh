#!/usr/bin/env bash
#
# run_all.sh — build, flash firmware, scan, and view the point cloud.
#
# Steps (in order):
#   1. Build the C++ pipeline (cmake + make)
#   2. Compile + flash the ESP32 servo firmware (MOVE/OK, channels 12/14)
#   3. Run the stereo scan  -> build/scan.ply
#   4. Serve the cloud in the browser at http://localhost:8080
#
# Usage:
#   ./run_all.sh                 # do everything
#   ./run_all.sh --no-flash      # skip ESP32 flash (firmware already on chip)
#   ./run_all.sh --no-scan       # skip scanning, just view existing scan.ply
#   ./run_all.sh --view-only     # only launch the viewer
#
# Options:
#   --port DEV     serial port           (default /dev/ttyUSB0)
#   --left N       left camera index     (default 0)
#   --right N      right camera index    (default 2)
#   --output FILE  cloud output path     (default scan.ply, in build/)
#   --http PORT    viewer HTTP port      (default 8080)
#   --fqbn STR     arduino board fqbn    (default esp32:esp32:esp32)

set -euo pipefail

# ── Defaults ──────────────────────────────────────────────────────────────────
PORT="/dev/ttyUSB0"
LEFT=0
RIGHT=2
OUTPUT="scan.ply"
HTTP_PORT=8080
FQBN="esp32:esp32:esp32"
DO_BUILD=1
DO_FLASH=1
DO_SCAN=1
DO_VIEW=1

# ── Parse args ────────────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-build)  DO_BUILD=0; shift ;;
        --no-flash)  DO_FLASH=0; shift ;;
        --no-scan)   DO_SCAN=0;  shift ;;
        --view-only) DO_BUILD=0; DO_FLASH=0; DO_SCAN=0; shift ;;
        --port)      PORT="$2";      shift 2 ;;
        --left)      LEFT="$2";      shift 2 ;;
        --right)     RIGHT="$2";     shift 2 ;;
        --output)    OUTPUT="$2";    shift 2 ;;
        --http)      HTTP_PORT="$2"; shift 2 ;;
        --fqbn)      FQBN="$2";      shift 2 ;;
        -h|--help)   sed -n '2,30p' "$0"; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done

# ── Locate project root (this script's dir) ───────────────────────────────────
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

say() { printf '\n\033[1;36m=== %s ===\033[0m\n' "$1"; }

# ── 1. Build C++ ──────────────────────────────────────────────────────────────
if [[ $DO_BUILD -eq 1 ]]; then
    say "1/4  Building C++ pipeline"
    mkdir -p build
    ( cd build && cmake .. >/dev/null && make -j"$(nproc)" )
fi

# ── 2. Flash ESP32 firmware ───────────────────────────────────────────────────
if [[ $DO_FLASH -eq 1 ]]; then
    say "2/4  Compiling + flashing ESP32 firmware ($FQBN)"
    FW="esp32/servo_driver/servo_driver.ino"
    arduino-cli compile --fqbn "$FQBN" "$FW"
    arduino-cli upload -p "$PORT" --fqbn "$FQBN" "$FW"
    sleep 2   # let the board reboot into the new firmware
fi

# ── Free the cameras (the viewer/live server grabs them) ──────────────────────
if [[ $DO_SCAN -eq 1 || $DO_VIEW -eq 1 ]]; then
    pkill -f "map3d_server.py" 2>/dev/null && sleep 1 || true
fi

# ── 3. Run the scan ───────────────────────────────────────────────────────────
if [[ $DO_SCAN -eq 1 ]]; then
    say "3/4  Scanning (cameras $LEFT/$RIGHT on $PORT)"
    ( cd build && rm -f scan.config && \
      ./stereo_scan --port "$PORT" --left "$LEFT" --right "$RIGHT" --output "$OUTPUT" )
fi

# ── 4. View ───────────────────────────────────────────────────────────────────
if [[ $DO_VIEW -eq 1 ]]; then
    PLY="build/$OUTPUT"
    if [[ ! -f "$PLY" ]]; then
        echo "No cloud at $PLY — run a scan first (omit --no-scan)." >&2
        exit 1
    fi
    say "4/4  Serving $PLY at http://localhost:$HTTP_PORT"
    echo "Open http://localhost:$HTTP_PORT   (Ctrl+C to stop)"
    exec python3 tools/map3d_server.py --ply "$PLY" --port "$HTTP_PORT"
fi
