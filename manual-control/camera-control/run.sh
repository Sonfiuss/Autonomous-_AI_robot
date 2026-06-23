#!/usr/bin/env bash
# One command: flash ESP32 firmware → build Jetson app → run keypress control.
#
#   ./run.sh                 flash firmware, build, run   (full pipeline)
#   ./run.sh --no-flash      skip ESP32 upload, just build + run
#   ./run.sh --port /dev/ttyUSB1   use a different serial port
#   ./run.sh clean           wipe build dir first
#
# Requires: cmake, g++, arduino-cli (for the --flash step).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
FW_DIR="$SCRIPT_DIR/esp32_firmware"

PORT="/dev/ttyUSB0"
FQBN="esp32:esp32:esp32"      # generic ESP32 dev board
DO_FLASH=1

for arg in "$@"; do
    case "$arg" in
        clean)      rm -rf "$BUILD_DIR" ;;
        --no-flash) DO_FLASH=0 ;;
    esac
done
# --port <dev> (consume the value that follows)
while [[ $# -gt 0 ]]; do
    case "$1" in
        --port) PORT="$2"; shift 2 ;;
        *)      shift ;;
    esac
done

# ── 1. Flash ESP32 firmware ──────────────────────────────────────────────────
if [[ "$DO_FLASH" -eq 1 ]]; then
    echo ">> [1/3] Flashing ESP32 firmware on $PORT"
    if ! command -v arduino-cli >/dev/null; then
        echo "!! arduino-cli not found — re-run with --no-flash, or install it." >&2
        exit 1
    fi
    # Ensure ESP32 core + servo library are present (no-op if already installed).
    arduino-cli core list | grep -q '^esp32:esp32' || {
        arduino-cli config add board_manager.additional_urls \
            https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json || true
        arduino-cli core update-index
        arduino-cli core install esp32:esp32
    }
    arduino-cli lib list | grep -qi 'Adafruit PWM Servo Driver' || \
        arduino-cli lib install "Adafruit PWM Servo Driver Library"

    arduino-cli compile --fqbn "$FQBN" "$FW_DIR"
    arduino-cli upload   --fqbn "$FQBN" --port "$PORT" "$FW_DIR"
    echo ">> firmware uploaded; giving the board a moment to reboot"
    sleep 2
else
    echo ">> [1/3] Skipping ESP32 flash (--no-flash)"
fi

# ── 2. Build Jetson app ──────────────────────────────────────────────────────
echo ">> [2/3] Building host app"
cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j"$(nproc)"

# ── 3. Run ───────────────────────────────────────────────────────────────────
echo ">> [3/3] Launching control (port $PORT)"
exec "$BUILD_DIR/camera_control" --port "$PORT"
