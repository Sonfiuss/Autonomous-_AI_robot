#!/bin/bash
# Flash ESP32 unified controller firmware.
#
# Usage: ./upload_esp32.sh [port]
#        ./upload_esp32.sh /dev/ttyUSB0   (default)

set -e

SKETCH="$(cd "$(dirname "$0")/esp32_unified_controller" && pwd)"
FQBN="esp32:esp32:esp32"
PORT="${1:-/dev/ttyUSB0}"

echo "=== ESP32 Upload ==="
echo "Sketch : $SKETCH"
echo "Board  : $FQBN"
echo "Port   : $PORT"
echo ""

if ! command -v arduino-cli &>/dev/null; then
    echo "ERROR: arduino-cli not found."
    echo "Install: curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    exit 1
fi

echo "[1/2] Compiling..."
arduino-cli compile --fqbn "$FQBN" "$SKETCH"

echo "[2/2] Uploading to $PORT..."
arduino-cli upload --fqbn "$FQBN" --port "$PORT" "$SKETCH"

echo ""
echo "Done. ESP32 ready on $PORT."
