#!/usr/bin/env bash
# servo.sh — quick servo control wrapper
#
# Usage:
#   ./scripts/servo.sh <pan> <tilt>          move to angle
#   ./scripts/servo.sh 0 0                   reset to origin
#   ./scripts/servo.sh reset                 same as 0 0
#   ./scripts/servo.sh <pan> <tilt> [port]   override port
#
# Examples:
#   ./scripts/servo.sh 0 0
#   ./scripts/servo.sh 45 -20
#   ./scripts/servo.sh reset
#   ./scripts/servo.sh 0 0 /dev/ttyUSB1

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/../build"
BINARY="$BUILD_DIR/servo_ctrl"
PORT="${SERVO_PORT:-/dev/ttyUSB0}"

# ── Check binary exists ───────────────────────────────────────────────────────
if [ ! -f "$BINARY" ]; then
    echo "[servo.sh] Binary not found. Building..."
    cmake -S "$SCRIPT_DIR/.." -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -q
    cmake --build "$BUILD_DIR" --target servo_ctrl -j"$(nproc)"
fi

# ── Parse args ────────────────────────────────────────────────────────────────
if [ $# -eq 0 ]; then
    echo "Usage: $0 <pan> <tilt> [port]"
    echo "       $0 reset [port]"
    exit 1
fi

if [ "$1" = "reset" ]; then
    PAN=0
    TILT=0
    [ -n "$2" ] && PORT="$2"
else
    if [ $# -lt 2 ]; then
        echo "Error: need pan and tilt angles"
        echo "Usage: $0 <pan> <tilt> [port]"
        exit 1
    fi
    PAN="$1"
    TILT="$2"
    [ -n "$3" ] && PORT="$3"
fi

# ── Run ───────────────────────────────────────────────────────────────────────
exec "$BINARY" --port "$PORT" D "$PAN" "$TILT"
