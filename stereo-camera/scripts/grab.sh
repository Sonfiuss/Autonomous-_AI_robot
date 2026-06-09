#!/usr/bin/env bash
# grab.sh — one-shot stereo image capture, no depth/servo needed
#
# Usage:
#   ./scripts/grab.sh              # save to captures/
#   ./scripts/grab.sh --show       # display after saving
#   ./scripts/grab.sh --out /tmp   # custom output dir
#   ./scripts/grab.sh --left 0 --right 2 --show
#
# All extra args are forwarded to grab_images.py

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 "$SCRIPT_DIR/grab_images.py" "$@"
