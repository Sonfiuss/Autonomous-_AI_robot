#!/usr/bin/env bash
# ===========================================================================
# run_capture_checkerboard.sh  -  chụp cặp bàn cờ stereo + calibrate (CLI).
#
# Usage:
#   ./run_capture_checkerboard.sh                 # default board 9x6, sq=25mm
#   EXTRA="--square 30 --left 0 --right 2" ./run_capture_checkerboard.sh
#
# Tự dò python có cv2 (Jetson: python3 hệ thống; Windows/MSYS2:
#   C:\msys64\mingw64\bin\python).
# ===========================================================================
set -e
cd "$(dirname "$0")"

pick_python() {
    for p in python3 \
             /c/msys64/mingw64/bin/python \
             "C:/msys64/mingw64/bin/python" \
             python; do
        if "$p" -c "import cv2" >/dev/null 2>&1; then
            echo "$p"; return 0
        fi
    done
    return 1
}

PY=$(pick_python) || {
    echo "ERROR: không tìm thấy python có cv2." >&2
    echo "  Jetson : sudo apt install python3-opencv" >&2
    echo "  Windows: pacman -S mingw-w64-x86_64-python-opencv (MSYS2)" >&2
    exit 1
}

echo "==> python: $PY"
echo "==> tools/capture_checkerboard.py $EXTRA"
exec "$PY" tools/capture_checkerboard.py $EXTRA
