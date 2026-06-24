#!/usr/bin/env bash
# ===========================================================================
# run_calibrate_from_images.sh  -  định nghĩa config stereo từ ảnh ĐÃ LƯU.
#
# Usage:
#   DIR=captures/checkerboard/20260623_160738 ./run_calibrate_from_images.sh
#   DIR=... EXTRA="--rows 7 --cols 7 --square 18" ./run_calibrate_from_images.sh
#
# Bàn cờ 8x8 Ô  →  7x7 GÓC TRONG (mặc định). Đổi qua EXTRA nếu khác.
# Tự dò python có cv2 (Jetson python3; Windows/MSYS2 mingw64 python).
# ===========================================================================
set -e
cd "$(dirname "$0")"

: "${DIR:?Phai dat DIR=duong/dan/toi/session_anh}"

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
    exit 1
}

echo "==> python: $PY"
echo "==> tools/calibrate_from_images.py --dir $DIR $EXTRA"
exec "$PY" tools/calibrate_from_images.py --dir "$DIR" $EXTRA
