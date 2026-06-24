#!/usr/bin/env bash
# ===========================================================================
# run_segment2d.sh  -  one-shot: pure 2D object/floor segmentation.
#
# Outlines objects in a single image; the remainder is the floor/background.
# No AD-Census binary needed — runs in ~1s. Writes 8 intermediate PNGs.
#
# Usage:
#   ./run_segment2d.sh                              # default test image
#   ./run_segment2d.sh captures/myimg.jpg seg2      # your image + prefix
#   EXTRA="--floor-thresh 0.5 --illum clahe" ./run_segment2d.sh   # tune
#
# Windows: run in the 'MSYS2 MinGW 64-bit' shell (needs python with cv2:
#   pacman -S mingw-w64-x86_64-python-opencv ).
# ===========================================================================
set -e
cd "$(dirname "$0")"

CAP="captures"
IMG="${1:-$CAP/left_20260610_001156.jpg}"
PREFIX="${2:-$CAP/seg}"
EXTRA="${EXTRA:-}"

# find a Python that can import cv2
PY=""
for cand in "$PYTHON" python python3 /c/msys64/mingw64/bin/python; do
    [ -n "$cand" ] || continue
    if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import cv2" >/dev/null 2>&1; then
        PY="$cand"; break
    fi
done
if [ -z "$PY" ]; then
    echo "ERROR: no Python with cv2 found." >&2
    echo "  MSYS2: pacman -S mingw-w64-x86_64-python-opencv" >&2
    echo "  Linux: sudo apt install python3-opencv" >&2
    exit 1
fi
[ -f "$IMG" ] || { echo "ERROR: image not found: $IMG" >&2; exit 1; }

echo "==> python: $PY   image: $IMG   prefix: $PREFIX   extra: ${EXTRA:-<none>}"
echo
"$PY" tools/segment2d.py --image "$IMG" --out-prefix "$PREFIX" $EXTRA

echo
echo "==> stage outputs (eyeball in order):"
for s in _01_input _02_illum _03_floorseed _04_floorprob \
         _05_objmask_raw _06_objmask _07_objects _08_background; do
    f="${PREFIX}${s}.png"
    [ -f "$f" ] && echo "    $f"
done
