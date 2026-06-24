#!/usr/bin/env bash
# ===========================================================================
# run_armmask.sh  -  one-shot: build the AD-Census binary (if missing) + run
# the full preprocess pipeline with arm masking and the free-space depth grid.
#
# Pipeline (tools/preprocess_run.py):
#   rectify -> denoise+CLAHE -> AD-Census -> ARM MASK -> clean -> zone -> FREE-SPACE
#
# Usage:
#   ./run_armmask.sh                          # default test pair
#   ./run_armmask.sh L.jpg R.jpg captures/out # your own pair + prefix
#   EXTRA="--arm-near-cm 45 --focal 600" ./run_armmask.sh   # tune thresholds
#
# Works on Jetson/Linux and on Windows under the 'MSYS2 MinGW 64-bit' shell.
# The C++ binary needs a MinGW/Linux OpenCV; the Python wrapper needs Python
# with the cv2 module (on the Windows dev box that is
#   C:\msys64\mingw64\bin\python  ->  pacman -S mingw-w64-x86_64-python-opencv).
# ===========================================================================
set -e
cd "$(dirname "$0")"

CAP="captures"
LEFT="${1:-$CAP/left_20260610_001156.jpg}"
RIGHT="${2:-$CAP/right_20260610_001156.jpg}"
PREFIX="${3:-$CAP/armtest}"
NDISP="${NDISP:-128}"
EXTRA="${EXTRA:-}"          # extra flags passed straight to preprocess_run.py

# --- 1. ensure the C++ binary exists (build via build.sh if not) ------------
BIN=""
for c in adcensus_depth.exe adcensus_depth build/adcensus_depth.exe build/adcensus_depth; do
    [ -f "$c" ] && BIN="$c" && break
done
if [ -z "$BIN" ]; then
    echo "==> adcensus binary missing - building via build.sh"
    ./build.sh
    for c in adcensus_depth.exe adcensus_depth; do
        [ -f "$c" ] && BIN="$c" && break
    done
fi
[ -n "$BIN" ] || { echo "ERROR: build produced no adcensus binary." >&2; exit 1; }
echo "==> using binary: $BIN"

# --- 2. find a Python that can 'import cv2' --------------------------------
PY=""
for cand in "$PYTHON" python python3 /c/msys64/mingw64/bin/python; do
    [ -n "$cand" ] || continue
    if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import cv2" >/dev/null 2>&1; then
        PY="$cand"; break
    fi
done
if [ -z "$PY" ]; then
    echo "ERROR: no Python with the cv2 module found." >&2
    echo "  Windows/MSYS2: pacman -S mingw-w64-x86_64-python-opencv" >&2
    echo "                 then re-run, or set PYTHON=C:/msys64/mingw64/bin/python" >&2
    echo "  Jetson/Linux:  sudo apt install python3-opencv" >&2
    exit 1
fi
echo "==> using python : $PY ($("$PY" -c 'import cv2;print("opencv",cv2.__version__)'))"

# --- 3. sanity: input images ----------------------------------------------
for f in "$LEFT" "$RIGHT"; do
    [ -f "$f" ] || { echo "ERROR: input image not found: $f" >&2; exit 1; }
done

# --- 4. run the full pipeline ----------------------------------------------
echo
echo "==> running pipeline"
echo "    left  : $LEFT"
echo "    right : $RIGHT"
echo "    prefix: $PREFIX   ndisp: $NDISP   extra: ${EXTRA:-<none>}"
echo
# (preprocess_run.py auto-detects the binary with an absolute path; passing a
#  relative --bin breaks Windows CreateProcess, so we let it find $BIN itself.)
"$PY" tools/preprocess_run.py \
    --left "$LEFT" --right "$RIGHT" \
    --out-prefix "$PREFIX" --ndisp "$NDISP" $EXTRA

# --- 5. list outputs to eyeball --------------------------------------------
echo
echo "==> outputs (eyeball these):"
for suffix in _armmask _freespace _zones _clean_color _disp_color; do
    f="${PREFIX}${suffix}.png"
    [ -f "$f" ] && echo "    $f"
done
