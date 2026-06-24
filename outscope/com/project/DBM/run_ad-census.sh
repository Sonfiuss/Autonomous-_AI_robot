#!/usr/bin/env bash
# ===========================================================================
# run_ad-census.sh  -  one-shot: build + run AD-Census depth, get output.
#
# Usage:
#   ./run_ad-census.sh                       # build + run on default test pair
#   ./run_ad-census.sh L.png R.png out 0 128 # build + run on your own pair
#
# Works on Jetson/Linux and on Windows under MSYS2 MinGW64 / Git-Bash
# (needs a MinGW-built OpenCV; install with:
#   pacman -S mingw-w64-x86_64-gcc mingw-w64-x86_64-opencv pkg-config ).
# ===========================================================================
set -e
cd "$(dirname "$0")"

CXX="${CXX:-g++}"
OUT_BIN="adcensus_depth"
SRCS="main.cpp adcensus/ADCensusStereo.cpp adcensus/adcensus_util.cpp \
      adcensus/cost_computor.cpp adcensus/cross_aggregator.cpp \
      adcensus/scanline_optimizer.cpp adcensus/multistep_refiner.cpp"
CXXFLAGS="-std=c++14 -O2 -I. -w"

# --- default run arguments (the 640x480 capture pair in ./captures) ---------
CAP="captures"
LEFT="${1:-$CAP/left_20260610_001156.jpg}"
RIGHT="${2:-$CAP/right_20260610_001156.jpg}"
# default output prefix writes the PNGs into ./captures next to the inputs
PREFIX="${3:-$CAP/adcensus_out}"
MIND="${4:-0}"
MAXD="${5:-128}"

# --- sanity: compiler ------------------------------------------------------
if ! command -v "$CXX" >/dev/null 2>&1; then
    echo "ERROR: '$CXX' not found on PATH." >&2
    echo "  Jetson: sudo apt install build-essential" >&2
    echo "  Windows: use the 'MSYS2 MinGW 64-bit' shell (has g++)." >&2
    exit 1
fi

# --- resolve OpenCV via pkg-config -----------------------------------------
if command -v pkg-config >/dev/null 2>&1 && pkg-config --exists opencv4 2>/dev/null; then
    OCV=$(pkg-config --cflags --libs opencv4)
elif command -v pkg-config >/dev/null 2>&1 && pkg-config --exists opencv 2>/dev/null; then
    OCV=$(pkg-config --cflags --libs opencv)
else
    echo "ERROR: OpenCV not found via pkg-config." >&2
    echo "  Jetson: sudo apt install libopencv-dev pkg-config" >&2
    echo "  Windows/MSYS2: pacman -S mingw-w64-x86_64-opencv pkg-config" >&2
    exit 1
fi

# --- sanity: input images exist --------------------------------------------
if [ ! -f "$LEFT" ] || [ ! -f "$RIGHT" ]; then
    echo "ERROR: input image(s) not found:" >&2
    [ -f "$LEFT" ]  || echo "  missing left:  $LEFT"  >&2
    [ -f "$RIGHT" ] || echo "  missing right: $RIGHT" >&2
    exit 1
fi

# --- build -----------------------------------------------------------------
echo "==> [1/2] building $OUT_BIN"
echo "    $CXX $CXXFLAGS $OCV"
$CXX $CXXFLAGS $SRCS -o "$OUT_BIN" $OCV
echo "    OK"

# --- run -------------------------------------------------------------------
echo
echo "==> [2/2] running"
echo "    left : $LEFT"
echo "    right: $RIGHT"
echo "    disp : [$MIND, $MAXD]"
echo
./"$OUT_BIN" "$LEFT" "$RIGHT" "$PREFIX" "$MIND" "$MAXD"

echo
echo "==> output:"
ls -1 "${PREFIX}_disp.png" "${PREFIX}_disp_color.png" 2>/dev/null \
    || echo "  (no output produced - check matcher messages above)"
