#!/usr/bin/env bash
# Build adcensus_depth without CMake, using g++ + pkg-config.
# Works on Jetson/Linux and on Windows under MSYS2/Git-Bash (with MinGW OpenCV).
set -e
cd "$(dirname "$0")"

CXX="${CXX:-g++}"
OUT="adcensus_depth"
SRCS="main.cpp adcensus/ADCensusStereo.cpp adcensus/adcensus_util.cpp \
      adcensus/cost_computor.cpp adcensus/cross_aggregator.cpp \
      adcensus/scanline_optimizer.cpp adcensus/multistep_refiner.cpp"
CXXFLAGS="-std=c++14 -O2 -I. -w"

# Resolve OpenCV flags (opencv4 first, then opencv).
if pkg-config --exists opencv4 2>/dev/null; then
    OCV=$(pkg-config --cflags --libs opencv4)
elif pkg-config --exists opencv 2>/dev/null; then
    OCV=$(pkg-config --cflags --libs opencv)
else
    echo "ERROR: OpenCV not found via pkg-config." >&2
    echo "  Jetson: sudo apt install libopencv-dev pkg-config" >&2
    echo "  MSYS2:  pacman -S mingw-w64-x86_64-opencv pkg-config" >&2
    exit 1
fi

echo "[build] $CXX $CXXFLAGS $OCV"
$CXX $CXXFLAGS $SRCS -o "$OUT" $OCV
echo "[OK] built $OUT"
echo
echo "Run example:"
echo "  ./$OUT ../stereo-camera/captures/left_20260610_001156.jpg \\"
echo "         ../stereo-camera/captures/right_20260610_001156.jpg adcensus_out 0 128"
