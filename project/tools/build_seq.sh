#!/usr/bin/env bash
# Builds the SEQ sequencer tests without CMake.
#   tools/build_seq.sh           -> build/seq/test_seq
# SEQ is the Jetson-side plan sequencer. It is pure logic (no port, no clock), so
# it builds and runs on a PC; that is the whole reason it lives here and not in
# motivation/jetson, which needs POSIX termios.
# Override the compiler with CXX=...
set -e
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
# MinGW toolchains need their own bin dir first on PATH to find cc1plus and their DLLs.
export PATH="$(dirname "$CXX"):$PATH"
OUT=build/seq
mkdir -p "$OUT"
# -fno-exceptions matches the rest of project/.
FLAGS="-std=c++14 -O2 -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti -Iinclude -Iconfig"
# RM for the leg-duration estimate (the same maths the firmware plans a leg with),
# LINK for the wire commands the sequencer emits.
SRC="src/SEQ/sequencer.cpp src/LINK/protocol.cpp"
SRC="$SRC src/RM/omni_kinematics.cpp src/RM/velocity_profile.cpp src/RM/speed_limit.cpp"
echo "compiler: $CXX"
$CXX $FLAGS $SRC tests/test_seq.cpp -o "$OUT/test_seq"
echo "built: $OUT/test_seq"
