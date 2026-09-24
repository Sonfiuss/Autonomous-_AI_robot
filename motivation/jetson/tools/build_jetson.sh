#!/usr/bin/env bash
# Builds the Jetson-side link tool.
#   tools/build_jetson.sh        -> build/robot_link
# POSIX only (termios): this does NOT build on Windows.
set -e
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
OUT=build
mkdir -p "$OUT"
# LINK comes from the shared project/ tree — the same protocol.cpp the firmware
# compiles, which is the point of putting it there. SEQ and the three RM files it
# needs come from there too: the sequencer estimates each leg's duration with the
# same maths the firmware plans the leg with, so the two cannot drift apart.
PROJECT=../../project
FLAGS="-std=c++14 -O2 -Wall -Wextra -Wpedantic -Iinclude -I$PROJECT/include -I$PROJECT/config"
SRC="src/serial_port.cpp src/robot_link.cpp src/mission_runner.cpp src/main.cpp"
SRC="$SRC $PROJECT/src/LINK/protocol.cpp $PROJECT/src/SEQ/sequencer.cpp"
SRC="$SRC $PROJECT/src/RM/omni_kinematics.cpp $PROJECT/src/RM/velocity_profile.cpp"
SRC="$SRC $PROJECT/src/RM/speed_limit.cpp"
echo "compiler: $CXX"
$CXX $FLAGS $SRC -o "$OUT/robot_link"
echo "built: $OUT/robot_link"
