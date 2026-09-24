#!/usr/bin/env bash
# Builds the firmware's host-testable parts — the motion state machine and the
# RTOS port — with a plain compiler, no ESP-IDF:
#   tools/build_test_motion.sh   -> build/test_motion
# Everything else in this firmware (drivers, tasks, main) needs the real
# toolchain and is NOT covered here.
set -e
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
export PATH="$(dirname "$CXX"):$PATH"
OUT=build
mkdir -p "$OUT"
# RM lives in the shared project/ tree; the firmware consumes it and nothing else.
PROJECT=../../project
FLAGS="-std=c++14 -O2 -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti -DFW_HOST_BUILD \
       -Iinclude -I$PROJECT/include -I$PROJECT/config"
SRC="src/motion_state.cpp \
     $PROJECT/src/RM/omni_kinematics.cpp $PROJECT/src/RM/velocity_profile.cpp \
     $PROJECT/src/RM/speed_limit.cpp $PROJECT/src/RM/odometry.cpp \
     $PROJECT/src/RM/driver_stepdir.cpp"
echo "compiler: $CXX"
$CXX $FLAGS $SRC tests/test_motion.cpp -o "$OUT/test_motion"
echo "built: $OUT/test_motion"
