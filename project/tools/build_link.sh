#!/usr/bin/env bash
# Builds the LINK protocol tests without CMake.
#   tools/build_link.sh          -> build/link/test_link
# LINK is the wire protocol shared by the ESP32 firmware and the Jetson app, so
# it depends on nothing but config/constants.h.
# Override the compiler with CXX=...
set -e
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
# MinGW toolchains need their own bin dir first on PATH to find cc1plus and their DLLs.
export PATH="$(dirname "$CXX"):$PATH"
OUT=build/link
mkdir -p "$OUT"
# -fno-exceptions matches the rest of project/: this code runs inside firmware.
FLAGS="-std=c++14 -O2 -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti -Iinclude -Iconfig"
SRC="src/LINK/protocol.cpp"
echo "compiler: $CXX"
$CXX $FLAGS $SRC tests/test_link.cpp -o "$OUT/test_link"
echo "built: $OUT/test_link"
