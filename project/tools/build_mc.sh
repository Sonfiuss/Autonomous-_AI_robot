#!/usr/bin/env bash
# Builds the MC shared library, CLI and tests without CMake.
#   tools/build_mc.sh            -> build/mc/{mc.dll|libmc.so, mc_cli, test_mc}
# MC is the motion executor and links the RM math sources with it.
# Override the compiler with CXX=... (must match the Python bitness for ctypes:
# on this PC C:/msys64/mingw64/bin/g++ is 64-bit, C:/MinGW/bin/g++ is 32-bit).
set -e
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
# MinGW toolchains need their own bin dir first on PATH to find cc1plus and their DLLs.
export PATH="$(dirname "$CXX"):$PATH"
OUT=build/mc
mkdir -p "$OUT"
FLAGS="-std=c++14 -O2 -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti -Iinclude -Iconfig"
SRC="src/MC/executor.cpp src/MC/mc_api.cpp \
     src/RM/omni_kinematics.cpp src/RM/velocity_profile.cpp src/RM/speed_limit.cpp \
     src/RM/odometry.cpp src/RM/driver_stepdir.cpp"
case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) LIB="$OUT/mc.dll"; SHFLAGS="-shared -static-libgcc -static-libstdc++ -DMC_BUILD_SHARED" ;;
    *)                    LIB="$OUT/libmc.so"; SHFLAGS="-shared -fPIC" ;;
esac
# The end-to-end test plans with MV first, so it links the MV sources too.
MV_SRC="src/MV/occupancy_grid.cpp src/MV/astar.cpp src/MV/path.cpp src/MV/mv_api.cpp"
echo "compiler: $CXX"
$CXX $FLAGS $SHFLAGS $SRC -o "$LIB"
$CXX $FLAGS $SRC tools/mc_cli.cpp -o "$OUT/mc_cli"
$CXX $FLAGS $SRC $MV_SRC tests/test_mc.cpp -o "$OUT/test_mc"
echo "built: $LIB $OUT/mc_cli $OUT/test_mc"
