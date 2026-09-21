#!/usr/bin/env bash
# Builds the MV shared library, CLI and tests without CMake.
#   tools/build_mv.sh            -> build/mv/{mv.dll|libmv.so, mv_cli, test_mv}
# Override the compiler with CXX=... (must match the Python bitness for ctypes:
# on this PC C:/msys64/mingw64/bin/g++ is 64-bit, C:/MinGW/bin/g++ is 32-bit).
set -e
cd "$(dirname "$0")/.."
CXX="${CXX:-g++}"
# MinGW toolchains need their own bin dir first on PATH to find cc1plus and their DLLs.
export PATH="$(dirname "$CXX"):$PATH"
OUT=build/mv
mkdir -p "$OUT"
FLAGS="-std=c++14 -O2 -Wall -Wextra -Wpedantic -fno-exceptions -fno-rtti -Iinclude -Iconfig"
SRC="src/MV/occupancy_grid.cpp src/MV/astar.cpp src/MV/path.cpp src/MV/mv_api.cpp"
case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*) LIB="$OUT/mv.dll"; SHFLAGS="-shared -static-libgcc -static-libstdc++ -DMV_BUILD_SHARED" ;;
    *)                    LIB="$OUT/libmv.so"; SHFLAGS="-shared -fPIC" ;;
esac
echo "compiler: $CXX"
$CXX $FLAGS $SHFLAGS $SRC -o "$LIB"
$CXX $FLAGS $SRC tools/mv_cli.cpp -o "$OUT/mv_cli"
$CXX $FLAGS $SRC tests/test_mv.cpp -o "$OUT/test_mv"
echo "built: $LIB $OUT/mv_cli $OUT/test_mv"
