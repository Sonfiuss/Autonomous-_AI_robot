#!/usr/bin/env bash

set -e
# ===============
# Configure the environment
# ===============

PROJECT_NAME="vision"

SCRIPT_DIR="$(cd "$( dirname "${0}" )" && pwd )"
VISION_DIR="$(cd "$SCRIPT_DIR/.." && pwd )"
src="$VISION_DIR/src"
SRC_FILE="$VISION_DIR/src/main.cpp"

[ -f "${SRC_FILE}" ]||
{
    echo "Error: main.cpp not found in $VISION_DIR/src"
    exit 1
}

OUT_FILE="$VISION_DIR/build/deepanything.exe"

# Allow overriding ONNX Runtime location from environment

ONNXRUNTIME_DIR="${ONNXRUNTIME_DIR:-$HOME/onnxruntime-linux-x64-1.25.1}"
ONNX_INC="${ONNXRUNTIME_DIR}/include"
ONNX_LIB="${ONNXRUNTIME_DIR}/lib"

# echo "== Checking environment variables =="
# echo "PROJECT_NAME: $PROJECT_NAME"
# echo "SCRIPT_DIR: $SCRIPT_DIR"
# echo "VISION_DIR: $VISION_DIR"
# echo "ONNXRUNTIME_DIR: $ONNXRUNTIME_DIR"

command -v g++ >/dev/null 2>&1 ||
{ 
     echo "Error: g++ is not installed. Please install g++ and try again."; 
     exit 1; 
}

pkg-config --cflags opencv4 ||
{ 
     echo "Error: OpenCV is not installed or pkg-config cannot find it. Please install OpenCV and ensure pkg-config is configured correctly."; 
     exit 1; 
}
[ -f "$ONNX_INC/onnxruntime_cxx_api.h" ]||
{ 
     echo "Error: ONNX Runtime headers not found in $ONNX_INC. Please check your ONNXRUNTIME_DIR environment variable."; 
     exit 1; 
}
[ -f "$ONNX_LIB/libonnxruntime.so" ] ||
{ 
     echo "Error: ONNX Runtime library not found in $ONNX_LIB. Please check your ONNXRUNTIME_DIR environment variable."; 
     exit 1; 
}

echo "ONNX_INC: $ONNX_INC"

echo "Environment check passed. All required tools and libraries are available."

echo "== Building the project =="
OPENCV_CFLAGS=$(pkg-config --cflags opencv4)

OPENCV_LIBS=$(pkg-config --libs opencv4)
g++ -std=c++17 -O2 \
  ${OPENCV_CFLAGS} \
  -I"${ONNX_INC}" \
  "${SRC_FILE}" \
  ${OPENCV_LIBS} \
  "${ONNX_LIB}/libonnxruntime.so" \
  -Wl,-rpath,"${ONNX_LIB}" \
  -lpthread \
  -o "${OUT_FILE}"

set +x
echo "== Build SUCCESS =="
# echo "Build completed successfully. Executable created at $OUT_FILE"
export LD_LIBRARY_PATH="$ONNX_LIB:$LD_LIBRARY_PATH"
echo
echo "==run with=="
# echo " ${OUT_FILE}"
