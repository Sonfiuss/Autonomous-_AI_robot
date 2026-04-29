#!/usr/bin/env bash

set -e

 

echo "=========================================="

echo "DIAGNOSTIC CHECK FOR BUILD ERRORS"

echo "=========================================="

 

# Get paths

SHELL_DIR="$(cd "$(dirname "$0")" && pwd)"

VISION_DIR="$(cd "${SHELL_DIR}/.." && pwd)"

SRC_DIR="${VISION_DIR}/src"

SRC_FILE="${SRC_DIR}/main.cpp"

 

echo ""

echo "[1] Checking source file..."

if [ ! -f "${SRC_FILE}" ]; then

  echo "ERROR: ${SRC_FILE} not found"

  exit 1

fi

echo "OK: ${SRC_FILE} exists"

echo "Size: $(wc -c < ${SRC_FILE}) bytes"

if [ ! -s "${SRC_FILE}" ]; then

  echo "ERROR: ${SRC_FILE} is empty!"

  exit 1

fi

echo "OK: File is not empty"

 

echo ""

echo "[2] Checking g++ compiler..."

if ! command -v g++ >/dev/null 2>&1; then

  echo "ERROR: g++ not found in PATH"

  exit 1

fi

echo "OK: g++ found at: $(which g++)"

g++ --version | head -1

 

echo ""

echo "[3] Checking OpenCV..."

if ! pkg-config --exists opencv4; then

  echo "ERROR: opencv4 not found"

  echo "Install with: sudo apt-get install libopencv-dev"

  exit 1

fi

echo "OK: opencv4 found"

echo "Flags: $(pkg-config --cflags --libs opencv4)"

 

echo ""

echo "[4] Checking ONNX Runtime..."

ONNXRUNTIME_DIR="${ONNXRUNTIME_DIR:-$HOME/onnxruntime-linux-x64-1.25.1}"

echo "ONNXRUNTIME_DIR: ${ONNXRUNTIME_DIR}"

if [ ! -d "${ONNXRUNTIME_DIR}" ]; then

  echo "ERROR: ONNXRUNTIME_DIR directory not found"

  echo "Download from: https://github.com/microsoft/onnxruntime/releases"

  exit 1

fi

echo "OK: ONNXRUNTIME_DIR exists"

 

ONNX_INC="${ONNXRUNTIME_DIR}/include"

ONNX_LIB="${ONNXRUNTIME_DIR}/lib"

if [ ! -f "${ONNX_INC}/onnxruntime_cxx_api.h" ]; then

  echo "ERROR: onnxruntime_cxx_api.h not found at ${ONNX_INC}/"

  echo "Available files:"

  ls -la "${ONNX_INC}/" 2>/dev/null || echo "Directory not accessible"

  exit 1

fi

echo "OK: onnxruntime_cxx_api.h found"

 

if [ ! -f "${ONNX_LIB}/libonnxruntime.so" ] && [ ! -f "${ONNX_LIB}/libonnxruntime.so.1" ]; then

  echo "ERROR: libonnxruntime.so* not found at ${ONNX_LIB}/"

  echo "Available files:"

  ls -la "${ONNX_LIB}/" 2>/dev/null || echo "Directory not accessible"

  exit 1

fi

echo "OK: libonnxruntime.so found"

ls -la "${ONNX_LIB}"/libonnxruntime.so*

 

echo ""

echo "[5] Checking main.cpp syntax..."

if ! g++ -c "${SRC_FILE}" -I"${ONNX_INC}" $(pkg-config --cflags opencv4) -std=c++17 -o /tmp/test.o 2>&1; then

  echo "ERROR: Compilation failed (syntax error)"

  echo "Running again for details:"

  g++ -c "${SRC_FILE}" -I"${ONNX_INC}" $(pkg-config --cflags opencv4) -std=c++17 -o /tmp/test.o

  exit 1

fi

echo "OK: Syntax check passed"

rm /tmp/test.o

 

echo ""

echo "=========================================="

echo "ALL CHECKS PASSED"

echo "=========================================="

echo ""

echo "You can now run: bash ${SHELL_DIR}/Main.sh"
