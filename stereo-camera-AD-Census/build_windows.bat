@echo off
REM ==========================================================================
REM  build_windows.bat  -  Build adcensus_depth on Windows with MinGW g++
REM  (No CMake required. For local testing before deploying to Jetson.)
REM
REM  Two modes:
REM    1) pkg-config  (recommended; works if you installed OpenCV via MSYS2:
REM         pacman -S mingw-w64-x86_64-opencv mingw-w64-x86_64-gcc pkg-config
REM       then run this from the "MSYS2 MinGW 64-bit" shell or with that
REM       toolchain on PATH).
REM    2) manual      (edit OPENCV_INC / OPENCV_LIB / OPENCV_LIBS below to
REM       point at a MinGW-built OpenCV install).
REM
REM  IMPORTANT: official opencv.org Windows builds are MSVC and will NOT link
REM  with MinGW g++. Use MSYS2's OpenCV (mode 1) for MinGW.
REM ==========================================================================
setlocal enabledelayedexpansion
cd /d "%~dp0"

set "CXX=g++"
set "OUT=adcensus_depth.exe"
set "SRCS=main.cpp adcensus\ADCensusStereo.cpp adcensus\adcensus_util.cpp adcensus\cost_computor.cpp adcensus\cross_aggregator.cpp adcensus\scanline_optimizer.cpp adcensus\multistep_refiner.cpp"
set "CXXFLAGS=-std=c++14 -O2 -I. -w"

REM ---- Mode 1: try pkg-config (MSYS2) --------------------------------------
where pkg-config >nul 2>nul
if %errorlevel%==0 (
    for /f "delims=" %%i in ('pkg-config --cflags opencv4 2^>nul') do set "OCV_CFLAGS=%%i"
    for /f "delims=" %%i in ('pkg-config --libs   opencv4 2^>nul') do set "OCV_LIBS=%%i"
    if "!OCV_CFLAGS!"=="" (
        for /f "delims=" %%i in ('pkg-config --cflags opencv 2^>nul') do set "OCV_CFLAGS=%%i"
        for /f "delims=" %%i in ('pkg-config --libs   opencv 2^>nul') do set "OCV_LIBS=%%i"
    )
)

REM ---- Mode 2: manual fallback ---------------------------------------------
REM  EDIT these if you are NOT using MSYS2/pkg-config. Example values shown.
if "!OCV_CFLAGS!"=="" (
    echo [info] pkg-config not found / no OpenCV .pc - using manual paths.
    set "OPENCV_INC=C:\msys64\mingw64\include\opencv4"
    set "OPENCV_LIB=C:\msys64\mingw64\lib"
    set "OPENCV_LIBS=-lopencv_core -lopencv_imgproc -lopencv_imgcodecs"
    set "OCV_CFLAGS=-I!OPENCV_INC!"
    set "OCV_LIBS=-L!OPENCV_LIB! !OPENCV_LIBS!"
)

echo [build] %CXX% %CXXFLAGS% !OCV_CFLAGS!
echo.
%CXX% %CXXFLAGS% !OCV_CFLAGS! %SRCS% -o %OUT% !OCV_LIBS!
if %errorlevel% neq 0 (
    echo.
    echo [FAILED] build error. Check that you have a MinGW-built OpenCV.
    exit /b 1
)

echo.
echo [OK] built %OUT%
echo.
echo Run example:
echo   %OUT% ..\stereo-camera\captures\left_20260610_001156.jpg ..\stereo-camera\captures\right_20260610_001156.jpg adcensus_out 0 128
endlocal
