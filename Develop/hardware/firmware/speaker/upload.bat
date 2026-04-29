@echo off
setlocal

if "%~1"=="" (
    echo Usage: %~nx0 COMx [-S]
    echo Example: %~nx0 COM3
    echo          %~nx0 COM3 -S    ^(upload + open serial monitor^)
    exit /b 1
)

set "PORT=%~1"
set "SHOW_SERIAL=0"
if /i "%~2"=="-S" set "SHOW_SERIAL=1"

set "PROJECT_DIR=%~dp0"

pushd "%PROJECT_DIR%"
python -m platformio run -e esp32doit-devkit-v1 -t upload --upload-port %PORT%
set "EXIT_CODE=%ERRORLEVEL%"

if "%EXIT_CODE%"=="0" if "%SHOW_SERIAL%"=="1" (
    echo.
    echo [SERIAL] Opening monitor on %PORT% at 115200 baud...
    python -m platformio device monitor --port %PORT% --baud 115200
)

popd

exit /b %EXIT_CODE%
