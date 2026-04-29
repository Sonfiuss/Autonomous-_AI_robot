@echo off
setlocal

if "%~1"=="" (
    echo Usage: %~nx0 COMx
    echo Example: %~nx0 COM7
    exit /b 1
)

set "PORT=%~1"
set "PROJECT_DIR=%~dp0"

echo ================================================
echo  ESP32 Speaker Debug Mode
echo  Env    : debug_speaker (no mic, no PSRAM)
echo  Port   : %PORT%
echo ================================================

pushd "%PROJECT_DIR%"
python -m platformio run -e debug_speaker -t upload --upload-port %PORT%
if %ERRORLEVEL% neq 0 (
    echo.
    echo UPLOAD FAILED
    popd
    exit /b 1
)

echo.
echo Upload OK. Opening serial monitor...
echo Expected: 3 pip tones + serial log
echo Press Ctrl+C to exit monitor.
echo.
python -m platformio device monitor -e debug_speaker --port %PORT% --baud 115200
popd
