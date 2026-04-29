@echo off
setlocal

if "%~1"=="" (
    echo Usage: %~nx0 COMx
    echo Example: %~nx0 COM7
    exit /b 1
)

set "PORT=%~1"
set "PROJECT_DIR=%~dp0"

pushd "%PROJECT_DIR%"
python -m platformio run -t upload --upload-port %PORT%
if errorlevel 1 (
    set "EXIT_CODE=%ERRORLEVEL%"
    popd
    exit /b %EXIT_CODE%
)

python -m platformio device monitor --port %PORT% --baud 115200
set "EXIT_CODE=%ERRORLEVEL%"
popd

exit /b %EXIT_CODE%