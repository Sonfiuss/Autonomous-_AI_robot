@echo off
setlocal

if "%~1"=="" (
    echo Usage: %~nx0 COMx
    echo Example: %~nx0 COM5
    exit /b 1
)

set "PORT=%~1"
set "PROJECT_DIR=%~dp0"

pushd "%PROJECT_DIR%"
python -m platformio run -e fan -t upload --upload-port %PORT%
set "EXIT_CODE=%ERRORLEVEL%"
popd

exit /b %EXIT_CODE%
