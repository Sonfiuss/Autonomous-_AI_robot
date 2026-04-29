@echo off
setlocal

set "PROJECT_DIR=%~dp0"

pushd "%PROJECT_DIR%"
python -m platformio run
set "EXIT_CODE=%ERRORLEVEL%"
popd

exit /b %EXIT_CODE%
