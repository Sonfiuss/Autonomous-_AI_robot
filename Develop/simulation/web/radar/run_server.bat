@echo off
setlocal

set "PROJECT_DIR=%~dp0"
pushd "%PROJECT_DIR%"

echo Installing dependencies...
python -m pip install -r requirements.txt -q

echo.
echo Starting Radar Visualization Server...
echo Open http://localhost:5000 in your browser
echo.

python server.py

popd
