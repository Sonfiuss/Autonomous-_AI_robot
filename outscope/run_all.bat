@echo off
setlocal

set DBM_SRC=com\project\DBM
set FUNCS=functions.txt
set CMD_FILE=testtool\command.txt
set TEST_SRC=testtool\OrbitTestImpl.cpp
set OUT_DIR=output

echo [1/2] Generating %FUNCS% from %DBM_SRC% ...
python tools\extract_functions.py %DBM_SRC% --out %FUNCS%
if errorlevel 1 (
    echo ERROR: extract_functions.py failed. Aborting.
    exit /b 1
)

if not exist %FUNCS% (
    echo ERROR: %FUNCS% was not created. Aborting.
    exit /b 1
)

for %%A in (%FUNCS%) do set FUNCS_SIZE=%%~zA
if "%FUNCS_SIZE%"=="0" (
    echo ERROR: %FUNCS% is empty. Aborting.
    exit /b 1
)

echo [1/2] OK — %FUNCS% ready.

echo [2/2] Running call-tree tracer ...
python tools\main.py ^
    --cmd   %CMD_FILE% ^
    --src   %TEST_SRC% ^
    --funcs %FUNCS% ^
    --dbm   %DBM_SRC% ^
    --out   %OUT_DIR%
if errorlevel 1 (
    echo ERROR: main.py failed.
    exit /b 1
)

echo [2/2] Done. Results in %OUT_DIR%\
endlocal
