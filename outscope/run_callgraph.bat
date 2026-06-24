@echo off
REM Call-graph pipeline: Doxygen -> trace script -> Graphviz (Windows).
REM Usage:  run_callgraph.bat [svg|png|pdf]
setlocal enabledelayedexpansion
cd /d "%~dp0"

set "FORMAT=%~1"
if "%FORMAT%"=="" set "FORMAT=svg"
set "XML_DIR=doxygen_out\xml"
set "OUT_DIR=callgraph_output"
set "COMMANDS=testtool\command.txt"
if "%PYTHON%"=="" set "PYTHON=python"

where doxygen >nul 2>&1 || (echo [fatal] doxygen not installed & exit /b 1)
where dot     >nul 2>&1 || (echo [fatal] graphviz 'dot' not installed & exit /b 1)

echo == Step 1: Doxygen (XML extraction) ==
doxygen Doxyfile || exit /b 1

echo == Step 2: Trace script (DOT generation) ==
"%PYTHON%" tools\callgraph_trace.py --xml "%XML_DIR%" --commands "%COMMANDS%" --out "%OUT_DIR%" || exit /b 1

echo == Step 3: Graphviz render (.%FORMAT%) ==
for %%F in ("%OUT_DIR%\*.dot") do (
    dot -T%FORMAT% "%%F" -o "%%~dpnF.%FORMAT%" && echo [render] %%~dpnF.%FORMAT%
)

echo == Done. Graphs in %OUT_DIR%\ ==
endlocal
