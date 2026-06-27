#!/usr/bin/env bash
# Call-graph pipeline: Doxygen -> trace script -> Graphviz.
# Run from the outscope/ directory. Produces one rendered graph per command.
#
#   ./run_callgraph.sh [format]      format = svg (default) | png | pdf
set -euo pipefail

FORMAT="${1:-svg}"
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

XML_DIR="doxygen_out/xml"
OUT_DIR="callgraph_output"
COMMANDS="tools/command.txt"

command -v doxygen >/dev/null || { echo "[fatal] doxygen not installed"; exit 1; }
command -v dot     >/dev/null || { echo "[fatal] graphviz 'dot' not installed"; exit 1; }
PYTHON="${PYTHON:-python3}"

echo "== Step 1: Doxygen (XML extraction) =="
# Sources are piped through tools/strip_comments.py (INPUT_FILTER in Doxyfile),
# which handles Shift-JIS/CP932 decoding and strips comments before parsing.
doxygen Doxyfile

echo "== Step 2: Trace script (DOT generation) =="
"$PYTHON" tools/callgraph_trace.py --xml "$XML_DIR" --commands "$COMMANDS" --out "$OUT_DIR"

echo "== Step 3: Graphviz render (.$FORMAT) =="
shopt -s nullglob
for dot in "$OUT_DIR"/*.dot; do
    out="${dot%.dot}.$FORMAT"
    if dot -T"$FORMAT" "$dot" -o "$out"; then
        echo "[render] $out"
    else
        echo "[render] FAILED for $dot" >&2
    fi
done

echo "== Done. Graphs in $OUT_DIR/ =="
