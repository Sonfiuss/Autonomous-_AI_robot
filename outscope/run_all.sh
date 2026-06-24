#!/usr/bin/env bash
set -e   # abort on any error

DBM_SRC="com/project/DBM"
FUNCS="functions.txt"
CMD_FILE="testtool/command.txt"
TEST_SRC="testtool/OrbitTestImpl.cpp"
OUT_DIR="output"

echo "[1/2] Generating ${FUNCS} from ${DBM_SRC} ..."
python tools/extract_functions.py "$DBM_SRC" --out "$FUNCS"

# Verify the file exists and is non-empty
if [ ! -s "$FUNCS" ]; then
    echo "ERROR: ${FUNCS} is missing or empty. Aborting."
    exit 1
fi
echo "[1/2] OK — ${FUNCS} ready ($(wc -l < "$FUNCS") functions)."

echo "[2/2] Running call-tree tracer ..."
python tools/main.py \
    --cmd   "$CMD_FILE" \
    --src   "$TEST_SRC" \
    --funcs "$FUNCS"    \
    --dbm   "$DBM_SRC"  \
    --out   "$OUT_DIR"

echo "[2/2] Done. Results in ${OUT_DIR}/"
