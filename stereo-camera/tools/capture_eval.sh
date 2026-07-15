#!/usr/bin/env bash
# capture_eval.sh — tagged stereo capture into an eval/calib session folder
# (Jetson). Same ffmpeg warmup+grab as capture_pair.sh, plus a manifest row so
# depth_eval.py / epipolar_check.py / stereo_calibrate can consume the session.
#
# Usage:
#   ./capture_eval.sh <tag> [z_true_m] [session_dir] [left_idx] [right_idx]
#
#   ./capture_eval.sh d040 0.40          # eval target at tape-measured 0.40 m
#   ./capture_eval.sh d040b 0.40         # repeatability pair, same distance
#   ./capture_eval.sh cb01               # calibration board pose (no distance)
#   SESSION=captures/calib_20260712 ./capture_eval.sh cb02
#
# Distance convention: LEFT camera lens front -> target plane, along the
# optical axis. Scene must be STATIC (cams are unsynced) — prop the board.
#
# Output: <session>/<tag>_left.jpg  <session>/<tag>_right.jpg
#         <session>/manifest.csv   (tag,z_true_m,left,right)

set -euo pipefail

TAG="${1:?usage: capture_eval.sh <tag> [z_true_m] [session_dir] [left_idx] [right_idx]}"
Z_TRUE="${2:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SESSION="${3:-${SESSION:-${SCRIPT_DIR}/captures/eval_$(date +%Y%m%d)}}"
LEFT_IDX="${4:-0}"
RIGHT_IDX="${5:-2}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"

mkdir -p "${SESSION}"
LEFT_DEV="/dev/video${LEFT_IDX}"
RIGHT_DEV="/dev/video${RIGHT_IDX}"
LEFT_OUT="${SESSION}/${TAG}_left.jpg"
RIGHT_OUT="${SESSION}/${TAG}_right.jpg"
MANIFEST="${SESSION}/manifest.csv"

for dev in "${LEFT_DEV}" "${RIGHT_DEV}"; do
    [[ -e "${dev}" ]] || { echo "[capture] ERROR: ${dev} not found"; exit 1; }
done
if [[ -f "${LEFT_OUT}" ]]; then
    echo "[capture] ERROR: tag '${TAG}' already exists in ${SESSION}"
    exit 1
fi

grab() {
    ffmpeg -loglevel error -f v4l2 -input_format mjpeg \
        -video_size "${WIDTH}x${HEIGHT}" -i "$1" -frames:v 1 -y "$2"
}
warmup() {
    ffmpeg -loglevel error -f v4l2 -input_format mjpeg \
        -video_size "${WIDTH}x${HEIGHT}" -i "$1" \
        -frames:v 5 -f null - 2>/dev/null || true
}

echo "[capture] warming up ..."
warmup "${LEFT_DEV}" & W1=$!
warmup "${RIGHT_DEV}" & W2=$!
wait $W1 $W2

echo "[capture] grabbing '${TAG}' (z_true=${Z_TRUE:-n/a}) ..."
grab "${LEFT_DEV}" "${LEFT_OUT}" & P1=$!
grab "${RIGHT_DEV}" "${RIGHT_OUT}" & P2=$!
OK=1
wait $P1 || OK=0
wait $P2 || OK=0
if [[ $OK -ne 1 ]]; then
    echo "[capture] ERROR: grab failed"
    rm -f "${LEFT_OUT}" "${RIGHT_OUT}"
    exit 1
fi

[[ -f "${MANIFEST}" ]] || echo "tag,z_true_m,left,right" > "${MANIFEST}"
echo "${TAG},${Z_TRUE},${TAG}_left.jpg,${TAG}_right.jpg" >> "${MANIFEST}"

echo "[capture] ${LEFT_OUT}"
echo "[capture] ${RIGHT_OUT}"
echo "[capture] manifest: $(grep -c . "${MANIFEST}") lines (incl. header)"
