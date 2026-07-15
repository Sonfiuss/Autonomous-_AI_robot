#!/usr/bin/env bash
# capture_pair.sh — grab one frame from each stereo camera simultaneously.
#
# Default: left=/dev/video0  right=/dev/video2  1280x720 MJPG
# Output:  stereo-camera/tools/captures/left.jpg
#          stereo-camera/tools/captures/righ.jpg
#
# Usage:
#   ./capture_pair.sh                          # defaults
#   ./capture_pair.sh 0 2                      # explicit left/right device index
#   ./capture_pair.sh 0 2 640 480              # override resolution

set -euo pipefail

# ── configurable ──────────────────────────────────────────────────────────────
LEFT_IDX="${1:-0}"
RIGHT_IDX="${2:-2}"
WIDTH="${3:-1280}"
HEIGHT="${4:-720}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${OUT_DIR:-${SCRIPT_DIR}/captures}"
mkdir -p "${OUT_DIR}"

LEFT_DEV="/dev/video${LEFT_IDX}"
RIGHT_DEV="/dev/video${RIGHT_IDX}"
LEFT_OUT="${OUT_DIR}/left.jpg"
RIGHT_OUT="${OUT_DIR}/righ.jpg"

# ── sanity check ──────────────────────────────────────────────────────────────
missing=()
[[ -e "${LEFT_DEV}"  ]] || missing+=("${LEFT_DEV}")
[[ -e "${RIGHT_DEV}" ]] || missing+=("${RIGHT_DEV}")
if [[ ${#missing[@]} -gt 0 ]]; then
    echo "[capture] ERROR: device(s) not found: ${missing[*]}"
    echo "[capture] Present video devices: $(ls /dev/video* 2>/dev/null | tr '\n' ' ' || echo '(none)')"
    echo "[capture] Run 'v4l2-ctl --list-devices' for details."
    echo "[capture] Usage: $0 <left_idx> <right_idx>"
    exit 1
fi

# ── grab one frame via ffmpeg ─────────────────────────────────────────────────
grab() {
    local dev="$1" out="$2"
    ffmpeg -loglevel error \
        -f v4l2 -input_format mjpeg \
        -video_size "${WIDTH}x${HEIGHT}" \
        -i "${dev}" \
        -frames:v 1 -y "${out}"
}

# ── warm up both cameras first (open/read/close) ──────────────────────────────
# Single-frame grab on first open can return a stale or dark frame on MJPG
# cameras; run a short discard pass on each device in parallel.
echo "[capture] Warming up cameras ..."
warmup() {
    local dev="$1"
    # grab 5 frames, keep none — just drives exposure/AWB to settle
    ffmpeg -loglevel error \
        -f v4l2 -input_format mjpeg \
        -video_size "${WIDTH}x${HEIGHT}" \
        -i "${dev}" \
        -frames:v 5 -f null - 2>/dev/null || true
}
warmup "${LEFT_DEV}"  &
WARM_L=$!
warmup "${RIGHT_DEV}" &
WARM_R=$!
wait $WARM_L
wait $WARM_R

# ── capture both cameras simultaneously ───────────────────────────────────────
# Grab to temp names and promote ONLY if BOTH succeed: a partial failure must
# never leave a mismatched left/right pair on disk (on 2026-07-05 a failed left
# grab left left.jpg 8 days older than righ.jpg — every later offline test on
# that "pair" computed stereo on two different moments).
echo "[capture] Grabbing frame from ${LEFT_DEV} and ${RIGHT_DEV} at ${WIDTH}x${HEIGHT} ..."
grab "${LEFT_DEV}"  "${LEFT_OUT}.tmp"  &
PID_L=$!
grab "${RIGHT_DEV}" "${RIGHT_OUT}.tmp" &
PID_R=$!

DONE_L=0; DONE_R=0
wait $PID_L && DONE_L=1 || true
wait $PID_R && DONE_R=1 || true

# ── report ────────────────────────────────────────────────────────────────────
OK=1
[[ $DONE_L -eq 1 ]] || { echo "[capture] ERROR: left  (${LEFT_DEV}) failed"; OK=0; }
[[ $DONE_R -eq 1 ]] || { echo "[capture] ERROR: right (${RIGHT_DEV}) failed"; OK=0; }
if [[ $OK -eq 1 ]]; then
    mv "${LEFT_OUT}.tmp"  "${LEFT_OUT}"
    mv "${RIGHT_OUT}.tmp" "${RIGHT_OUT}"
    echo "[capture] left  -> ${LEFT_OUT}"
    echo "[capture] right -> ${RIGHT_OUT}"
else
    rm -f "${LEFT_OUT}.tmp" "${RIGHT_OUT}.tmp"
    echo "[capture] partial failure — no files written (old pair untouched)"
fi

if [[ $OK -eq 1 ]]; then
    echo "[capture] Done. Run stereo pipeline:"
    echo "  python3 depth-anything/src/stereo_ruler.py --left ${LEFT_OUT} --right ${RIGHT_OUT}"
fi
exit $((1 - OK))
