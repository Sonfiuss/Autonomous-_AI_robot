#!/usr/bin/env bash
# lock_exposure.sh — capture hygiene probe (Jetson, Stage D3).
#
# The two USB cameras run auto-exposure/auto-WB independently, so the L/R pair
# can differ in brightness (NCC matching degrades) and exposure can drift
# between the golden-point match and the DA-V2 frame. This script tries to
# lock manual exposure/gain/WB on BOTH cameras and REPORTS what actually
# stuck — UVC cameras often silently ignore controls, so every set is re-read.
#
# Usage:
#   ./lock_exposure.sh                 # probe + lock both cams with defaults
#   ./lock_exposure.sh 0 2 200         # left idx, right idx, exposure value
#   ./lock_exposure.sh --list          # just list supported controls
#
# After locking, verify the effect with a fresh pair:
#   ./capture_eval.sh explock 0        # then epipolar_check.py brightness cols

set -uo pipefail

if [[ "${1:-}" == "--list" ]]; then
    for d in /dev/video0 /dev/video2; do
        echo "=== ${d} ==="
        v4l2-ctl -d "${d}" --list-ctrls || true
    done
    exit 0
fi

LEFT_IDX="${1:-0}"
RIGHT_IDX="${2:-2}"
EXPOSURE="${3:-166}"     # ~1/60s in 100us units on most UVC cams
GAIN="${GAIN:-32}"

try_set() {
    local dev="$1" ctrl="$2" val="$3"
    if ! v4l2-ctl -d "${dev}" --list-ctrls 2>/dev/null | grep -q "${ctrl}"; then
        echo "    ${ctrl}: not supported"
        return
    fi
    v4l2-ctl -d "${dev}" --set-ctrl "${ctrl}=${val}" 2>/dev/null
    local got
    got=$(v4l2-ctl -d "${dev}" --get-ctrl "${ctrl}" 2>/dev/null | awk '{print $2}')
    if [[ "${got}" == "${val}" ]]; then
        echo "    ${ctrl}=${got}  OK"
    else
        echo "    ${ctrl}: asked ${val}, got '${got}'  (IGNORED by camera)"
    fi
}

for idx in "${LEFT_IDX}" "${RIGHT_IDX}"; do
    dev="/dev/video${idx}"
    echo "[lock] ${dev}"
    # manual exposure: UVC names vary across kernel versions
    try_set "${dev}" auto_exposure 1                  # 1 = manual on new kernels
    try_set "${dev}" exposure_auto 1                  # legacy name
    try_set "${dev}" exposure_time_absolute "${EXPOSURE}"
    try_set "${dev}" exposure_absolute "${EXPOSURE}"  # legacy name
    try_set "${dev}" gain "${GAIN}"
    try_set "${dev}" white_balance_automatic 0
    try_set "${dev}" white_balance_temperature_auto 0 # legacy name
    try_set "${dev}" white_balance_temperature 4600
done

echo
echo "[lock] verify with: ./capture_eval.sh explock_test 0"
echo "[lock]   then compare bright_delta in epipolar_check.py before/after."
echo "[lock] NOTE: settings reset on unplug/reboot — rerun before a session."
