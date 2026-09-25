#!/usr/bin/env bash
# Build, flash and watch the unified controller in one command.
#
#   tools/esp32_flash.sh                 build + flash + monitor
#   tools/esp32_flash.sh --no-monitor    build + flash, then exit
#   tools/esp32_flash.sh --build-only    no hardware needed
#   tools/esp32_flash.sh -p /dev/ttyUSB1
#   tools/esp32_flash.sh --erase         erase the whole flash first
#   tools/esp32_flash.sh --clean         idf.py fullclean before building
#
# Ctrl-] leaves the monitor. Needs ESP-IDF; run tools/esp32_setup_idf.sh once.
set -e
cd "$(dirname "$0")/.."

PORT="${ESP_PORT:-}"
BAUD="${ESP_BAUD:-460800}"
MONITOR=1
FLASH=1
ERASE=0
CLEAN=0

while [ $# -gt 0 ]; do
    case "$1" in
        -p|--port)     PORT="$2"; shift 2 ;;
        -b|--baud)     BAUD="$2"; shift 2 ;;
        --no-monitor)  MONITOR=0; shift ;;
        --build-only)  FLASH=0; MONITOR=0; shift ;;
        --erase)       ERASE=1; shift ;;
        --clean)       CLEAN=1; shift ;;
        -h|--help)     sed -n '2,11p' "$0"; exit 0 ;;
        *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
done

set +e
ESP_PORT="$PORT"
# shellcheck disable=SC1091
. tools/esp32_env.sh >/dev/null
rc=$?
set -e
if [ "$FLASH" -eq 1 ] && [ $rc -ne 0 ]; then
    echo "esp32_flash: environment not ready (see above)." >&2
    exit 1
fi
if [ $rc -ne 0 ] && ! command -v idf.py >/dev/null 2>&1; then
    echo "esp32_flash: ESP-IDF missing, cannot build." >&2
    exit 1
fi
PORT="${ESP_PORT}"

# set-target writes sdkconfig and must happen before the first build. It also
# wipes sdkconfig, so it runs only when there is none to wipe.
if [ ! -f sdkconfig ]; then
    echo "== idf.py set-target esp32 (first build)"
    idf.py set-target esp32
fi

[ "$CLEAN" -eq 1 ] && { echo "== idf.py fullclean"; idf.py fullclean; }

echo "== idf.py build"
idf.py build

if [ "$FLASH" -eq 0 ]; then
    echo "built only — binary at build/esp32_unified_controller.bin"
    exit 0
fi

esp32_env_warn_if_busy "$PORT" || exit 1

if [ "$ERASE" -eq 1 ]; then
    echo "== idf.py erase-flash on $PORT"
    idf.py -p "$PORT" -b "$BAUD" erase-flash
fi

echo "== idf.py flash on $PORT at $BAUD"
if ! idf.py -p "$PORT" -b "$BAUD" flash; then
    echo >&2
    echo "esp32_flash: flash failed. On a CH340 board without auto-reset, hold BOOT," >&2
    echo "esp32_flash: tap EN, release BOOT, then rerun. If it still fails try -b 115200." >&2
    exit 1
fi

if [ "$MONITOR" -eq 1 ]; then
    echo "== monitor on $PORT (Ctrl-] to exit)"
    idf.py -p "$PORT" monitor
fi
