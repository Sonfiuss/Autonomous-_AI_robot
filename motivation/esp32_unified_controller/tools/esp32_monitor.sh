#!/usr/bin/env bash
# Attach to the ESP32's serial console without rebuilding or reflashing.
#
#   tools/esp32_monitor.sh
#   tools/esp32_monitor.sh -p /dev/ttyUSB1
#
# Typed lines go to the firmware, so this doubles as a hand-driven protocol
# console: `R`, `F 0.3 0.15`, `T 90 0.8`, `S`. Ctrl-] exits.
set -e
cd "$(dirname "$0")/.."

case "${1:-}" in
    -p|--port) ESP_PORT="$2" ;;
    "")        ;;
    *)         echo "usage: $(basename "$0") [-p /dev/ttyUSBn]" >&2; exit 2 ;;
esac

# shellcheck disable=SC1091
. tools/esp32_env.sh >/dev/null || { echo "esp32_monitor: environment not ready (see above)." >&2; exit 1; }

esp32_env_warn_if_busy "$ESP_PORT" || exit 1

echo "== monitor on $ESP_PORT (Ctrl-] to exit)"
exec idf.py -p "$ESP_PORT" monitor
