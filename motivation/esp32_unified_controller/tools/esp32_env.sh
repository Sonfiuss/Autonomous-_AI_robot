#!/usr/bin/env bash
# Sourced, never executed: puts ESP-IDF and the ESP32's serial port into the
# calling shell so plain idf.py commands work.
#
#   source tools/esp32_env.sh              # auto-detect both
#   ESP_PORT=/dev/ttyUSB1 source tools/esp32_env.sh
#   IDF_PATH=/opt/esp-idf source tools/esp32_env.sh
#
# Exports IDF_PATH and ESP_PORT. Returns non-zero instead of exiting, because
# `set -e` in a sourced file would kill the interactive shell that sourced it.

esp32_env_find_idf() {
    if [ -n "${IDF_PATH:-}" ] && [ -f "$IDF_PATH/export.sh" ]; then
        export IDF_PATH
        return 0
    fi
    local c
    for c in "$HOME/esp/esp-idf" "$HOME/esp-idf" /opt/esp-idf /opt/esp/esp-idf; do
        if [ -f "$c/export.sh" ]; then
            export IDF_PATH="$c"
            return 0
        fi
    done
    echo "esp32_env: no ESP-IDF found." >&2
    echo "esp32_env: looked in \$IDF_PATH, ~/esp/esp-idf, ~/esp-idf, /opt/esp-idf, /opt/esp/esp-idf" >&2
    echo "esp32_env: install it once:  tools/esp32_setup_idf.sh" >&2
    return 1
}

esp32_env_load_idf() {
    esp32_env_find_idf || return 1
    # export.sh takes a couple of seconds and is noisy; skip it if this shell
    # already has the toolchain on PATH from an earlier source.
    if command -v idf.py >/dev/null 2>&1 && [ -n "${IDF_PYTHON_ENV_PATH:-}" ]; then
        return 0
    fi
    # shellcheck disable=SC1091
    . "$IDF_PATH/export.sh" >/dev/null || return 1
    command -v idf.py >/dev/null 2>&1 || {
        echo "esp32_env: sourced $IDF_PATH/export.sh but idf.py is still missing." >&2
        echo "esp32_env: the install step probably did not finish; rerun tools/esp32_setup_idf.sh" >&2
        return 1
    }
}

esp32_env_find_port() {
    if [ -n "${ESP_PORT:-}" ]; then
        if [ -e "$ESP_PORT" ]; then
            export ESP_PORT
            return 0
        fi
        echo "esp32_env: ESP_PORT=$ESP_PORT does not exist." >&2
        return 1
    fi
    local found=()
    local p
    for p in /dev/ttyUSB* /dev/ttyACM*; do
        [ -e "$p" ] && found+=("$p")
    done
    case ${#found[@]} in
        0)
            echo "esp32_env: no /dev/ttyUSB* or /dev/ttyACM* present — is the ESP32 plugged in?" >&2
            return 1
            ;;
        1)
            export ESP_PORT="${found[0]}"
            return 0
            ;;
        *)
            echo "esp32_env: several serial ports present: ${found[*]}" >&2
            echo "esp32_env: pick one with  --port <dev>  or  export ESP_PORT=<dev>" >&2
            return 1
            ;;
    esac
}

# A held port is the most common flash failure on this robot: robot_link and
# stereo_scan both open the same ESP32. Report the holder instead of letting
# esptool fail with a bare permission/busy error.
esp32_env_warn_if_busy() {
    local port="${1:-$ESP_PORT}"
    command -v fuser >/dev/null 2>&1 || return 0
    local holders
    holders=$(fuser "$port" 2>/dev/null) || return 0
    [ -z "$holders" ] && return 0
    echo "esp32_env: WARNING $port is open by PID(s):$holders" >&2
    command -v ps >/dev/null 2>&1 && ps -o pid=,comm= -p ${holders} 2>/dev/null >&2
    echo "esp32_env: stop that process first (robot_link / stereo_scan share this port)." >&2
    return 1
}

esp32_env_load_idf && esp32_env_find_port && {
    echo "IDF_PATH=$IDF_PATH"
    echo "ESP_PORT=$ESP_PORT"
}
