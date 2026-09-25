#!/usr/bin/env bash
# One-time ESP-IDF install for this firmware. Run it once, then use
# tools/esp32_flash.sh for everything else.
#
#   tools/esp32_setup_idf.sh            install into ~/esp/esp-idf
#   tools/esp32_setup_idf.sh --apt      also apt-get the system prerequisites
#   IDF_BRANCH=release/v5.3 tools/esp32_setup_idf.sh
#   IDF_DEST=/opt/esp-idf tools/esp32_setup_idf.sh
#
# Downloads roughly 2-3 GB (shallow clone plus the xtensa toolchain). The board
# is a plain ESP32, so only that target's toolchain is installed.
set -e

IDF_BRANCH="${IDF_BRANCH:-release/v5.1}"
IDF_DEST="${IDF_DEST:-$HOME/esp/esp-idf}"
DO_APT=0
[ "${1:-}" = "--apt" ] && DO_APT=1

PKGS="git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0"

missing=""
for p in $PKGS; do
    dpkg -s "$p" >/dev/null 2>&1 || missing="$missing $p"
done

if [ -n "$missing" ]; then
    if [ "$DO_APT" -eq 1 ]; then
        echo "== apt-get install$missing"
        sudo apt-get update
        # shellcheck disable=SC2086
        sudo apt-get install -y $missing
    else
        echo "Missing system packages:$missing"
        echo
        echo "Install them first:"
        echo "  sudo apt-get install -y$missing"
        echo
        echo "or rerun this script as:  $0 --apt"
        exit 1
    fi
fi

if [ -d "$IDF_DEST/.git" ]; then
    echo "== ESP-IDF already cloned at $IDF_DEST (leaving it alone)"
else
    echo "== cloning ESP-IDF $IDF_BRANCH into $IDF_DEST"
    mkdir -p "$(dirname "$IDF_DEST")"
    git clone -b "$IDF_BRANCH" --depth 1 --recursive --shallow-submodules \
        https://github.com/espressif/esp-idf.git "$IDF_DEST"
fi

echo "== installing the esp32 toolchain"
cd "$IDF_DEST"
./install.sh esp32

cat <<EOF

ESP-IDF is installed at $IDF_DEST

Next:
  cd $(cd "$(dirname "$0")/.." && pwd)
  tools/esp32_flash.sh --build-only     # check it compiles before touching the robot

tools/esp32_flash.sh finds this install by itself; you do not need to source
export.sh by hand.
EOF
