#!/usr/bin/env bash
# Trigger an Astra depth-open attempt and dump the kernel USB messages it produced.
# Run:  sudo bash diag_open.sh
set -u
LIB="/home/nvidia/Documents/astra-depth/lib"
echo "== clearing kernel ring buffer marker =="
dmesg -C 2>/dev/null || sudo dmesg -C
echo "== running probe (10s) =="
( cd "$LIB" && timeout 10 env LD_LIBRARY_PATH="$LIB" ./astra_probe 8 ./ 2>&1 | head -8 )
echo "== kernel USB messages during the attempt =="
dmesg | grep -iE "usb|2bc5|reset|over-?current|power budget|error -[0-9]+|not accepting|disconnect|xhci" | tail -30
