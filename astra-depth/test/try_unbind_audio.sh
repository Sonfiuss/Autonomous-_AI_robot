#!/usr/bin/env bash
# Astra Pro depth-open experiment: the 2bc5:0403 device also carries a mic whose
# audio interfaces get claimed by snd-usb-audio. That can block the Orbbec driver
# from selecting the USB configuration that exposes the command (bulk-OUT) endpoint.
# This script prints the full descriptor, unbinds snd-usb-audio from the Astra's
# audio interfaces, then runs one clean probe.  Run:  sudo bash try_unbind_audio.sh
set -u
LIB="/home/nvidia/Documents/astra-depth/lib"

echo "== full USB descriptor of 2bc5:0403 (configs + endpoints) =="
lsusb -v -d 2bc5:0403 2>/dev/null | grep -iE "bNumConfigurations|bConfigurationValue|bInterfaceNumber|bAlternateSetting|bNumEndpoints|bEndpointAddress|Transfer Type|bInterfaceClass"

echo
echo "== locate the 2bc5:0403 sysfs device =="
DEV=""
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  if [ "$(cat "$d/idVendor")" = "2bc5" ] && [ "$(cat "$d/idProduct")" = "0403" ]; then DEV="$d"; fi
done
echo "device: $DEV"

echo "== unbind snd-usb-audio from Astra audio interfaces =="
for intf in "$DEV":*; do
  drv=$(basename "$(readlink "$intf/driver" 2>/dev/null)" 2>/dev/null)
  if [ "$drv" = "snd-usb-audio" ]; then
    name=$(basename "$intf")
    echo "  unbinding $name"
    echo -n "$name" > /sys/bus/usb/drivers/snd-usb-audio/unbind 2>/dev/null && echo "   ok" || echo "   failed"
  fi
done

echo "== run one clean probe =="
( cd "$LIB" && LD_LIBRARY_PATH="$LIB" ./astra_probe 12 ./ 2>&1 \
    | grep -iE "found|open|version|NACK|mode|endpoint|frame|valid|saved" )
