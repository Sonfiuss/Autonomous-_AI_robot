# Task: wifi-switch
Status: testing
Module: tools (system utility, no module)

## Task
Script to switch the Jetson's Wi-Fi to network "Sonsss".

## Input
SSID/password (default Sonsss / 23344556, overridable as args).

## Expected output
wlan0 connected to Sonsss with internet; rollback to previous network on failure.

## Plan
1. Check NetworkManager / wlan0 / existing profiles.
2. Write tools/wifi_connect.sh (nmcli, detached so SSH drop can't kill it, ping check, rollback).
3. Syntax check + --status run (do NOT switch live: agent session is SSH over wlan0).

## Execution log
[--:--] step 1: nmcli present; wlan0 on "KIM THANH" 192.168.1.90; "Sonsss" profile already saved, visible (signal 34) | risk: SSH session 192.168.1.75→.90 runs over wlan0 | info: nmcli modify/control needs sudo (password)
[--:--] step 2: tools/wifi_connect.sh written; sets autoconnect-priority 10 so Sonsss wins at boot
[--:--] step 3: bash -n OK, --status OK; live switch left to user

## Test result
(pending) run: sudo /home/nvidia/Documents/tools/wifi_connect.sh ; then check /var/log/wifi_connect.log
