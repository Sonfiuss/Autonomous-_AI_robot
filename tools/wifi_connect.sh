#!/usr/bin/env bash
# wifi_connect.sh — switch the Jetson's Wi-Fi (wlan0) to another network via NetworkManager.
#
# Usage:
#   sudo ./wifi_connect.sh                     # connect to the default network below
#   sudo ./wifi_connect.sh <SSID> <PASSWORD>   # connect to another network
#   sudo ./wifi_connect.sh --status            # show current Wi-Fi state only
#
# Safe over SSH: the switch runs detached (setsid + nohup), so it finishes even
# when the SSH session drops with the old network. If the new network gives no
# internet within TIMEOUT seconds, it rolls back to the previous connection.
# Log: /var/log/wifi_connect.log

set -u

DEFAULT_SSID="HKHOA"
DEFAULT_PASS="htcncdms"
IFACE="wlan0"
TIMEOUT=30                 # seconds to wait for the new link
PING_HOST="8.8.8.8"        # internet check target
PRIORITY=10                # autoconnect priority for the new network (higher wins at boot)
LOG_FILE="/var/log/wifi_connect.log"

log() { echo "[$(date '+%F %T')] $*" | tee -a "$LOG_FILE"; }

show_status() {
    nmcli -t -f DEVICE,STATE,CONNECTION device status | grep "^${IFACE}:"
    ip -4 -o addr show "$IFACE" | awk '{print "IP: " $4}'
}

if [[ "${1:-}" == "--status" ]]; then
    show_status
    exit 0
fi

if [[ $EUID -ne 0 ]]; then
    exec sudo "$0" "$@"
fi

SSID="${1:-$DEFAULT_SSID}"
PASS="${2:-$DEFAULT_PASS}"

if (( ${#PASS} < 8 )); then
    echo "Error: WPA password must be at least 8 characters." >&2
    exit 1
fi

# Re-launch detached so an SSH disconnect (SIGHUP) cannot kill us mid-switch.
if [[ -z "${WIFI_CONNECT_DETACHED:-}" ]]; then
    WIFI_CONNECT_DETACHED=1 setsid nohup "$0" "$SSID" "$PASS" >/dev/null 2>&1 < /dev/null &
    echo "Switching $IFACE to '$SSID' in the background."
    echo "If you are on SSH over Wi-Fi, this session will drop; the Jetson gets a new IP on '$SSID'."
    echo "Log: $LOG_FILE   (check later with: $0 --status)"
    exit 0
fi

# ---- detached part ----------------------------------------------------------
PREV_CONN="$(nmcli -t -f DEVICE,CONNECTION device status | awk -F: -v d="$IFACE" '$1==d {print $2}')"
log "start: $IFACE currently on '${PREV_CONN:-none}', target '$SSID'"

if [[ "$PREV_CONN" == "$SSID" ]]; then
    log "already connected to '$SSID', nothing to do"
    exit 0
fi

nmcli device wifi rescan ifname "$IFACE" >/dev/null 2>&1
sleep 3
if ! nmcli -t -f SSID device wifi list ifname "$IFACE" | grep -Fxq "$SSID"; then
    log "ERROR: SSID '$SSID' not visible in scan — staying on '${PREV_CONN:-none}'"
    exit 1
fi

# Create or update the saved profile so the password is always current.
if nmcli -t -f NAME connection show | grep -Fxq "$SSID"; then
    log "updating saved profile '$SSID'"
    nmcli connection modify "$SSID" \
        wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASS" \
        connection.autoconnect yes connection.autoconnect-priority "$PRIORITY"
else
    log "creating profile '$SSID'"
    nmcli connection add type wifi ifname "$IFACE" con-name "$SSID" ssid "$SSID" \
        wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PASS" \
        connection.autoconnect yes connection.autoconnect-priority "$PRIORITY"
fi

if nmcli --wait "$TIMEOUT" connection up "$SSID" ifname "$IFACE" >>"$LOG_FILE" 2>&1 \
   && ping -c 2 -W 3 -I "$IFACE" "$PING_HOST" >/dev/null 2>&1; then
    NEW_IP="$(ip -4 -o addr show "$IFACE" | awk '{print $4}')"
    log "OK: connected to '$SSID', IP $NEW_IP, internet reachable"
    exit 0
fi

log "ERROR: '$SSID' failed (wrong password / no internet)"
if [[ -n "$PREV_CONN" ]]; then
    log "rolling back to '$PREV_CONN'"
    nmcli --wait "$TIMEOUT" connection up "$PREV_CONN" ifname "$IFACE" >>"$LOG_FILE" 2>&1 \
        && log "rollback OK" || log "rollback FAILED"
fi
exit 1
