#!/bin/bash
# ==============================================================================
# Autonomous RC Car - Wi-Fi Hotspot / Access Point Manager
# Enables direct standalone vehicle control anywhere without an external router.
# ==============================================================================

set -e

HOTSPOT_NAME="RCCar-Hotspot"
SSID="RCCar-WiFi"
PASSWORD="rccar12345"

ACTION="${1:-status}"

echo "=== RC CAR WI-FI & RANGE MANAGER ==="

check_networkmanager() {
  command -v nmcli &>/dev/null
}

show_status() {
  echo "[*] Current Wi-Fi Status:"
  if check_networkmanager; then
    nmcli -t -f DEVICE,TYPE,STATE,CONNECTION dev | grep "wlan0" || true
    echo ""
    echo "[*] Active IP addresses:"
    ip -4 addr show wlan0 2>/dev/null | grep -oP '(?<=inet\s)\d+(\.\d+){3}' || echo "No IP assigned on wlan0"
  else
    echo "NetworkManager (nmcli) not detected."
  fi
}

enable_hotspot() {
  echo "[*] Configuring Wi-Fi Hotspot (SSID: $SSID, Pass: $PASSWORD)..."
  if check_networkmanager; then
    # Delete old hotspot profile if exists
    sudo nmcli connection delete "$HOTSPOT_NAME" 2>/dev/null || true

    # Create new Access Point profile on wlan0
    sudo nmcli connection add \
      type wifi \
      ifname wlan0 \
      con-name "$HOTSPOT_NAME" \
      autoconnect no \
      ssid "$SSID" \
      802-11-wireless.mode ap \
      802-11-wireless.band bg \
      ipv4.method shared \
      wifi-sec.key-mgmt wpa-psk \
      wifi-sec.psk "$PASSWORD"

    echo "[*] Starting Hotspot AP..."
    sudo nmcli connection up "$HOTSPOT_NAME"
    echo ""
    echo "==========================================================="
    echo " [OK] Hotspot Activated!"
    echo "  1. Connect your phone/laptop Wi-Fi to: $SSID"
    echo "  2. Password: $PASSWORD"
    echo "  3. Open Dashboard at: http://10.42.0.1:5000"
    echo "==========================================================="
  else
    echo "[!] NetworkManager is required. Installing..."
    sudo apt-get update && sudo apt-get install -y network-manager
    enable_hotspot
  fi
}

disable_hotspot() {
  echo "[*] Disabling Hotspot and restoring client Wi-Fi..."
  if check_networkmanager; then
    sudo nmcli connection down "$HOTSPOT_NAME" 2>/dev/null || true
    # Bring up default client wifi connection if any
    DEFAULT_CON=$(nmcli -t -f NAME,TYPE con show | grep ":802-11-wireless" | grep -v "$HOTSPOT_NAME" | head -n 1 | cut -d: -f1)
    if [ -n "$DEFAULT_CON" ]; then
      echo "[*] Reconnecting to $DEFAULT_CON..."
      sudo nmcli connection up "$DEFAULT_CON" || true
    fi
  fi
  echo "[OK] Switched back to standard Wi-Fi mode."
}

case "$ACTION" in
  enable|start|ap)
    enable_hotspot
    ;;
  disable|stop|client)
    disable_hotspot
    ;;
  status)
    show_status
    ;;
  *)
    echo "Usage: $0 [enable | disable | status]"
    exit 1
    ;;
esac
