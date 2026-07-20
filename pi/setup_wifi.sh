#!/usr/bin/env bash
# Pin the Pi's WiFi to a fixed static IP on the HOME network.
#
# Replaces the old setup_ap.sh (the Pi used to host its own WiFi network;
# that was retired 2026-07-20 because it was the main source of flaky
# bring-up). Now the Pi is just another device on the house WiFi router,
# same as the DevKit and the ESP32-CAM.
#
# If the SD card was imaged with Raspberry Pi Imager's WiFi preset (SSID
# + password set at imaging time, in the gear-icon "advanced options"),
# the Pi already joins home WiFi over DHCP before you ever run this
# script — this script's only job is switching that connection from a
# DHCP-assigned address to the fixed one everything else expects.
#
# IP plan: home router .1, Pi .50 (this script), DevKit .51, CAM .52.
#
# Run once:  bash setup_wifi.sh
# Undo:      sudo nmcli connection modify "$SSID" ipv4.method auto && sudo nmcli connection up "$SSID"
set -euo pipefail

SSID="Airtel_kuma_9602"
PSK="air71417"
PI_IP="192.168.1.50"
GATEWAY="192.168.1.1"

# Old Pi-hosted-AP profile, if it exists from a previous setup — delete it
# so nothing fights for wlan0 at boot.
sudo nmcli connection delete car-ap 2>/dev/null || true

# Create the home-WiFi connection if it doesn't already exist (e.g. this
# Pi wasn't preset with the SSID/password at imaging time), then pin it
# to a static IP.
if ! nmcli -t -f NAME connection show | grep -qx "$SSID"; then
  sudo nmcli connection add type wifi ifname wlan0 con-name "$SSID" \
       ssid "$SSID" wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK"
fi

sudo nmcli connection modify "$SSID" \
     connection.autoconnect yes connection.autoconnect-priority 10 \
     ipv4.method manual ipv4.addresses "$PI_IP/24" \
     ipv4.gateway "$GATEWAY" ipv4.dns "$GATEWAY" \
     ipv6.method disabled

sudo nmcli connection up "$SSID"

echo
echo "Pi is on '$SSID' at $PI_IP."
echo "Dashboard will be at http://$PI_IP/ once webapp is running."
echo "Next: flash the DevKit and CAM with the same SSID/password (already"
echo "the default in CarTestBench/config.h and CamStreamer/CamStreamer.ino),"
echo "then check they answer: ping 192.168.1.51 && ping 192.168.1.52"
