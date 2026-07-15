#!/usr/bin/env bash
# Turn this Pi into the car's WiFi access point (Pi-centric phase B).
#
# Uses NetworkManager (stock on Raspberry Pi OS Bookworm) in AP mode with
# a shared IPv4 subnet — NM runs its own dnsmasq for DHCP, so there is no
# separate hostapd/dnsmasq install to fight with.
#
# Same SSID/PSK as the old ESP32 AP, so the ESP32-CAM firmware needs zero
# changes; it simply finds the "same" network with a better radio.
#
# IP plan: Pi .1 (this AP), DevKit .5 (static), CAM .10 (static),
# phones .100-.200 (DHCP).
#
# Run once:  bash setup_ap.sh
# Undo:      sudo nmcli connection delete car-ap   (and re-enable 'car')
set -euo pipefail

SSID="RC_Car_TestBench"
PSK="carbench123"
AP_IP="192.168.4.1"

# DHCP pool for phones/laptops; .2-.99 stays clear for the static boards.
sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d
echo 'dhcp-range=192.168.4.100,192.168.4.200,12h' \
  | sudo tee /etc/NetworkManager/dnsmasq-shared.d/car-ap.conf >/dev/null

# The old station profile ('car', from the Pi Integration phase) must not
# fight for wlan0 at boot. Keep it around for manual use, autoconnect off.
sudo nmcli connection modify car connection.autoconnect no 2>/dev/null || true

sudo nmcli connection delete car-ap 2>/dev/null || true
sudo nmcli connection add type wifi ifname wlan0 con-name car-ap \
     autoconnect yes connection.autoconnect-priority 10 \
     ssid "$SSID" \
     802-11-wireless.mode ap 802-11-wireless.band bg \
     wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$PSK" \
     ipv4.method shared ipv4.addresses "$AP_IP/24" \
     ipv6.method disabled
sudo nmcli connection up car-ap

echo
echo "AP '$SSID' is up at $AP_IP."
echo "Check stations join: CAM should appear at 192.168.4.10, DevKit at .5"
echo "(flash the DevKit's station-mode firmware only AFTER this AP works)."
