#!/bin/bash
# Install / update the Wi-Fi roam guard on the rover (needs sudo).
#   bash ~/rover_nav/wifi_roam/install_wifi_roam.sh            # install + start
#   bash ~/rover_nav/wifi_roam/install_wifi_roam.sh --remove   # uninstall
set -e
D="$(cd "$(dirname "$0")" && pwd)"
if [ "${1:-}" = "--remove" ]; then
  sudo systemctl disable --now rover-wifi-roam 2>/dev/null || true
  sudo rm -f /usr/local/sbin/rover-wifi-roam /etc/systemd/system/rover-wifi-roam.service
  sudo systemctl daemon-reload; echo "removed"; exit 0
fi
bash -n "$D/rover-wifi-roam.sh"
sudo install -m 755 "$D/rover-wifi-roam.sh" /usr/local/sbin/rover-wifi-roam
sudo install -m 644 "$D/rover-wifi-roam.service" /etc/systemd/system/rover-wifi-roam.service
sudo systemctl daemon-reload
sudo systemctl enable --now rover-wifi-roam
sudo systemctl restart rover-wifi-roam
systemctl is-active rover-wifi-roam
