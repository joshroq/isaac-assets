#!/usr/bin/env bash
# Run this ON THE PI (from the copied pi_sender/ folder) to install the
# auto-start camera streamer. Idempotent: safe to re-run after editing.
#
#   chmod +x install_on_pi.sh
#   ./install_on_pi.sh
set -euo pipefail

cd "$(dirname "$0")"

echo "[1/5] installing ffmpeg + v4l-utils ..."
sudo apt-get update
sudo apt-get install -y ffmpeg v4l-utils

echo "[2/5] adding $USER to the 'video' group (camera access) ..."
sudo usermod -aG video "$USER"

echo "[3/5] installing stream script to /opt/sim2real ..."
sudo mkdir -p /opt/sim2real
sudo cp stream_camera.sh /opt/sim2real/stream_camera.sh
sudo chmod +x /opt/sim2real/stream_camera.sh

echo "[4/5] installing systemd service (runs as $USER) ..."
sed "s/@USER@/$USER/" sim2real-camera.service \
  | sudo tee /etc/systemd/system/sim2real-camera.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable sim2real-camera.service
sudo systemctl restart sim2real-camera.service

echo "[5/5] status:"
sleep 2
systemctl --no-pager --full status sim2real-camera.service || true

cat <<'EOF'

Done. The camera now streams automatically on every boot.
Useful commands:
  systemctl status sim2real-camera      # is it running?
  journalctl -u sim2real-camera -f      # live logs (ffmpeg output)
  sudo systemctl restart sim2real-camera
  sudo systemctl stop sim2real-camera   # temporarily stop streaming

If you were just added to the 'video' group for the first time, reboot once
so the group membership takes effect for the service.
EOF
