#!/usr/bin/env bash
# First-boot Tailscale bootstrap, baked onto the SD card before the Pi's first
# power-on. Installs Tailscale (if needed) and joins the tailnet with SSH enabled,
# using a one-time auth key placed at /boot/firmware/ts-authkey. Then it shreds the
# key and disables itself so it never runs again.
#
# Works despite the AP's Wi-Fi client isolation: this only needs OUTBOUND internet
# to reach Tailscale's control plane, which isolation does not block.
set -uo pipefail

KEYFILE=/boot/firmware/ts-authkey
LOG=/var/log/ts-firstboot.log
exec >>"$LOG" 2>&1
echo "=== ts-firstboot $(date -Is) ==="

[ -f "$KEYFILE" ] || { echo "no authkey file; nothing to do"; exit 0; }
KEY="$(tr -d '[:space:]' < "$KEYFILE")"
[ -n "$KEY" ] || { echo "authkey file empty; aborting"; exit 0; }

# Wait for internet (DNS + reachability) - up to ~2 min.
for i in $(seq 1 24); do
  if curl -fsS --max-time 5 https://login.tailscale.com/health >/dev/null 2>&1; then
    echo "internet reachable (try $i)"; break
  fi
  echo "waiting for internet ($i) ..."; sleep 5
done

# Install Tailscale if it isn't already present.
if ! command -v tailscale >/dev/null 2>&1; then
  echo "installing tailscale ..."
  curl -fsSL https://tailscale.com/install.sh | sh
fi

# Join the tailnet: SSH enabled, stable hostname.
echo "tailscale up ..."
if tailscale up --authkey="$KEY" --ssh --hostname=pi-cam; then
  echo "tailscale up OK: $(tailscale ip -4 2>/dev/null | head -1)"
  # Success: remove the key and stop this service from ever running again.
  shred -u "$KEYFILE" 2>/dev/null || rm -f "$KEYFILE"
  systemctl disable ts-firstboot.service
  echo "done; key shredded, service disabled"
else
  echo "tailscale up FAILED; leaving key in place to retry next boot"
  exit 1
fi
