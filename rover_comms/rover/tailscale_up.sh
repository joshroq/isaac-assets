#!/usr/bin/env bash
# tailscale_up.sh — install Tailscale on the rover and join the tailnet (far
# mode). The rover MUST have a working internet uplink for this (join uni WiFi
# as a client, a phone hotspot, or ethernet — NOT its own ugv AP-only mode).
#
# Join the SAME account as the desktop (joshusroquemore@). After this,
# `tailscale ip -4` gives the rover's 100.x address and it can reach the
# desktop's discovery server at 100.107.253.10:11811 from anywhere.
set -eo pipefail

if ! command -v tailscale >/dev/null 2>&1; then
  echo "[tailscale_up] installing Tailscale…"
  curl -fsSL https://tailscale.com/install.sh | sh
else
  echo "[tailscale_up] tailscale already installed: $(tailscale version | head -1)"
fi

echo "[tailscale_up] checking internet uplink…"
if ! ping -c1 -W3 1.1.1.1 >/dev/null 2>&1; then
  echo "[tailscale_up] WARNING: no internet reachable. Tailscale cannot connect"
  echo "               from the rover's own ugv AP. Give the rover an uplink"
  echo "               (uni WiFi client / phone hotspot / ethernet) first."
fi

echo "[tailscale_up] bringing Tailscale up (a login URL will be printed — log in"
echo "               as the SAME account as the desktop: joshusroquemore@)…"
sudo tailscale up

echo
echo "[tailscale_up] rover tailnet IP: $(tailscale ip -4 2>/dev/null | head -1 || echo '<not connected>')"
echo "[tailscale_up] testing reach to desktop 100.107.253.10 …"
if ping -c2 -W3 100.107.253.10 >/dev/null 2>&1; then
  echo "[tailscale_up] OK — desktop reachable over Tailscale."
else
  echo "[tailscale_up] Could NOT reach the desktop yet. Ensure the desktop is"
  echo "               powered on and on the same tailnet (tailscale status)."
fi
