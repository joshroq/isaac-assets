#!/bin/bash
# rover-wifi-roam: move the rover to a CLEARLY stronger access point of the SAME network.
#
# Why: the Jetson's RTL8822CE + NetworkManager is a "sticky client" - it keeps the access
# point it joined at boot even when another node of the same mesh is far stronger
# (2026-11-11 garage: stuck on the far node at -70 dBm / 58 Mbit/s, MCS1, SSH stalls of 7+ s,
# 100 % ping loss, while the near node measured -32 dBm). It never roamed by itself.
#
# What: every PERIOD s, if the link is weaker than WEAK dBm, scan; if an AP with the same
# SSID is at least GAIN dB stronger, reconnect to it (BSSID pinned only for that one
# activation, then UNPINNED in the saved profile so nothing stays locked to one AP - the
# rover still joins any AP / network normally at the next boot or location, e.g. ODU).
# If the switch fails it reconnects normally. Cooldown prevents flapping.
#
# Installed as a root systemd service by install_wifi_roam.sh; log: journalctl -u rover-wifi-roam
IF="${IF:-wlP1p1s0}"
PERIOD="${PERIOD:-20}"
WEAK="${WEAK:--62}"        # only look for a better AP when the link is weaker than this
GAIN="${GAIN:-10}"         # ... and only switch for at least this many dB more signal
COOLDOWN="${COOLDOWN:-120}"
GATEWAY_TRIES=3

log() { echo "$(date '+%F %T') $*"; }
last_roam=0
while true; do
  sleep "$PERIOD"
  link=$(iw dev "$IF" link 2>/dev/null) || continue
  echo "$link" | grep -q "Connected to" || continue
  cur=$(echo "$link" | awk '/Connected to/{print tolower($3)}')
  sig=$(echo "$link" | awk '/signal:/{print int($2)}')
  ssid=$(echo "$link" | sed -n 's/^[[:space:]]*SSID: //p')
  [ -n "$sig" ] && [ -n "$ssid" ] || continue
  [ "$sig" -lt "$WEAK" ] || continue
  now=$(date +%s); [ $((now - last_roam)) -ge "$COOLDOWN" ] || continue
  # best AP advertising the same SSID: strongest signal, 5 GHz preferred by BAND_BONUS dB
  # (same node: 2.4 GHz reads a few dB stronger but carries ~1/3 of the throughput)
  best=$(iw dev "$IF" scan 2>/dev/null | awk -v want="$ssid" -v bonus="${BAND_BONUS:-6}" '
      /^BSS /      { mac = tolower(substr($2, 1, 17)); s = -999; f = 0 }
      /freq:/      { f = int($2) }
      /signal:/    { s = int($2) }
      /^\tSSID: /  { name = substr($0, index($0, ":") + 2)
                     sc = s + (f >= 5000 ? bonus : 0)
                     if (name == want && sc > bsc) { bsc = sc; bs = s; bm = mac } }
      BEGIN        { bsc = -999 }
      END          { if (bm != "") print bm, bs }')
  [ -n "$best" ] || continue
  bm=${best% *}; bs=${best#* }
  [ "$bm" != "$cur" ] || continue
  [ $((bs - sig)) -ge "$GAIN" ] || { log "weak ($sig dBm on $cur) but best same-SSID AP $bm is only $bs dBm"; continue; }
  con=$(nmcli -t -g GENERAL.CONNECTION dev show "$IF" 2>/dev/null)
  [ -n "$con" ] || continue
  log "roaming: $cur at $sig dBm -> $bm at $bs dBm ($ssid, profile '$con')"
  last_roam=$now
  nmcli con modify "$con" 802-11-wireless.bssid "$bm"
  nmcli con up "$con" >/dev/null 2>&1
  nmcli con modify "$con" 802-11-wireless.bssid ""        # never leave it pinned
  sleep 8
  gw=$(ip -4 route show default | awk '{print $3; exit}')
  ok=0
  for _ in $(seq $GATEWAY_TRIES); do ping -c1 -W2 "$gw" >/dev/null 2>&1 && { ok=1; break; }; done
  now_ap=$(iw dev "$IF" link 2>/dev/null | awk '/Connected to/{print tolower($3)}')
  if [ "$ok" = 1 ]; then
    log "ok: now on $now_ap at $(iw dev "$IF" link | awk '/signal:/{print $2}') dBm"
  else
    log "switch failed (gateway unreachable) - reconnecting normally"
    nmcli con up "$con" >/dev/null 2>&1 || nmcli dev connect "$IF" >/dev/null 2>&1
  fi
done
