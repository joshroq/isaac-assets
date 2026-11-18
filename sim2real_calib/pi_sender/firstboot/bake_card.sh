#!/usr/bin/env bash
# Run this ON THE DESKTOP with the Pi's SD card in the reader. It bakes the
# first-boot Tailscale bootstrap onto the card (rootfs) and, as a backup, adds
# this desktop's SSH public key to the Pi user's authorized_keys.
#
# It does NOT write the auth key itself - you place that separately at
# <bootfs>/ts-authkey (kept out of version control and shredded on first boot).
#
#   sudo ./bake_card.sh
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
PUBKEY="${PUBKEY:-$HOME/.ssh/id_ed25519.pub}"
# When invoked via sudo, $HOME is root's; fall back to the invoking user's key.
if [ ! -f "$PUBKEY" ] && [ -n "${SUDO_USER:-}" ]; then
  PUBKEY="/home/$SUDO_USER/.ssh/id_ed25519.pub"
fi

find_mount() {  # $1 = filesystem label -> prints mountpoint (mounting if needed)
  local label="$1" dev mp
  dev="$(blkid -L "$label" 2>/dev/null || true)"
  [ -n "$dev" ] || { echo "ERROR: no partition labeled '$label' found" >&2; return 1; }
  mp="$(findmnt -no TARGET "$dev" 2>/dev/null || true)"
  if [ -z "$mp" ]; then
    mp="/mnt/sd_$label"; mkdir -p "$mp"; mount "$dev" "$mp"
  fi
  echo "$mp"
}

echo "[*] locating SD card partitions ..."
ROOT="$(find_mount rootfs)"
BOOT="$(find_mount bootfs)"
echo "    rootfs -> $ROOT"
echo "    bootfs -> $BOOT"

echo "[*] installing first-boot script + service ..."
install -m 0755 "$HERE/ts-firstboot.sh" "$ROOT/usr/local/sbin/ts-firstboot.sh"
install -m 0644 "$HERE/ts-firstboot.service" "$ROOT/etc/systemd/system/ts-firstboot.service"
# Enable it the offline way: symlink into multi-user.target.wants.
mkdir -p "$ROOT/etc/systemd/system/multi-user.target.wants"
ln -sf ../ts-firstboot.service \
  "$ROOT/etc/systemd/system/multi-user.target.wants/ts-firstboot.service"

echo "[*] adding desktop SSH key to the Pi user's authorized_keys ..."
if [ -f "$PUBKEY" ]; then
  # The Pi's login user is the single non-root home under /home.
  PIUSER="$(ls "$ROOT/home" | head -1)"
  SSHDIR="$ROOT/home/$PIUSER/.ssh"
  mkdir -p "$SSHDIR"
  cat "$PUBKEY" >> "$SSHDIR/authorized_keys"
  sort -u "$SSHDIR/authorized_keys" -o "$SSHDIR/authorized_keys"
  # uid/gid of the first real user is 1000 on Raspberry Pi OS.
  chown -R 1000:1000 "$SSHDIR"
  chmod 700 "$SSHDIR"; chmod 600 "$SSHDIR/authorized_keys"
  echo "    added for pi user '$PIUSER'"
else
  echo "    WARN: no pubkey at $PUBKEY; skipping (Tailscale SSH will still work)"
fi

echo "[*] syncing ..."
sync
echo
echo "DONE. Now place your one-time auth key on the boot partition:"
echo "    echo 'tskey-auth-XXduplicate-your-key' > \"$BOOT/ts-authkey\""
echo "Then eject the card, put it in the Pi, and power on."
