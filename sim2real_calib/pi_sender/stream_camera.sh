#!/usr/bin/env bash
# Grab the C920e and stream H.264 / MPEG-TS over UDP to the desktop (Tailscale).
# Runs headless; started automatically by sim2real-camera.service on boot.
#
# All settings are overridable via environment (the systemd unit sets them).
set -euo pipefail

DESKTOP_IP="${DESKTOP_IP:-100.107.253.10}"   # desktop's Tailscale IP (receiver)
PORT="${PORT:-5000}"
RES="${RES:-1280x720}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4M}"
# pkt_size 1128 (=6*188) stays under Tailscale's ~1280-byte MTU so UDP packets
# don't fragment and drop. Don't raise without testing for lag/loss.
PKTSIZE="${PKTSIZE:-1128}"

# Prefer the stable by-id node for the C920e so a shuffled /dev/videoN can't
# point us at the wrong camera; fall back to CAM_DEV or /dev/video0.
DEV=""
for d in /dev/v4l/by-id/*C920e*index0 /dev/v4l/by-id/*C920*index0; do
  [ -e "$d" ] && DEV="$d" && break
done
DEV="${DEV:-${CAM_DEV:-/dev/video0}}"

if [ ! -e "$DEV" ]; then
  echo "camera device $DEV not found; is the C920e plugged in?" >&2
  exit 1
fi

echo "streaming $DEV ${RES}@${FPS} -> udp://${DESKTOP_IP}:${PORT}"

# Fixed exposure (units of 100 us) + gain for sticker tracking. On auto, harsh garage light
# washed the orange-red sticker out to near-white (S~20) and it dropped out mid-spin, and a
# lighting change mid-session breaks the calibrated colours either way - keep the light steady.
# 2026-11-18 settings: dim garage light 300/200 (current); bright work light 100/51
# (calib/sticker_config.bright_exp100_20261118.json). CAM_EXPOSURE=auto restores auto exposure.
# Changing these changes the sticker colours: re-run calib/calibrate_stickers.py.
EXPOSURE="${CAM_EXPOSURE:-300}"
GAIN="${CAM_GAIN:-200}"
if command -v v4l2-ctl >/dev/null; then
  if [ "$EXPOSURE" = auto ]; then
    v4l2-ctl -d "$DEV" -c auto_exposure=3 || true
  else
    v4l2-ctl -d "$DEV" -c auto_exposure=1 -c exposure_time_absolute="$EXPOSURE" -c gain="$GAIN" || true
  fi
fi

# -input_format mjpeg: the C920e must deliver MJPEG to hit 720p30 over USB2
# (raw YUYV maxes out far lower). ffmpeg then re-encodes to low-latency H.264.
exec ffmpeg -hide_banner -loglevel warning \
  -f v4l2 -input_format mjpeg -video_size "$RES" -framerate "$FPS" -i "$DEV" \
  -c:v libx264 -preset ultrafast -tune zerolatency -pix_fmt yuv420p \
  -g "$FPS" -b:v "$BITRATE" -maxrate "$BITRATE" -bufsize 1M \
  -f mpegts "udp://${DESKTOP_IP}:${PORT}?pkt_size=${PKTSIZE}"
