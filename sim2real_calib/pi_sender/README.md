# pi_sender — Raspberry Pi capture host (auto-start C920e streamer)

Permanent replacement for the temporary Windows `laptop_sender/`. On boot the Pi
grabs the C920e and streams H.264/MPEG-TS over UDP to the desktop's Tailscale IP
(`100.107.253.10:5000`), where `capture/receive.py` (and the calibration tools)
consume it. No login required once installed.

## Phase 1 — Flash the SD card (on your desktop/laptop)

Use **Raspberry Pi Imager**.
1. Choose OS: **Raspberry Pi OS (64-bit)** — the "Lite" (no-desktop) build is ideal
   for a headless appliance.
2. Click the gear / **Edit Settings** before writing and set:
   - **Hostname:** `pi-cam`
   - **Enable SSH** → "Use password authentication" (or paste a public key).
   - **Username/password** — remember these; the service runs as this user.
   - **Wi-Fi SSID + password** + your country (so it joins the network on boot).
3. Write the card, put it in the Pi, plug in the **C920e** and power.

## Phase 2 — First login + Tailscale

Find it on the LAN and SSH in (mDNS usually works):
```bash
ssh <user>@pi-cam.local        # or ssh <user>@<lan-ip>
```

Install Tailscale and join the tailnet with SSH enabled:
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up --ssh
```
Open the printed URL, authenticate to your tailnet. After this you can reach it
from anywhere as `pi-cam` over Tailscale, and `tailscale ssh pi-cam` works without
managing keys. Confirm the desktop is reachable:
```bash
ping -c3 100.107.253.10
```

## Phase 3 — Install the auto-start streamer

Copy this folder to the Pi and run the installer:
```bash
# from your desktop, in sim2real_calib/:
scp -r pi_sender <user>@pi-cam:~/pi_sender
# then on the Pi:
cd ~/pi_sender && chmod +x install_on_pi.sh stream_camera.sh && ./install_on_pi.sh
sudo reboot        # first time only, so the 'video' group takes effect
```

That's it — the camera streams on every power-on.

## Verify

On the **desktop**, run the receiver and you should see the feed within seconds:
```bash
python3 sim2real_calib/capture/receive.py
```
On the **Pi**, check the service / logs:
```bash
systemctl status sim2real-camera
journalctl -u sim2real-camera -f
```

## Tuning / troubleshooting

- Settings (resolution, fps, bitrate, desktop IP) live as `Environment=` lines in
  `/etc/systemd/system/sim2real-camera.service`. After editing:
  `sudo systemctl daemon-reload && sudo systemctl restart sim2real-camera`.
- **Laggy/smeary:** drop `RES=640x480` and/or `BITRATE=2M`.
- **Underpowered Pi (Zero 2 W):** libx264 720p30 may saturate the CPU — use
  `RES=640x480`, or switch the encoder in `stream_camera.sh` to the hardware
  `-c:v h264_v4l2m2m`.
- **Wrong/again camera:** `v4l2-ctl --list-devices` shows nodes; the script prefers
  the stable `/dev/v4l/by-id/*C920e*` path automatically.
- **No feed:** confirm `ping 100.107.253.10` works from the Pi and that only one
  consumer holds UDP 5000 on the desktop.
