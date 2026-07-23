# Campus WiFi (rover uplink for far mode)

For remote operation the rover needs real internet so Tailscale can connect. Its
own `ugv` AP has **no uplink**, so at campus the rover must join the university
network (or a phone hotspot / ethernet) as a **client**. This is the single most
failure-prone part of remote operation on a headless Jetson — set it up and
prove it **at home first** if you can.

## Quick path (WPA2-Personal / open, or campus with a password)

```bash
rover-wifi-join "<SSID>" "<PASSWORD>"      # creates an autoconnecting profile
tailscale ip -4                            # confirm rover got on the tailnet
ping 100.107.253.10                        # reach the desktop
```

## WPA2-Enterprise (very common on campus: eduroam-style, per-user login)

```bash
rover-wifi-join --enterprise "<SSID>" "<your-username>" "<your-password>"
# if the campus requires a CA certificate:
sudo nmcli connection modify "<SSID>" 802-1x.ca-cert /path/to/campus-ca.pem
sudo nmcli connection up "<SSID>"
```
The exact EAP method (PEAP/TTLS), phase-2 auth, and whether a CA cert / anonymous
identity is required are campus-specific — check your IT's Linux/"other device"
instructions. The helper defaults to PEAP + MSCHAPv2, which covers most.

## Captive portals (click-through login pages)

A headless Jetson can't click a portal. Options, in order of reliability:
1. **Register the rover's WiFi MAC** in the campus device portal ahead of time
   (many universities let you pre-register "gaming/IoT" devices) — then it joins
   without a portal.
2. Do the portal once from a browser while the Jetson is reachable (VNC / attach
   a screen), then it's remembered for a while.
3. **Avoid it entirely with a phone hotspot / cellular** — portal-free and the
   most reliable for peer-to-peer Tailscale.

## Client isolation

Campus APs often block client-to-client traffic. That breaks a same-LAN ROS link
but **not** Tailscale — Tailscale falls back to a DERP relay (a bit more latency,
still works). This is a key reason the far setup uses Tailscale rather than LAN IPs.

## Verify the uplink is solid before relying on it

```bash
ping -c3 1.1.1.1          # internet works
tailscale status          # shows the desktop peer
tailscale ip -4           # rover's 100.x
ping -c3 100.107.253.10   # desktop reachable over the tunnel
```

## Auto-recovery (so it comes back on its own)

- The WiFi profile is created with `connection.autoconnect yes`, so it rejoins on
  boot / when in range.
- Tailscale persists across reboots (`tailscale up` once).
- The driver runs under `rover-comms-bringup.service` with `Restart=on-failure`.
- Your **SSH-over-Tailscale** session (`ssh jroqu005@<rover-100.x>`) is the remote
  lifeline for everything else. Confirm it works before the rover leaves.
