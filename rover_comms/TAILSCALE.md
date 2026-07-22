# Tailscale overlay (the "far" transport, works for "near" too)

Tailscale is a zero-config WireGuard mesh VPN. It gives every machine a stable
`100.x.y.z` IP that is reachable **identically whether the machines are on the
same LAN or across the internet** — which is exactly why we use one method for
both the nearby demo and the 30-minutes-away university demo.

## Current tailnet (already set up)

The desktop is already installed and logged in:

| Machine       | Tailscale IP     | Notes                         |
|---------------|------------------|-------------------------------|
| josh-desktop  | `100.107.253.10` | runs the Discovery Server     |
| homunculus    | `100.78.143.16`  | (windows, not used here)      |
| odu-4k8lx74   | `100.92.52.9`    | (windows, not used here)      |
| **rover**     | _not joined yet_ | add it per step below         |

Account: `joshusroquemore@`. The rover must join the **same** account.

## Add the rover

On the rover (needs internet uplink — see caveat below):
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up           # opens a login URL; authenticate as joshusroquemore@
tailscale ip -4             # -> the rover's 100.x.y.z
```
Then from the rover: `ping 100.107.253.10` should reach the desktop.

## The internet-uplink caveat (important for the university demo)

Tailscale needs each node to reach the internet to coordinate the tunnel. The
rover's **`ugv` WiFi is an access point the rover itself hosts — in that mode
the rover has no upstream internet**, so Tailscale can't connect from it.

For remote (far) operation the rover must instead be a **client** of a network
that has internet:
- join the university WiFi as a station, **or**
- tether to a phone hotspot, **or**
- plug in ethernet.

Once the rover has internet + Tailscale, the desktop at home controls it over
`100.x` regardless of distance. No port-forwarding, no public IP, no router
changes.

## Why Tailscale + Discovery Server (not multicast, not initial-peers XML)

- **Multicast** is dropped by the AP (proven by packet capture) and never
  routes across the internet at all — dead end for both near and far.
- **initial-peers XML** worked in theory but overrode the participant locator
  lists, which isolated Isaac's local nodes (the failure you kept hitting).
- **Discovery Server** is pure unicast, needs no multicast, and touches no
  locator lists — so local Isaac keeps working while the rover connects.
- **Tailscale** gives the Discovery Server a single stable IP that is valid
  near and far, so the rover config is identical in both locations.

## Near demo without Tailscale (fallback)

If you just want the nearby demo and the rover is running its own `ugv` AP with
no internet, skip Tailscale: have the desktop **join the rover's `ugv` WiFi**,
then use `near` mode (rover points at the desktop's `10.168.168.240`). Same
Discovery Server, just the LAN IP. Reserve static DHCP leases so the IPs don't
drift.
