# rover_comms — desktop ↔ physical rover ROS 2 link

Reliable ROS 2 communication between the **Linux desktop** (Isaac Sim / Nav2 /
teleop) and the **Waveshare Jetson Orin rover**, for two situations:

- **near** — rover sitting next to the desktop
- **far**  — rover 30 min away at the university, driven from the home desktop

Both use **one method**: a **Fast DDS Discovery Server** on the desktop. The
only thing that changes is which desktop IP the rover points at.

---

## Why the old setup failed (root cause)

ROS 2 / Fast DDS default discovery uses **UDP multicast**. Your WiFi AP forwards
unicast (pings work) but **drops multicast** — proven by packet capture in
`../chatgpt_logs`: the rover's discovery packets leave `wlP1p1s0` but never
arrive at the desktop. So the two machines never discover each other.

The earlier fix attempt (`fastdds_rover.xml` with `initialPeersList` +
`metatrafficUnicastLocatorList` overrides) *did* remove the multicast
dependency, but by **overriding the locator lists** it also broke Isaac's
**local** discovery — which is why Isaac kept "seeing nothing." That file is now
retired.

**Discovery Server** fixes discovery without that side effect: it is pure
unicast (no multicast) and does **not** touch locator lists, so local Isaac
keeps working while the rover connects.

```
        rover node ─┐                        ┌─ desktop node / teleop / Nav2
                    │  unicast to server     │
                    └──►  Discovery Server  ◄─┘   (on desktop, 0.0.0.0:11811)
                              relays "who exists"
        then topic DATA flows peer-to-peer directly (unicast)
```

---

## Files

```
rover_comms/
├── README.md                         ← you are here
├── TAILSCALE.md                      ← the near/far transport + uplink caveat
├── desktop/
│   ├── discovery_server.sh           ← run this on the desktop (the server)
│   ├── desktop_env.sh                ← source in desktop ROS terminals
│   └── fastdds-discovery-server.service  ← optional: run server as a service
├── rover/
│   ├── rover_env.sh                  ← copy to rover; source it (near|far)
│   └── ROVER_SETUP.md                ← copy-paste brief to apply on the rover
└── verify/
    └── verify_desktop.sh             ← desktop-side sanity check
```

Shared config baked into the scripts (edit if your IPs change):
- Domain `0`, `rmw_fastrtps_cpp`, `ROS_LOCALHOST_ONLY=0`
- Server port `11811`, server id `0`
- Desktop Tailscale IP `100.107.253.10` · Desktop ugv-AP LAN IP `10.168.168.240`

---

## Quickstart — DESKTOP side (do this first)

```bash
cd ~/projects/isaac-assets

# 1. Start the discovery server (leave running; own terminal)
rover_comms/desktop/discovery_server.sh

# 2. In every OTHER desktop ROS terminal, point at the server:
source rover_comms/desktop/desktop_env.sh

# 3. Sanity check
rover_comms/verify/verify_desktop.sh
```

Optionally make the server permanent (survives closing the terminal):
see the header of `desktop/fastdds-discovery-server.service`.

## Quickstart — ROVER side

Follow `rover/ROVER_SETUP.md` on the Jetson (remove the old profile, source
`rover_env.sh`, restart the ugv driver). Pick the mode:

| Mode   | When                                   | Rover points at        | Internet needed? |
|--------|----------------------------------------|------------------------|------------------|
| `near` | desktop joined the rover's `ugv` AP    | `10.168.168.240:11811` | no               |
| `far`  | university, over Tailscale             | `100.107.253.10:11811` | **yes** (uplink) |

For `far`, the rover must be on Tailscale **and** have an internet uplink (it
can't be its own AP-only) — see `TAILSCALE.md`.

## Verify the whole link

```bash
# desktop
ros2 topic info /cmd_vel --verbose        # Subscription count >= 1 (rover driver)
ros2 topic list | grep -E 'scan|odom|imu' # rover's real sensor topics appear

# drive it
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Isaac Sim + real rover at the same time (optional mirrored control)

Isaac normally uses plain local multicast (works on one machine). To ALSO drive
the physical rover from the same `/cmd_vel`, launch Isaac from a terminal that
has `desktop_env.sh` sourced so Isaac joins the discovery server too. Isaac's
sim topics stay under `/ugv_sim/*`; the shared drive topic is `/cmd_vel`. If you
don't need the real rover, launch Isaac normally (no discovery env) as before.

---

## Troubleshooting

- **`ros2 topic list` empty on desktop:** server not running, or the terminal
  didn't source `desktop_env.sh` (check `echo $ROS_DISCOVERY_SERVER`).
- **Rover can't be pinged in far mode:** Tailscale down or rover has no internet
  uplink (AP-only). See `TAILSCALE.md`.
- **Discovery works but `ros2 topic echo` hangs (far mode):** the participants
  advertised LAN locators that the other side can't reach. Restrict DDS to the
  Tailscale interface with an interface whitelist. Create `~/fastdds_tailscale.xml`
  on each machine listing loopback + that machine's own `100.x` IP under
  `<interfaceWhiteList>`, and `export FASTRTPS_DEFAULT_PROFILES_FILE=...` to it.
  (Whitelist only — do **not** re-add `initialPeersList` / locator overrides.)
- **Nothing connects at all:** confirm both ends match `ROS_DOMAIN_ID=0`,
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_LOCALHOST_ONLY=0`. A Waveshare
  image defaulting to CycloneDDS will never talk to Fast DDS.
- **Stale state after config changes:** `ros2 daemon stop` on both ends and
  restart the ROS processes so they re-read the env.
