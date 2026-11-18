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
│   ├── drive_rover.sh                ← teleop the real rover (watchdog-safe)
│   └── fastdds-discovery-server.service  ← optional: run server as a service
├── rover/
│   ├── bootstrap.sh                  ← one-command rover setup after git clone
│   ├── install_rover_comms.sh        ← idempotent installer
│   ├── rover_env.sh                  ← the env (near|far); source it
│   ├── bin/                          ← rover-{env,mode,bringup,verify,
│   │                                    teleop-test,cmdvel-watchdog,wifi-join}
│   ├── watchdog/cmd_vel_watchdog.py  ← rover-side deadman (stops on link loss)
│   ├── campus_wifi.md                ← campus uplink (captive portal/enterprise)
│   ├── tailscale_up.sh               ← join the tailnet (far mode)
│   ├── systemd/rover-comms-bringup.service
│   └── ROVER_SETUP.md                ← full rover walkthrough
└── verify/
    ├── verify_desktop.sh             ← desktop-side sanity check
    └── link_monitor.sh               ← live link/RTT/graph monitor
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

## Remote operation (rover on campus, desktop at home): method & safety

The far setup is the same Discovery Server over Tailscale — **near and far are
ROS-identical**, so develop with physical access first, then relocate.

**Phase 0 — at home, co-located (do ~90% here).**
- Put the rover on Tailscale (`rover/tailscale_up.sh`) and confirm your lifeline:
  `ssh <user>@<rover-100.x>` works over the tunnel. This is how you fix anything
  once the rover is remote.
- Prove the *real* far path on the bench: `rover-mode far` so the rover targets
  the desktop's **Tailscale** IP (routes through the tunnel, not the LAN). If
  telemetry + on-blocks driving work here, campus is just the uplink question.
- Test the **deadman**: `rover-bringup` starts a rover-side cmd_vel watchdog
  (`watchdog/cmd_vel_watchdog.py`). Drive on blocks, then `sudo tailscale down`
  mid-command → wheels stop; `tailscale up` → control resumes.

**Phase 1 — campus, telemetry only.** Rover joins campus WiFi as a client
(`rover/campus_wifi.md`), Tailscale reconnects, driver service comes up. From
home: `rover-verify`, `ros2 topic echo /scan|/odom`, `verify/link_monitor.sh`.
**No autonomous motion** until rates/latency look good.

**Phase 2 — campus, supervised driving / nav.** Wheels on blocks first, then
floor driving with a **live camera feed + an on-campus spotter**, strict speed
caps. For Nav2, run the nav stack **on the rover** and send goals from home —
do **not** stream raw sensors/camera over the WAN.

### Safety essentials (remote driving)
- **Deadman is mandatory:** the Waveshare driver has no cmd_vel timeout, so the
  rover-side watchdog is what stops a runaway on link loss. Keep it enabled.
- **Drive via the watchdog:** teleop from the desktop with
  `desktop/drive_rover.sh` — it publishes `/cmd_vel_remote`, which the watchdog
  passes through to `/cmd_vel` (and zeroes on staleness). Publishing straight to
  `/cmd_vel` from the desktop bypasses the deadman — don't, for remote driving.
- **On blocks first, spotter always**, per `jetson_rover_setup` ("keep the rover
  on blocks while testing").
- **Clean WAN DDS (optional):** if far discovery works but `ros2 topic echo`
  hangs, set `ROVER_COMMS_DDS_WHITELIST=1` (rover) and `DESKTOP_DDS_WHITELIST=1`
  (desktop) before sourcing the env — restricts DDS to loopback + Tailscale.

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
- **Discovery works but `ros2 topic echo` hangs (far mode):** participants
  advertised LAN locators the other side can't reach. Restrict DDS to loopback +
  Tailscale: set `ROVER_COMMS_DDS_WHITELIST=1` (rover) / `DESKTOP_DDS_WHITELIST=1`
  (desktop) before sourcing the env — the env scripts auto-generate a safe
  whitelist-only `fastdds_tailscale.xml` from each machine's live `100.x` IP.
- **Rover keeps moving after the link drops:** the cmd_vel watchdog isn't
  running or you drove via `/cmd_vel` directly. Use `desktop/drive_rover.sh`
  (publishes `/cmd_vel_remote`) and confirm `rover-bringup` logged the watchdog.
  (Whitelist only — do **not** re-add `initialPeersList` / locator overrides.)
- **Nothing connects at all:** confirm both ends match `ROS_DOMAIN_ID=0`,
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `ROS_LOCALHOST_ONLY=0`. A Waveshare
  image defaulting to CycloneDDS will never talk to Fast DDS.
- **Stale state after config changes:** `ros2 daemon stop` on both ends and
  restart the ROS processes so they re-read the env.
