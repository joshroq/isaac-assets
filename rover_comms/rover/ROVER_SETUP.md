# Rover-side setup (Waveshare Jetson Orin)

Everything the rover needs to talk to the desktop over the Fast DDS Discovery
Server. There is **no XML profile** — the old `fastdds_rover.xml` initial-peers
approach is retired (it isolated nodes).

The rover's real driver is the Waveshare **`ugv_bringup`** launch, normally run
through `ugv-env` (see `../../jetson_rover_setup`). This kit wraps that same
driver with the discovery-server environment and fixes the domain mismatch
(the old `ros2_rover.env` used `ROS_DOMAIN_ID=42`; the desktop/Isaac use **0**).

Assumed from prior setup: user `jroqu005`, ROS 2 Humble at `/opt/ros/humble`,
workspace `~/isaac-assets/ugv_ws`. Adjust `rover_env.sh` if paths differ.

---

## Contents

```
rover_comms/rover/
├── rover_env.sh              # single source of truth for ROS + discovery env
├── install_rover_comms.sh    # idempotent installer (run this)
├── tailscale_up.sh           # install + join tailnet (far mode)
├── bin/
│   ├── rover-env             # source env, then run a command (like ugv-env)
│   ├── rover-mode            # show/persist near|far
│   ├── rover-bringup         # launch ugv_bringup WITH the discovery env
│   ├── rover-verify          # rover-side link diagnostics
│   └── rover-teleop-test     # short /cmd_vel drive test
└── systemd/
    └── rover-comms-bringup.service
```

## 1. Get the repo onto the rover and install (GitHub workflow)

SSH into the rover (over the `ugv` WiFi from your laptop), clone the repo, and
run the one bootstrap command:

```bash
# on the rover:
git clone https://github.com/<you>/isaac-assets.git ~/isaac-assets   # first time
# or, if it's already there:  cd ~/isaac-assets && git pull

cd ~/isaac-assets/rover_comms/rover
./bootstrap.sh
```

`bootstrap.sh` checks ROS 2 Humble + Fast DDS, offers to build the Waveshare
driver workspace if it isn't built yet, then runs `install_rover_comms.sh`,
which: removes any old `FASTRTPS_DEFAULT_PROFILES_FILE` lines, installs
`rover_env.sh` to `/usr/local/lib/rover_comms/` and the `rover-*` helpers to
`/usr/local/bin`, records where the repo lives, installs the systemd service,
sets default mode `far`, and optionally installs Tailscale.

You can clone to any path — the scripts detect the repo location; they are not
hard-wired to `~/isaac-assets`. (Just re-run `./bootstrap.sh` after a `git pull`
to pick up updates.)

## 2. Pick the mode

| Mode   | When                                  | Rover points at        | Internet? |
|--------|---------------------------------------|------------------------|-----------|
| `near` | desktop joined the rover's `ugv` AP   | `10.168.168.240:11811` | no        |
| `far`  | university, over Tailscale            | `100.107.253.10:11811` | **yes**   |

```bash
rover-mode          # show
rover-mode near     # nearby, no-internet demo
rover-mode far      # remote over Tailscale
```

For **far**, put the rover on Tailscale (needs a real internet uplink — join
uni WiFi as a client / phone hotspot / ethernet, NOT the rover's own AP):

```bash
./tailscale_up.sh          # log in as the SAME account as the desktop
```

## 3. Start the driver

Foreground (for a first test):
```bash
rover-bringup
```
Or as a service (recommended). Enable only ONE driver service:
```bash
sudo systemctl disable --now ugv-bringup.service        # if it was enabled
sudo systemctl enable  --now rover-comms-bringup.service
```

## 4. Verify

```bash
rover-verify
```
Checks: env baseline (domain 0 / Fast DDS / localhost_only 0), reachability to
the desktop, Tailscale (far mode), and that the rover's own `/cmd_vel`
subscriber + real `/scan,/odom,/imu,/tf` topics are up.

Then, with the desktop's discovery server running and the desktop driving:
```bash
rover-env ros2 topic echo /cmd_vel     # twists from the desktop appear here
rover-env ros2 node list               # desktop nodes visible = link is up
```

## 5. Drive test (moves the rover)

```bash
rover-teleop-test 0.05 0.0 1.0         # 0.05 m/s forward, 1 s, then stop
```

---

## Notes / gotchas

- **Domain must be 0** everywhere (desktop, Isaac, rover). `rover_env.sh` forces
  it; if you still run the old `ugv-bringup.service`, it will use `ros2_rover.env`
  (domain 42) and won't connect — that's why we switch to `rover-comms-bringup`.
- **RMW must be `rmw_fastrtps_cpp`.** If the Waveshare image defaults to
  CycloneDDS, it will never talk to the desktop. `rover_env.sh` sets Fast DDS.
- **IPs drift?** Override without editing scripts: put
  `DESKTOP_TAILSCALE_IP=...` / `DESKTOP_LAN_IP=...` in
  `~/.config/rover_comms/desktop_ips.env`. Reserve static DHCP leases for near mode.
- **Far discovery works but `ros2 topic echo` hangs:** participants advertised
  unreachable LAN locators. Restrict DDS to the Tailscale interface with an
  interface-whitelist XML (loopback + the rover's own `100.x` only) — see the
  Troubleshooting section of `../README.md`.
