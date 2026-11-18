# Live-visualize the physical rover on the desktop (RViz)

Bring the **real rover's** live navigation graph — map, laser scan, TF, costmaps, plan,
robot model — into the **same RViz views** the sim uses (`navlaunch -v2 -<stack>`), running
on your desktop over SSH/Wi-Fi, and set Nav2 goals by clicking. Two commands total:

```bash
ssh rover  ~/rover_nav/rovernav lidar         # rover: bring a stack up (or: depth)
rover_comms/desktop/view_rover.sh lidar       # desktop: see it + set goals
```

## How it fits together

Nav2 + SLAM run **on the rover**; the desktop is a **pure viewer + goal sender**. The rover
uses **native ROS 2 multicast** for its own discovery (exactly like the sim / standalone
local nav), so its Nav2/SLAM bringup is reliable. The Wi-Fi AP drops multicast *between*
machines, so the **desktop** reaches the rover with a Fast DDS **initial-peers** profile —
that's entirely desktop-side and never disturbs the rover's discovery.

```
  ROVER (Jetson) — native multicast          DESKTOP (here)
  rovernav <stack>:                          view_rover.sh <stack>:
    ugv_bringup + LD19 + OAK                    initial-peers -> rover metatraffic ports
    slam_toolbox / Nav2  ───── LAN unicast ──   robot_state_publisher (real ugv_rover.urdf)
    /scan /map /tf /plan /costmaps  ────────►   rviz2 (same per-stack .rviz, de-namespaced)
    Nav2 subscribes /goal_pose  ◄────────────   click "2D Nav Goal" -> /goal_pose
```

> **Why no discovery server?** Routing the rover's own nodes through a Fast DDS Discovery
> Server (desktop- *or* rover-hosted) broke intra-rover **service** discovery — Nav2/SLAM
> bringup hung forever on `Waiting for service map_saver/get_state`. Native multicast fixes
> that. The desktop-side initial-peers profile is what crosses the AP.

## Recipe

**1 — Rover (ssh):** one command per stack. `lidar` and `depth` are fully orchestrated
(`depth` starts the OAK camera for you; set `ROVER_SUDO_PW=shen25` so it can chmod the USB
node without prompting):

```bash
ssh rover
export ROVER_SUDO_PW=shen25            # only needed for camera stacks (depth)
~/rover_nav/rovernav lidar             # or: rovernav depth
```
Leave it running (Ctrl-C stops it + the camera it started). Only **one** base stack at a
time — the built-in guard refuses a second (it would double-open the serial port).

**2 — Desktop:** one command, matching the stack:

```bash
cd ~/projects/isaac-assets
rover_comms/desktop/view_rover.sh lidar      # or: depth
rover_comms/desktop/view_rover.sh lidar far  # over Tailscale instead of the ugv AP
```
It builds the initial-peers profile (rover near `10.168.168.115` / far `100.89.177.82`;
override with `ROVER_IP=<ip>`), bounces the ROS 2 daemon, preflights, and opens RViz.

**3 — Set a goal:** click **2D Nav Goal** in RViz (publishes `/goal_pose`, which the rover's
Nav2 receives). Drive a little first so SLAM builds the map (`Robot is out of bounds of the
costmap` until then — benign). RViz-less fallback over ssh: `~/rover_nav/send_goal.py`.

## Stacks

| command | what | desktop view |
|---|---|---|
| `rovernav lidar` | LiDAR SLAM + Nav2 (self-contained) | `view_rover.sh lidar` |
| `rovernav depth` | LiDAR SLAM + Nav2 + OAK depth cloud | `view_rover.sh depth` |
| rgb / orbslam3 / cuvslam | multi-piece; `rovernav <stack>` prints the manual sequence | `view_rover.sh <stack>` |

`lidar` and `depth` are the reliable stacks. Depth's raw `/oak/points` cloud is slow
(~0.7 Hz — the OAK-D-Lite is on USB2 and `depth_image_proc` barely syncs), so **the depth
obstacles are best seen in the costmap** on the desktop, not the raw cloud.

## Fidelity notes (making it match the sim)

- **Robot model:** the viewer loads the rover's real `ugv_rover.urdf` (13 links), so the
  RobotModel matches the live TF exactly. (The sim's `ugv_rover_isaac_fixed.urdf` has 3 links
  the physical rover lacks → "complete RobotModel error"; don't use it here.)
- **360° LiDAR:** the LD19's default 90° rear angle-crop (225–315°) is disabled on the rover
  (`ld19.launch.py`, backup `.orig`) — the cropped sector returns real environment, not the
  rover body, so removing it restores full 360° coverage (like the sim) and gives slam_toolbox
  more data. Re-enable by restoring the `.orig` if you ever see phantom rear obstacles.
- **`rf2o` "Eigensolver couldn't find a solution"** spam is benign (laser-odometry solver on a
  stationary/sparse scan); wheel odom provides the odom→base TF regardless. It quiets under
  motion and with the fuller 360° scan.

## Troubleshooting

- **RViz empty / only `/scan` + `/tf`, no `/map` or costmaps:** the initial-peers range was
  too small to reach `nav2_container`. `view_rover.sh` probes 40 participant IDs by default;
  bump `ROVER_PEER_PARTICIPANTS=60` if the rover has been restarted many times (IDs drift up).
- **RViz empty entirely:** rover not in comms mode, or unreachable. Confirm `rovernav` printed
  `comms mode ON -> native multicast`, and that `ping <ROVER_IP>` works for your near/far mode.
- **Bringup hangs / serial garbage / `KeyError 'T'`:** two base stacks fighting over the serial
  port. Run `~/rover_nav/stop_all.sh`, then a single `rovernav`.
- **Goal ignored:** Nav2 must be active and a `map->base_footprint` TF present; check
  `ros2 topic info /goal_pose` shows a subscriber (the rover's Nav2).
- **`nav2_rviz_plugins/Selector`/`Docking` failed to load** (lidar view): harmless — the
  upstream config references newer Nav2 panels than are installed; displays still work.
