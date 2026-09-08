# Rover local navigation — new-rover setup

This bundle is everything needed to run the Waveshare UGV rover's navigation stacks
**locally on the Jetson** (no desktop link). It is a versioned snapshot of what lives
in `~/rover_nav` on the current (battery-faulted) rover, plus the two out-of-tree
pieces it depends on (the `ugv_ws` source edit and the ORB-SLAM3 wrapper source).

> The current rover is being retired for a **hardware power fault** (failing 3S 18650
> UPS pack — runs on wall power only). Nothing here is a software workaround for that;
> this is a clean re-deploy onto new hardware.

## Target hardware / OS (assumed identical on the new rover)

- Waveshare UGV Rover, **Jetson Orin**, JetPack 6 / L4T r36.4, **ROS 2 Humble**, aarch64
- Base ESP32 (motors + IMU + wheel odom) on **`/dev/ttyTHS1`** @115200
- **LD19** lidar on `/dev/ttyACM0` @230400 → `/scan` @10 Hz
- **OAK-D** (RGB-D + IMU) over USB (vendor `03e7`)
- ~7.4 GB RAM — matters for the ORB-SLAM3 build (see OOM note below)

## What's in this bundle

| Path | Purpose | Destination on new rover |
|---|---|---|
| `00`–`07_*.sh` | ordered install/build scripts | `~/rover_nav/` |
| `start_*.sh`, `drive_keyboard.sh`, `stop_all.sh` | runtime launchers | `~/rover_nav/` |
| `*.launch.py`, `oak_stereo.yaml`, `inject_depth.py` | launch + configs | `~/rover_nav/` |
| `send_goal.py` | RViz-less Nav2 goal sender (SSH) | `~/rover_nav/` |
| `local_ros_env.sh` | the local-only ROS env every script sources | `~/rover_nav/` |
| `_sudo.sh` | sudo helper (password parameterized — see below) | `~/rover_nav/` |
| `param_backups/amcl_dwa.yaml.orig` | pristine Nav2 params (depth is injected at runtime) | `~/rover_nav/` |
| `calib/` | sim→real motion calibration tools (needs battery to run) | `~/rover_nav/` |
| `ugv_ws_overlay/` | the `ugv_driver.py` edit + a `.patch` | apply into `~/ugv_ws` |
| `orbslam_ws_src/orbslam3_ros2_wrapper/` | ORB-SLAM3 ROS 2 wrapper source | `~/orbslam_ws/src/` |

## Prerequisites on the new rover

1. ROS 2 Humble installed (`/opt/ros/humble`).
2. The Waveshare workspace cloned to **`~/ugv_ws`**:
   ```bash
   git clone -b ros2-humble-develop https://github.com/waveshareteam/ugv_ws.git ~/ugv_ws
   ```
3. This bundle placed at **`~/rover_nav`** (the scripts hardcode that path):
   ```bash
   # e.g. from a clone of the isaac-assets repo:
   cp -r isaac-assets/rover_nav ~/rover_nav
   chmod +x ~/rover_nav/*.sh
   ```

## Apply the `ugv_ws` source edit

The only meaningful source change to the fork is `ugv_driver.py` (hardcodes the Jetson
serial port via `UGV_SERIAL_PORT`, defaults `/dev/ttyTHS1`, and shuts down cleanly):

```bash
cd ~/ugv_ws && git apply ~/rover_nav/ugv_ws_overlay/ugv_driver.patch
# or just copy the file:
# cp ~/rover_nav/ugv_ws_overlay/src/ugv_main/ugv_bringup/ugv_bringup/ugv_driver.py \
#    ~/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/ugv_driver.py
```

(The Nav2 params are NOT patched into the tree — the depth-cloud config is injected at
runtime by `inject_depth.py` from `param_backups/amcl_dwa.yaml.orig`.)

## sudo password (parameterized)

The install scripts run detached, so they can't prompt. On the old rover the sudo
password was hardcoded inline; that has been **removed**. Set it once per session
before running the `0x` scripts:

```bash
export ROVER_SUDO_PW='<new rover sudo password>'   # do NOT commit this
```

If you leave `ROVER_SUDO_PW` unset, `rsudo` falls back to normal interactive `sudo`
(run `sudo -v` first to cache credentials for the detached builds).

## Install order  ⚠️ NOT strictly numeric

`nvidia-jetpack` (step 05) supplies the CUDA runtime **and** the OpenCV that ORB-SLAM3
links against, so it must come **before** the ORB-SLAM3 library build. The working order is:

```bash
cd ~/rover_nav
export ROVER_SUDO_PW='...'
./00_apt_deps.sh          # ROS apt deps (robot_localization, depthai, rtabmap, image_proc...)
./01_build.sh             # colcon build the FULL ~/ugv_ws workspace  (was previously incomplete)
./02_download_orbslam.sh  # clone ORB_SLAM3 + Pangolin, unpack ORBvoc.txt
./03_install_cuvslam.sh   # apt install Isaac ROS visual-slam + nvblox
./05_install_jetpack.sh   # nvidia-jetpack (CUDA + VPI + OpenCV)  — BEFORE building ORB-SLAM3
./04_build_orbslam.sh     # build Pangolin + deps (its ORB-SLAM3 build succeeds now that 05 ran)
./06_build_orbslam_lib.sh # (re)build libORB_SLAM3.so  — idempotent with 04
./07_build_wrapper.sh     # colcon build orbslam3_ros2_wrapper against the lib
```

> `04` and `06` overlap (both build `libORB_SLAM3.so`); `04` also builds Pangolin and
> installs apt deps. Running `05` then `04` then `06` is safe and idempotent.
>
> **OOM:** ORB-SLAM3's `build.sh` defaults to all cores and OOM-kills compiling
> `Optimizer.cc` on ~7.4 GB RAM. Both `04`/`06` now pin `make -j2`
> (override with `ORB_SLAM3_JOBS` if you add swap).

Before step 07, put the wrapper source in place:
```bash
mkdir -p ~/orbslam_ws/src
cp -r ~/rover_nav/orbslam_ws_src/orbslam3_ros2_wrapper ~/orbslam_ws/src/
```

Each script drops a `logs/<name>.done` marker so you can tell what finished.
Long builds must survive SSH drops — run them under `setsid nohup ... &`.

## Shell env  ⚠️ fix stale `.bashrc`

The nav scripts always source `local_ros_env.sh` (ROS + `~/ugv_ws/install`,
`ROS_DOMAIN_ID=0`, `ROS_LOCALHOST_ONLY=1`, `ROS_DISCOVERY_SERVER` **unset**,
`UGV_MODEL=ugv_rover`, `LDLIDAR_MODEL=ld19`), so they're self-contained.

But the old rover's `.bashrc` still exported `ROS_DISCOVERY_SERVER=10.168.168.240:11811`
and `ROS_LOCALHOST_ONLY=0` (a retired desktop-link model). **Do not copy that.** On the
new rover, make the default shell local-only so bare `ros2` CLI doesn't hang chasing a
dead server:

```bash
# ~/.bashrc  (recommended for local-only operation)
source /opt/ros/humble/setup.bash
[ -f ~/ugv_ws/install/setup.bash ] && source ~/ugv_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER
export UGV_MODEL=ugv_rover
export LDLIDAR_MODEL=ld19
```

## Running the nav stacks

Each in its own SSH terminal. `start_bringup.sh` first, always.

```bash
~/rover_nav/start_bringup.sh        # base motors + IMU + wheel odom + LD19 + rf2o + TF  (run first)
~/rover_nav/drive_keyboard.sh       # keyboard teleop -> /cmd_vel  (keep window focused)

# --- 2D lidar navigation ---
~/rover_nav/start_lidar_nav.sh      # slam_toolbox + Nav2 (dwa)

# --- lidar nav + OAK depth obstacle cloud ---
~/rover_nav/start_oak.sh            # OAK-D RGB-D + IMU (+ USB chmod watcher)
~/rover_nav/start_depth_nav.sh      # injects depth_cloud into the costmap, then Nav2

# --- RTAB-Map RGB-D + lidar SLAM ---
~/rover_nav/start_rtab.sh           # rtabmap -> /map + map->odom (headless)
~/rover_nav/start_rtab_navigation.sh

# --- ORB-SLAM3 RGB-D VIO ---
~/rover_nav/start_orbslam.sh        # drive through textured scenery to init tracking

# --- cuVSLAM stereo VIO ---
~/rover_nav/start_cuvslam.sh        # OAK stereo (oak_stereo.yaml) rectified -> cuVSLAM

# goals without RViz, over SSH:
python3 ~/rover_nav/send_goal.py --forward 1.0        # relative
python3 ~/rover_nav/send_goal.py --x 1.5 --y 0.0      # absolute

~/rover_nav/stop_all.sh             # kill everything these scripts started
```

## Known issues / gotchas

- **Battery:** the current rover crashes on battery (failing UPS pack) — motors only
  drive on battery, so validate motion once the new rover has a good pack. On wall power
  the base IMU is alive but wheel `/odom` reads flat when stationary.
- **OAK USB permissions:** the depthai udev rule doesn't survive the camera's on-the-fly
  re-enumeration, so `start_oak.sh` / `_oak_watcher.sh` run a root `chmod 666` loop on the
  `03e7` USB nodes. This needs sudo (via `ROVER_SUDO_PW` or cached `sudo -v`).
- **cuVSLAM / ORB-SLAM3 are the weaker stacks** (marginal in sim testing; lidar/depth/rtab
  are the reliable ones). Validate visual tracking on an actual drive before relying on them.
- **`inject_depth.py` round-trips `amcl_dwa.yaml`** through PyYAML (strips comments/quotes) —
  cosmetic; the pristine copy is `param_backups/amcl_dwa.yaml.orig` and `restore_params.sh`
  reverts it.

## Changes made vs. the raw copy on the old rover

- Removed the hardcoded sudo password (`shen25`) from all scripts → `rsudo` + `ROVER_SUDO_PW`.
- `01_build.sh` now builds the **full** workspace (was an incomplete `--packages-select`).
- `04`/`06` pin `make -j2` for the ORB-SLAM3 build to avoid the OOM kill.
- Runtime `logs/` and `__pycache__/` were not copied.
