#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${SCRIPT_DIR}/env/ros2_rover.env"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required at /opt/ros/humble/setup.bash" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-imu-filter-madgwick \
  ros-humble-imu-complementary-filter \
  ros-humble-tf2-tools \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rmw-fastrtps-cpp

if ! rosdep --version >/dev/null 2>&1; then
  echo "rosdep is installed but not available on PATH" >&2
  exit 1
fi

if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then
  sudo rosdep init || true
fi
rosdep update

set -a
source "${ENV_FILE}"
set +a

cd "${REPO_ROOT}/ugv_ws"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble || true
colcon build --symlink-install

sudo install -m 0755 "${SCRIPT_DIR}/scripts/ugv-env" /usr/local/bin/ugv-env
sudo install -m 0755 "${SCRIPT_DIR}/scripts/ugv-bringup" /usr/local/bin/ugv-bringup
sudo install -m 0755 "${SCRIPT_DIR}/scripts/ugv-check" /usr/local/bin/ugv-check
sudo install -m 0755 "${SCRIPT_DIR}/scripts/ugv-cmdvel-test" /usr/local/bin/ugv-cmdvel-test

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT
for unit in ugv-bringup.service ugv-ros-env-check.service; do
  sed \
    -e "s#__ROVER_USER__#${USER}#g" \
    -e "s#__ROVER_HOME__#${HOME}#g" \
    "${SCRIPT_DIR}/systemd/${unit}" > "${tmpdir}/${unit}"
  sudo install -m 0644 "${tmpdir}/${unit}" "/etc/systemd/system/${unit}"
done
sudo systemctl daemon-reload

echo "Installed Jetson rover setup."
echo "Next:"
echo "  sudo systemctl enable --now ugv-ros-env-check.service"
echo "  sudo systemctl enable --now ugv-bringup.service"
