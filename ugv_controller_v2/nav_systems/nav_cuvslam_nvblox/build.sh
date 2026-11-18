#!/usr/bin/env bash
# One-time setup for the v2 nav_cuvslam_nvblox system.
#
# Runtime is still just:
#   navlaunch -cuvslam
#
# This script prepares the Isaac ROS packages that navlaunch needs to find:
#   isaac_ros_visual_slam  (cuVSLAM)
#   nvblox_ros             (nvblox node)
#   nvblox_nav2            (Nav2 costmap plugin)
#
# Preferred Humble path, when NVIDIA's Isaac ROS apt repo is configured:
#   ./build.sh --debs
#
# Source path:
#   ./build.sh --source
#
# The source path creates a separate Isaac ROS colcon workspace. For ROS Humble it
# defaults to NVIDIA's Humble-era Isaac ROS branch, because current Isaac ROS 4.5
# source trees are Jazzy-era and should not be dropped into this Humble rover stack.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="${SOURCE_ROOT:-/home/joshroq/projects/slam_source}"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-$HOME/workspaces/isaac_ros-dev}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ "$ROS_DISTRO" == "humble" ]]; then
  ISAAC_ROS_RELEASE="${ISAAC_ROS_RELEASE:-release-3.2}"
else
  ISAAC_ROS_RELEASE="${ISAAC_ROS_RELEASE:-release-4.5}"
fi

# NVIDIA Isaac ROS apt repository (verified for Humble/jammy on 2026-07-16).
# NOTE: NVIDIA's *current* apt repo (release-4.5) is Jazzy / Ubuntu 24.04 only.
# Humble Debian packages live in the older release-3 repo (dist jammy, component
# release-3.0), which still serves ros-humble-isaac-ros-visual-slam,
# ros-humble-isaac-ros-nvblox, ros-humble-nvblox-nav2, etc.
ISAAC_APT_KEY_URL="${ISAAC_APT_KEY_URL:-https://isaac.download.nvidia.com/isaac-ros/repos.key}"
ISAAC_APT_KEYRING="${ISAAC_APT_KEYRING:-/usr/share/keyrings/nvidia-isaac-ros.gpg}"
ISAAC_APT_LIST="${ISAAC_APT_LIST:-/etc/apt/sources.list.d/nvidia-isaac-ros.list}"
if [[ "$ROS_DISTRO" == "humble" ]]; then
  ISAAC_APT_REPO_LINE="${ISAAC_APT_REPO_LINE:-deb [signed-by=${ISAAC_APT_KEYRING}] https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0}"
else
  ISAAC_APT_REPO_LINE="${ISAAC_APT_REPO_LINE:-deb [signed-by=${ISAAC_APT_KEYRING}] https://isaac.download.nvidia.com/isaac-ros/release-4.5 noble main}"
fi

# --- Runtime (CUDA + VPI) libraries the Isaac ROS debs dlopen but do NOT declare ---
# The release-3 debs assume NVIDIA's container base image already provides a CUDA 12
# runtime and VPI. On a bare host, cuVSLAM (libvisual_slam_node.so) and nvblox
# (libnvblox_ros_lib.so) fail to load with e.g. "libcudart.so.12: cannot open shared
# object file". `ldd` on those .so's shows the full need:
#   libcudart.so.12, libcublas.so.12, libcusolver.so.11, libnpp*.so.12  (CUDA 12.x)
#   libnvvpi.so.3                                                        (VPI 3)
# Isaac ROS 3.2 targets CUDA 12.6 + VPI 3.2. We install LIBRARY packages only (never a
# cuda-drivers / cuda-runtime metapackage) so the host's existing NVIDIA driver is left
# untouched. Verified 2026-07-16: both repos + keys reachable, all package names present.
CUDA_APT_KEYRING_DEB="${CUDA_APT_KEYRING_DEB:-https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb}"
CUDA_LIB_VER="${CUDA_LIB_VER:-12-6}"        # CUDA 12.x runtime; SONAMEs are stable across 12.x
VPI_APT_KEY_URL="${VPI_APT_KEY_URL:-https://repo.download.nvidia.com/jetson/jetson-ota-public.asc}"
VPI_APT_KEYRING="${VPI_APT_KEYRING:-/usr/share/keyrings/nvidia-vpi.gpg}"
VPI_APT_LIST="${VPI_APT_LIST:-/etc/apt/sources.list.d/nvidia-vpi.list}"
# libnvvpi3 3.2.4 lives in the jetson x86_64 repo under dist r36.4 (matches Isaac ROS 3.2).
VPI_APT_REPO_LINE="${VPI_APT_REPO_LINE:-deb [signed-by=${VPI_APT_KEYRING}] https://repo.download.nvidia.com/jetson/x86_64/jammy r36.4 main}"

usage() {
  cat <<EOF
Usage:
  $0 --debs          Install prebuilt Isaac ROS Humble Debian packages.
  $0 --runtime-deps  Install the CUDA 12 runtime + VPI 3 libraries the nodes dlopen.
  $0 --source        Create/build an Isaac ROS source workspace.
  $0 --check         Check whether navlaunch can see the required packages.
  $0 --check-runtime Check the CUDA/VPI shared libs the nodes need actually resolve.

Environment:
  SOURCE_ROOT=$SOURCE_ROOT
  ISAAC_ROS_WS=$ISAAC_ROS_WS
  ROS_DISTRO=$ROS_DISTRO
  ISAAC_ROS_RELEASE=$ISAAC_ROS_RELEASE
  USE_LOCAL_SOURCE=${USE_LOCAL_SOURCE:-0}   # 1 = symlink SOURCE_ROOT instead of cloning

After setup:
  export ISAAC_ROS_WS=$ISAAC_ROS_WS
  navlaunch -cuvslam
EOF
}

check_pkgs() {
  set +u
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  if [[ -f "${ISAAC_ROS_WS}/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    set +u
    source "${ISAAC_ROS_WS}/install/setup.bash"
    set -u
  fi

  local missing=()
  for pkg in isaac_ros_visual_slam nvblox_ros nvblox_nav2; do
    if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
      missing+=("$pkg")
    fi
  done

  if (( ${#missing[@]} )); then
    echo "Missing: ${missing[*]}" >&2
    return 1
  fi

  echo "All required cuVSLAM/nvblox packages are visible."
}

configure_apt_repo() {
  # Idempotently install NVIDIA's Isaac ROS apt key + source list.
  if ! command -v gpg >/dev/null 2>&1; then
    echo "Installing gnupg (needed to dearmor the Isaac ROS signing key)..."
    sudo apt-get update
    sudo apt-get install -y gnupg curl
  fi

  if [[ ! -s "$ISAAC_APT_KEYRING" ]]; then
    echo "Installing Isaac ROS apt signing key -> $ISAAC_APT_KEYRING"
    curl -fsSL "$ISAAC_APT_KEY_URL" | sudo gpg --dearmor -o "$ISAAC_APT_KEYRING"
  else
    echo "Isaac ROS apt key already present: $ISAAC_APT_KEYRING"
  fi

  if ! grep -qxF "$ISAAC_APT_REPO_LINE" "$ISAAC_APT_LIST" 2>/dev/null; then
    echo "Adding Isaac ROS apt source -> $ISAAC_APT_LIST"
    echo "$ISAAC_APT_REPO_LINE" | sudo tee "$ISAAC_APT_LIST" >/dev/null
  else
    echo "Isaac ROS apt source already present: $ISAAC_APT_LIST"
  fi
}

check_runtime_libs() {
  # Report any CUDA/VPI shared libs the installed nodes still can't resolve.
  local nodes=(
    /opt/ros/${ROS_DISTRO}/lib/libvisual_slam_node.so
    /opt/ros/${ROS_DISTRO}/lib/libnvblox_ros_lib.so
  )
  local missing
  missing="$(for so in "${nodes[@]}"; do
    [[ -f "$so" ]] && ldd "$so" 2>/dev/null | awk '/not found/{print $1}'
  done | sort -u)"
  if [[ -n "$missing" ]]; then
    echo "Unresolved runtime libraries for cuVSLAM/nvblox:" >&2
    echo "$missing" | sed 's/^/  /' >&2
    return 1
  fi
  echo "All cuVSLAM/nvblox runtime libraries resolve."
}

install_runtime_deps() {
  # Install the CUDA 12.x runtime + VPI 3 libraries the Isaac debs dlopen (libs only).
  echo "Installing CUDA ${CUDA_LIB_VER} runtime + VPI 3 libraries (no driver change)..."
  if ! command -v gpg >/dev/null 2>&1; then
    sudo apt-get update && sudo apt-get install -y gnupg curl
  fi

  # CUDA apt repo (via cuda-keyring), then VPI (jetson x86_64) apt repo.
  local tmp_keyring="/tmp/cuda-keyring.deb"
  if ! dpkg -s cuda-keyring >/dev/null 2>&1; then
    echo "Adding CUDA apt repo (cuda-keyring)..."
    curl -fsSL -o "$tmp_keyring" "$CUDA_APT_KEYRING_DEB"
    sudo dpkg -i "$tmp_keyring"
  else
    echo "cuda-keyring already installed."
  fi

  if [[ ! -s "$VPI_APT_KEYRING" ]]; then
    echo "Adding VPI apt signing key..."
    curl -fsSL "$VPI_APT_KEY_URL" | sudo gpg --dearmor -o "$VPI_APT_KEYRING"
  fi
  if ! grep -qxF "$VPI_APT_REPO_LINE" "$VPI_APT_LIST" 2>/dev/null; then
    echo "Adding VPI apt source -> $VPI_APT_LIST"
    echo "$VPI_APT_REPO_LINE" | sudo tee "$VPI_APT_LIST" >/dev/null
  fi

  sudo apt-get update
  sudo apt-get install -y \
    "cuda-cudart-${CUDA_LIB_VER}" \
    "libcublas-${CUDA_LIB_VER}" \
    "libcusolver-${CUDA_LIB_VER}" \
    "libcusparse-${CUDA_LIB_VER}" \
    "libnpp-${CUDA_LIB_VER}" \
    "libnvjitlink-${CUDA_LIB_VER}" \
    libnvvpi3

  echo
  # CUDA runtime libs land under /usr/local/cuda-<ver>/ (targets/.../lib or lib64),
  # which is not on the default loader path. Register whichever dirs exist so the
  # Isaac nodes can dlopen them. (VPI's deb ships its own ld.so.conf.d entry.)
  local cuda_root="/usr/local/cuda-${CUDA_LIB_VER/-/.}"
  : >/tmp/nvidia-cuda-isaac.conf
  for d in "$cuda_root/targets/x86_64-linux/lib" "$cuda_root/lib64"; do
    [[ -d "$d" ]] && echo "$d" >>/tmp/nvidia-cuda-isaac.conf
  done
  if [[ -s /tmp/nvidia-cuda-isaac.conf ]]; then
    sudo cp /tmp/nvidia-cuda-isaac.conf /etc/ld.so.conf.d/nvidia-cuda-isaac.conf
  fi
  sudo ldconfig

  echo
  check_runtime_libs
}

install_debs() {
  echo "Installing NVIDIA Isaac ROS Humble Debian packages..."
  configure_apt_repo
  sudo apt-get update

  # Deliberately install the nvblox *leaf* packages, NOT the ros-*-isaac-ros-nvblox
  # metapackage. The metapackage pulls nvblox_examples_bringup, which hard-depends on
  # isaac_ros_triton -> gxf_isaac_triton -> libnvidia-compute-560 + TensorRT 10. Those
  # DNN/Triton deps are (a) not needed by cuVSLAM or nvblox mapping and (b) unresolvable
  # on newer NVIDIA drivers (the 560 pin conflicts with 595 on Blackwell GPUs). The
  # launch reads nvblox's config yamls from the source tree fallback, so the bringup
  # package is not required. cuVSLAM (isaac-ros-visual-slam) has no TensorRT dependency.
  local deb_pkgs=(
    "ros-${ROS_DISTRO}-isaac-ros-visual-slam"
    "ros-${ROS_DISTRO}-nvblox-ros"
    "ros-${ROS_DISTRO}-nvblox-nav2"
    "ros-${ROS_DISTRO}-nvblox-rviz-plugin"
  )

  local missing_from_apt=()
  for pkg in "${deb_pkgs[@]}"; do
    if ! apt-cache show "$pkg" >/dev/null 2>&1; then
      missing_from_apt+=("$pkg")
    fi
  done

  if (( ${#missing_from_apt[@]} )); then
    echo
    echo "ERROR: these Isaac ROS Debian packages are not visible to apt:" >&2
    printf '  %s\n' "${missing_from_apt[@]}" >&2
    echo >&2
    echo "The NVIDIA Isaac ROS apt source was configured at:" >&2
    echo "  $ISAAC_APT_LIST" >&2
    echo "  $ISAAC_APT_REPO_LINE" >&2
    echo "but the packages above still are not resolvable. Likely causes:" >&2
    echo "  - ROS_DISTRO ($ROS_DISTRO) / Ubuntu codename mismatch for this repo line" >&2
    echo "  - network/proxy cannot reach isaac.download.nvidia.com" >&2
    echo "Verify with: apt-cache policy ${missing_from_apt[0]}" >&2
    echo "Or fall back to the source/container path:  $0 --source" >&2
    echo >&2
    echo "NVIDIA repo docs:" >&2
    echo "  https://nvidia-isaac-ros.github.io/getting_started/index.html#configure-isaac-ros-apt-repository" >&2
    exit 1
  fi

  sudo apt-get install -y "${deb_pkgs[@]}"

  echo
  check_pkgs
  echo
  # The ROS packages resolving does NOT mean the nodes can load: they dlopen a CUDA 12
  # runtime + VPI 3 that the debs don't declare. Check and point at --runtime-deps.
  if ! check_runtime_libs; then
    echo
    echo "The ROS packages are installed, but cuVSLAM/nvblox need CUDA + VPI runtime" >&2
    echo "libraries that are not yet present. Install them with:" >&2
    echo "  $0 --runtime-deps" >&2
    exit 1
  fi
  echo
  echo "Done. You can run:"
  echo "  navlaunch -cuvslam"
}

link_source_tree() {
  local src="$1"
  local dst="$2"

  if [[ ! -d "$src" ]]; then
    echo "ERROR: source tree not found: $src" >&2
    exit 1
  fi

  if [[ -e "$dst" && ! -L "$dst" ]]; then
    echo "ERROR: destination exists and is not a symlink: $dst" >&2
    exit 1
  fi

  ln -sfn "$src" "$dst"
}

clone_or_update_repo() {
  local repo_url="$1"
  local dst="$2"
  local branch="$3"

  if [[ -L "$dst" ]]; then
    if [[ "${USE_LOCAL_SOURCE:-0}" == "1" ]]; then
      echo "-- Existing local source symlink: $dst"
      return 0
    fi
    echo "-- Replacing old symlink with ${branch} checkout: $dst"
    rm "$dst"
  fi

  if [[ -d "$dst/.git" ]]; then
    local current_branch
    current_branch="$(git -C "$dst" rev-parse --abbrev-ref HEAD)"
    if [[ "$current_branch" == "$branch" ]]; then
      echo "-- Existing repo already on ${branch}: $dst"
      return 0
    fi
    if [[ -n "$(git -C "$dst" status --porcelain)" ]]; then
      echo "ERROR: $dst is on $current_branch, but ${branch} is needed, and it has local changes." >&2
      echo "Commit/stash/remove that checkout, then rerun." >&2
      exit 1
    fi
    echo "-- Switching $dst from ${current_branch} to ${branch}"
    git -C "$dst" fetch origin "$branch"
    git -C "$dst" checkout "$branch"
    return 0
  fi
  if [[ -e "$dst" ]]; then
    echo "ERROR: destination exists and is not a git repo: $dst" >&2
    exit 1
  fi
  git clone -b "$branch" "$repo_url" "$dst"
}

build_source() {
  echo "Creating Isaac ROS workspace at: ${ISAAC_ROS_WS}"
  mkdir -p "${ISAAC_ROS_WS}/src"

  if [[ "${USE_LOCAL_SOURCE:-0}" == "1" ]]; then
    echo "Using local SOURCE_ROOT symlinks. Make sure those checkouts match ${ROS_DISTRO}."
    link_source_tree \
      "${SOURCE_ROOT}/isaac_ros_visual_slam" \
      "${ISAAC_ROS_WS}/src/isaac_ros_visual_slam"
    link_source_tree \
      "${SOURCE_ROOT}/isaac_ros_nvblox" \
      "${ISAAC_ROS_WS}/src/isaac_ros_nvblox"
  else
    clone_or_update_repo \
      "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git" \
      "${ISAAC_ROS_WS}/src/isaac_ros_visual_slam" \
      "${ISAAC_ROS_RELEASE}"
    clone_or_update_repo \
      "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git" \
      "${ISAAC_ROS_WS}/src/isaac_ros_nvblox" \
      "${ISAAC_ROS_RELEASE}"
  fi

  clone_or_update_repo \
    "https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git" \
    "${ISAAC_ROS_WS}/src/isaac_ros_common" \
    "${ISAAC_ROS_RELEASE}"

  echo
  echo "Isaac ROS release branch: ${ISAAC_ROS_RELEASE}"
  echo
  echo "Workspace prepared. For NVIDIA's container flow, run:"
  echo "  sudo apt-get install -y git-lfs docker-buildx"
  echo "  git lfs install"
  echo "  cd ${ISAAC_ROS_WS}/src/isaac_ros_common"
  echo "  ./scripts/run_dev.sh -d ${ISAAC_ROS_WS}"
  echo
  echo "Then inside that environment/container:"
  echo "  cd ${ISAAC_ROS_WS}"
  echo "  sudo apt-get update"
  echo "  rosdep update"
  echo "  rosdep install -i -r --from-paths src/isaac_ros_visual_slam src/isaac_ros_nvblox --rosdistro ${ROS_DISTRO} -y"
  echo "  colcon build --symlink-install --base-paths src/isaac_ros_visual_slam src/isaac_ros_nvblox"
  echo
  echo "After it succeeds, normal runtime is:"
  echo "  export ISAAC_ROS_WS=${ISAAC_ROS_WS}"
  echo "  navlaunch -cuvslam"
}

case "${1:---help}" in
  --debs) install_debs ;;
  --runtime-deps) install_runtime_deps ;;
  --source) build_source ;;
  --check) check_pkgs ;;
  --check-runtime) check_runtime_libs ;;
  -h|--help|help) usage ;;
  *)
    echo "Unknown option: $1" >&2
    echo >&2
    usage >&2
    exit 2
    ;;
esac
