# Local-only ROS 2 env for the rover. Everything runs on the Jetson; no desktop link.
source /opt/ros/humble/setup.bash
[ -f "$HOME/ugv_ws/install/setup.bash" ] && source "$HOME/ugv_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
# --- Waveshare model selection (copied from .bashrc) ---
export UGV_MODEL=ugv_rover
export LDLIDAR_MODEL=ld19
