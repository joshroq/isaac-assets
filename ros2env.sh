# ROS 2 environment for the Isaac Sim + Nav2 rover.
# Source this (do NOT execute it) so the exports affect your current shell:
#     source ~/projects/isaac-assets/ros2env.sh
# Or just type:  rosenv   (alias added to ~/.bashrc)
#
# These match what Isaac Sim publishes, so the two sides discover each other.

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export UGV_MODEL=ugv_rover

echo "[ros2env] sourced ROS 2 humble | ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW=$RMW_IMPLEMENTATION  UGV_MODEL=$UGV_MODEL"
