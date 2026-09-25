#!/bin/bash
# Run INSIDE the container: build this project's ROS packages into the Isaac ROS workspace
# (/workspaces/isaac_ros-dev, mounted from the host, so builds survive container restarts).
#   realsense_splitter     NVIDIA's, from src/isaac_ros_nvblox (not shipped as a Debian package)
#   kratos_nvblox_bringup  this project's launch/config/rviz
set -e
source /opt/ros/jazzy/setup.bash
[ -f /opt/zed_ws/install/setup.bash ] && source /opt/zed_ws/install/setup.bash
cd /workspaces/isaac_ros-dev
colcon build --symlink-install \
    --base-paths src/isaac_ros_nvblox/nvblox_examples/realsense_splitter /workspaces/kratos_nvblox/kratos_nvblox_bringup \
    --packages-select realsense_splitter kratos_nvblox_bringup "$@"
