#!/bin/bash
# Host side: start (or attach to) the Isaac ROS dev container with RealSense support.
# ~/kratos_nvblox is mounted at /workspaces/kratos_nvblox (see ~/.isaac_ros_dev-dockerargs).
export ISAAC_ROS_WS="${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev/}"
xhost +local: >/dev/null 2>&1 || true   # let rviz in the container use the display

"$(dirname "$(readlink -f "$0")")/scripts/zed_hid_rebind.sh"   # ZED 2i HID sensor interface
# Runs the kratos image (docker/build_image.sh tags it as the CLI's cached build image):
# NVIDIA's Isaac ROS + realsense image plus nvblox/Nav2 packages and the ZED SDK.
if docker image inspect cached_isaac_run_dev_image_local:latest >/dev/null 2>&1; then
    exec isaac-ros activate --use-cached-build-image "$@"
fi
echo "kratos image not built yet (docker/build_image.sh); starting NVIDIA's base image"
exec isaac-ros activate "$@"
