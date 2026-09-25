#!/bin/bash
# Host: build the kratos image on top of NVIDIA's Isaac ROS realsense image and tag it as the
# image `isaac-ros activate --use-cached-build-image` runs (start.sh passes that flag).
set -e
cd "$(dirname "$0")"
BASE=$(docker images --format '{{.Repository}}:{{.Tag}}' | grep -m1 'nvcr.io/nvidia/isaac/ros/isaac_ros-realsense_.*-arm64-jetpack')
[ -n "$BASE" ] || { echo "Isaac ROS realsense image not found; run: isaac-ros activate --build-local --build-only"; exit 1; }
echo "base: $BASE"
docker build -f Dockerfile.kratos --build-arg BASE_IMAGE="$BASE" -t kratos/isaac_ros_nvblox:latest .
docker tag kratos/isaac_ros_nvblox:latest cached_isaac_run_dev_image_local:latest
echo "built kratos/isaac_ros_nvblox:latest (also tagged cached_isaac_run_dev_image_local:latest)"
