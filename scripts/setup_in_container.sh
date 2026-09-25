#!/bin/bash
# Run INSIDE the Isaac ROS container (after `isaac-ros activate`).
# Installs nvblox + Nav2 from the Isaac ROS apt repo, builds realsense_splitter from
# source (it is not shipped as a Debian package) and sets up the ESS venv. Safe to re-run.
set -e
sudo apt-get update
sudo apt-get install -y \
    ros-jazzy-isaac-ros-nvblox \
    ros-jazzy-nvblox-examples-bringup \
    ros-jazzy-nvblox-nav2 \
    ros-jazzy-isaac-ros-visual-slam \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rviz2 \
    libopenblas0 python3-venv          # torch runtime deps for the ESS node
source /opt/ros/jazzy/setup.bash
/workspaces/kratos_nvblox/scripts/build_ws.sh

# ESS depth node: torch (Jetson wheel, CUDA 13.2) in a venv that also sees the system's
# ROS + TensorRT Python packages. It lives in the mounted folder, so it survives --rm.
ESS=/workspaces/kratos_nvblox/ess
if ! "$ESS/.venv/bin/python" -c "import torch" 2>/dev/null; then
    python3 -m venv --system-site-packages "$ESS/.venv"
    "$ESS/.venv/bin/pip" install -q "$ESS"/wheels/torch-*.whl "numpy==1.26.4"
fi
echo "Done. Now: source /workspaces/isaac_ros-dev/install/setup.bash"
