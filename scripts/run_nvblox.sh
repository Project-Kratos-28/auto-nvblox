#!/bin/bash
# Run INSIDE the container: kratos_nvblox_bringup (camera + depth + odometry + nvblox + Nav2 costmap).
# Args are forwarded, e.g.  camera:=zed2i  depth:=native  run_rviz:=False
source /opt/ros/jazzy/setup.bash
# Default Fast DDS discovery (SUBNET, domain 0): other machines on the network see the topics.
# ZED wrapper, built into the kratos image (docker/Dockerfile.kratos)
[ -f /opt/zed_ws/install/setup.bash ] && source /opt/zed_ws/install/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
exec ros2 launch kratos_nvblox_bringup bringup.launch.py "$@"
