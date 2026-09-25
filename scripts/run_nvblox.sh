#!/bin/bash
# Run INSIDE the container: kratos_nvblox_bringup (camera + depth + odometry + nvblox + Nav2 costmap).
# Args are forwarded, e.g.  camera:=zed2i  depth:=native  run_rviz:=False
source /opt/ros/jazzy/setup.bash
# Everything runs on this Orin, so keep DDS discovery on loopback. With the default
# (subnet) discovery, a Wi-Fi drop/reconnect leaves running nodes undiscoverable.
# Any other terminal that should see these topics needs the same setting.
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
# ZED wrapper, built into the kratos image (docker/Dockerfile.kratos)
[ -f /opt/zed_ws/install/setup.bash ] && source /opt/zed_ws/install/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
exec ros2 launch kratos_nvblox_bringup bringup.launch.py "$@"
