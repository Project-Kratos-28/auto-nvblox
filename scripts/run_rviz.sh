#!/bin/bash
# Run INSIDE the container: RViz with nvblox displays plus the Nav2 costmap, for a pipeline
# started with run_rviz:=False.
# Host must allow the container on the display first: xhost +local:
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}
# RViz config per camera: CAMERA=zed2i run_rviz.sh (default d435i)
source /workspaces/isaac_ros-dev/install/setup.bash
exec rviz2 -d "$(ros2 pkg prefix kratos_nvblox_bringup)/share/kratos_nvblox_bringup/rviz/${CAMERA:-d435i}.rviz" "$@"
