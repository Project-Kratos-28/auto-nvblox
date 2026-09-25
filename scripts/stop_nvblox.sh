#!/bin/bash
# Stop the pipeline started by run_nvblox.sh. Works on the host or in the container.
# The container runs with --pid=host, so `pkill -f <pattern>` also matches unrelated
# shells whose command line contains the pattern; match exact process names instead.
names='^(component_conta|nav2_costmap_2d|lifecycle_manag|rviz2)$'
launch=$(pgrep -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch (kratos_nvblox_bringup |/workspaces/kratos_nvblox/)')
[ -n "$launch" ] && sudo kill -INT $launch
for _ in $(seq 20); do pgrep -f 'ess/ess_stereo_node\.py' >/dev/null || pgrep "$names" >/dev/null || break; sleep 1; done
left=$(pgrep "$names"; pgrep -f '^/workspaces/kratos_nvblox/ess/\.venv/bin/python ')
[ -n "$left" ] && { echo "force-stopping: $left"; sudo kill -9 $left; }
# Force-killed processes leave Fast DDS shared-memory locks behind; stale ones crash
# later processes (glibc robust-mutex assertion), so remove them.
docker exec -u admin isaac_ros_dev_container bash -c 'source /opt/ros/jazzy/setup.bash; fastdds shm clean' 2>/dev/null \
  || { source /opt/ros/jazzy/setup.bash; fastdds shm clean; }
"$(dirname "$(readlink -f "$0")")/zed_hid_rebind.sh"   # ZED 2i HID sensor interface
echo "stopped"
