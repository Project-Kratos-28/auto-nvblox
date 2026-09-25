#!/bin/bash
# Health summary for the nvblox bringup, readable over SSH.
HERE="$(dirname "$(readlink -f "$0")")"
CONTAINER=isaac_ros_dev_container

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "container: not running"; exit 1
fi
launch=$(ps -eo args | grep -m1 '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch kratos_nvblox_bringup ')
[ -z "$launch" ] && { echo "container: up, nvblox: not running"; exit 1; }
echo "nvblox: ${launch#*bringup.launch.py}"

camera=$(grep -oE "camera=[a-z0-9]+" "$HERE/logs/latest.log" 2>/dev/null | head -1 | cut -d= -f2)
odom=/zed/zed_node/odom; [ "$camera" = d435i ] && odom=/visual_slam/tracking/odometry
docker exec -u admin "$CONTAINER" bash -c "source /opt/ros/jazzy/setup.bash; \
    python3 /workspaces/kratos_nvblox/scripts/topic_rates.py --seconds 5 \
        $odom /local_costmap/costmap /remote/costmap /remote/color/compressed 2>/dev/null"
ess=$(grep "latency ms" "$HERE/logs/latest.log" 2>/dev/null | tail -1 | grep -oE "latency ms: .*%" )
[ -n "$ess" ] && echo "ESS $ess"
tegrastats --interval 1000 2>/dev/null | head -1 | grep -oE "RAM [0-9]+/[0-9]+MB|GR3D_FREQ [0-9]+%|tj@[0-9.]+C" | tr '\n' ' '; echo
errors=$(grep -cE "process has died|\[ERROR\]" "$HERE/logs/latest.log" 2>/dev/null)
echo "log: $HERE/logs/latest.log ($errors error lines)"
# Base station: default Fast DDS (domain 0, SUBNET discovery) on the same network sees the topics.
for ip in $(hostname -I); do case $ip in 172.*|*:*) ;; *) echo "rover ip: $ip";; esac; done
