#!/bin/bash
# Stop the nvblox bringup (the container keeps running; --all also stops the container).
HERE="$(dirname "$(readlink -f "$0")")"
"$HERE/scripts/stop_nvblox.sh"
if [ "${1:-}" = "--all" ]; then
    docker stop isaac_ros_dev_container >/dev/null 2>&1 && echo "container stopped"
    "$HERE/scripts/zed_hid_rebind.sh"
fi
