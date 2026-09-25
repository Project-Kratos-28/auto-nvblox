#!/bin/bash
# Host: start the ZED depth benchmark container (kratos image, own name so it never attaches to
# kratos' isaac_ros_dev_container). Same device/GPU setup as `isaac-ros activate`.
# ZED AI models (NEURAL*) and camera calibration persist in bench_zed/zed_{resources,settings}:
# the image empties /usr/local/zed/resources, and optimizing a NEURAL model takes minutes.
set -e
NAME=zed_bench
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    exec docker exec -it -u 2002:2002 -e HOME=/workspaces/kratos_nvblox/bench_zed/home "$NAME" bash
fi
B=/home/bitsauto/kratos_nvblox
xhost +local: >/dev/null 2>&1 || true
docker run -d --rm --name "$NAME" \
    --privileged --network host --ipc host --runtime nvidia --gpus all \
    -e DISPLAY="${DISPLAY:-:1}" -e NVIDIA_DRIVER_CAPABILITIES=all -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev/bus/usb:/dev/bus/usb -v /dev/input:/dev/input \
    -v /usr/bin/tegrastats:/usr/bin/tegrastats \
    -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
    -v $B:/workspaces/kratos_nvblox \
    -v $B/bench_zed/zed_resources:/usr/local/zed/resources \
    -v $B/bench_zed/zed_settings:/usr/local/zed/settings \
    --entrypoint /bin/bash kratos/isaac_ros_nvblox:latest -c 'sleep infinity'
echo "started $NAME; attach: $0  (or docker exec -u 2002:2002 -e HOME=/workspaces/kratos_nvblox/bench_zed/home $NAME <cmd>)"
