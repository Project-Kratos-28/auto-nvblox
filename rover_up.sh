#!/bin/bash
# Bring up nvblox on the rover, from an SSH session over the WLAN link:
#
#     ssh bitsauto@<rover> ~/kratos_nvblox/rover_up.sh camera:=zed2i
#
# Everything is started detached from the SSH session, so a radio dropout or closing the
# laptop does not stop the rover's mapping:
#   1. reattach the ZED's HID interface if a previous run left it detached
#   2. start the container if it is not running (in its own session, not tied to this SSH)
#   3. start kratos_nvblox_bringup inside it in the background, logging to logs/
#   4. wait until the costmap is up, then print a health summary and the Foxglove address
#
# Args are passed to bringup.launch.py (camera:=d435i|zed2i, depth:=ess|native, ...).
# Stop with rover_down.sh; check with rover_status.sh.
set -u
HERE="$(dirname "$(readlink -f "$0")")"
CONTAINER=isaac_ros_dev_container
LOGDIR="$HERE/logs"
mkdir -p "$LOGDIR"
[ $# -eq 0 ] && { echo "usage: $0 camera:=zed2i|d435i [depth:=ess|native] [other launch args]"; exit 2; }

"$HERE/scripts/zed_hid_rebind.sh"

started=0
if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "starting container..."
    started=1
    rm -f "$LOGDIR/container.log"
    # isaac-ros activate needs a terminal: give it one with `script`, and setsid so it is not in
    # this SSH session's process group (no SIGHUP when the session ends).
    setsid script -qfc "$HERE/start.sh" "$LOGDIR/container.log" </dev/null >/dev/null 2>&1 &
    for _ in $(seq 60); do docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" && break; sleep 2; done
    docker ps --format '{{.Names}}' | grep -qx "$CONTAINER" || { echo "container did not start; see $LOGDIR/container.log"; exit 1; }
fi
# The container's entrypoint creates the admin user and adds it to video/render after the
# container is up. A process started before that keeps the old group list for good (and so
# does everything it respawns): without render, CUDA fails with error 801 in ZED and ESS.
for _ in $(seq 60); do
    docker exec "$CONTAINER" id -nG admin 2>/dev/null | grep -qw render \
        && { [ $started = 0 ] || grep -q 'admin@' "$LOGDIR/container.log" 2>/dev/null; } && break
    sleep 1
done
docker exec "$CONTAINER" id -nG admin 2>/dev/null | grep -qw render \
    || { echo "container user setup did not finish; see $LOGDIR/container.log"; exit 1; }

if pgrep -f '^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch kratos_nvblox_bringup ' >/dev/null; then
    echo "nvblox is already running (rover_status.sh to check, rover_down.sh to stop)"
    exit 0
fi

LOG="$LOGDIR/nvblox_$(date +%Y%m%d_%H%M%S).log"
ln -sfn "$(basename "$LOG")" "$LOGDIR/latest.log"
echo "bringup: $* (log: $LOG)"
docker exec -d -u admin "$CONTAINER" bash -c \
    'exec /workspaces/kratos_nvblox/scripts/run_nvblox.sh "$@" > "/workspaces/kratos_nvblox/logs/$0" 2>&1' \
    "$(basename "$LOG")" "$@"

# Wait for the costmap (it activates once odometry TF exists). The first ZED run after an SDK
# change spends several minutes optimizing its depth model; show that progress.
echo -n "waiting for costmap"
for _ in $(seq 900); do
    grep -q "Managed nodes are active" "$LOG" 2>/dev/null && { echo " - up"; break; }
    if grep -qE "Caught exception|Traceback" "$LOG" 2>/dev/null; then
        echo; echo "launch failed:"; grep -m3 -E "Caught exception|Error|ERROR" "$LOG"; exit 1
    fi
    opt=$(grep -o "Optimizing model: [a-z_0-9]* Progress: [0-9.]*%" "$LOG" 2>/dev/null | tail -1)
    [ -n "$opt" ] && echo -ne "\r$opt          " || echo -n "."
    sleep 1
done
grep -q "Managed nodes are active" "$LOG" || { echo; echo "costmap not up after 15 min; see $LOG"; exit 1; }
sleep 5
"$HERE/rover_status.sh"
