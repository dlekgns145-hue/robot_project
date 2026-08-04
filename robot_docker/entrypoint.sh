#!/usr/bin/env bash
# ROS 2 setup scripts intentionally probe optional variables that may be unset.
# Enabling `set -u` before sourcing them makes every runtime container restart.
set -eo pipefail

source /opt/ros/humble/setup.bash
if [[ -f /root/yahboomcar_ros2_ws/install/setup.bash ]]; then
    source /root/yahboomcar_ros2_ws/install/setup.bash
fi

case "${1:-}" in
    base)
        exec ros2 run yahboomcar_base_node base_node_X3
        ;;
    bridge)
        exec python3 /opt/robot-control/robot_cmd_bridge.py
        ;;
    camera)
        exec python3 /opt/robot-control/camera_stream_server.py
        ;;
    *)
        echo "usage: entrypoint.sh {base|bridge|camera}" >&2
        exit 2
        ;;
esac
