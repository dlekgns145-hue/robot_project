#!/usr/bin/env bash
# ROS 2 setup scripts intentionally probe optional variables that may be unset.
# Enabling `set -u` before sourcing them makes every runtime container restart.
set -eo pipefail

source /opt/ros/humble/setup.bash

# Yahboom image revisions use both workspace names. Source every workspace that
# exists so the official bringup package and its scan/odom relay nodes are
# available regardless of the image layout.
for setup_file in \
    /root/yahboomcar_ws/install/setup.bash \
    /root/yahboomcar_ros2_ws/install/setup.bash \
    /root/gmapping_ws/install/setup.bash; do
    if [[ -f "${setup_file}" ]]; then
        source "${setup_file}"
    fi
done

case "${1:-}" in
    bringup)
        if ros2 pkg prefix yahboomcar_bringup >/dev/null 2>&1; then
            exec ros2 launch yahboomcar_bringup yahboomcar_bringup_launch.py
        fi
        echo "warning: yahboomcar_bringup not found; falling back to base_node_X3" >&2
        exec ros2 run yahboomcar_base_node base_node_X3
        ;;
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
        echo "usage: entrypoint.sh {bringup|base|bridge|camera}" >&2
        exit 2
        ;;
esac
