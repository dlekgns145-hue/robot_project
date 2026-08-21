#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
for setup_file in \
    /root/yahboomcar_ws/install/setup.bash \
    /root/yahboomcar_ros2_ws/install/setup.bash \
    /root/gmapping_ws/install/setup.bash; do
    if [[ -f "${setup_file}" ]]; then
        source "${setup_file}"
    fi
done

exec python3 /opt/robot-control/apple/apple_detection_node.py
