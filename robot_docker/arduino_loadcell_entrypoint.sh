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

exec python3 /opt/robot-control/navigation/arduino_loadcell_bridge.py \
    --ros-args \
    -p serial_device:="${ROBOT_LOADCELL_SERIAL_DEVICE:-/dev/loadcell-arduino}" \
    -p baud:="${ROBOT_LOADCELL_SERIAL_BAUD:-9600}" \
    -p low_threshold_g:="${ROBOT_LOADCELL_LOW_THRESHOLD_G:-0.8}" \
    -p high_threshold_g:="${ROBOT_LOADCELL_HIGH_THRESHOLD_G:-2.8}"
