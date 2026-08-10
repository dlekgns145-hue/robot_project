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
    camera-safety)
        exec python3 /opt/robot-control/navigation/camera_obstacle_guard.py \
            --ros-args \
            -p camera_url:="${ROBOT_CAMERA_URL:-http://127.0.0.1:8080/stream.mjpg}"
        ;;
    mapping|mapping-server)
        camera_guard_enabled="${ROBOT_CAMERA_GUARD_ENABLED:-true}"
        if [[ "${1}" == "mapping-server" ]]; then
            camera_guard_enabled="false"
        fi
        exec ros2 launch /opt/robot-control/navigation/mapping_runtime_launch.py \
            map_output:="${ROBOT_MAP_OUTPUT:-/opt/robot-control/maps/orchard_map}" \
            camera_url:="${ROBOT_CAMERA_URL:-http://127.0.0.1:8080/stream.mjpg}" \
            camera_guard_enabled:="${camera_guard_enabled}" \
            texture_source_top_fraction:="${ROBOT_TEXTURE_SOURCE_TOP_FRACTION:-0.50}" \
            texture_near_m:="${ROBOT_TEXTURE_NEAR_M:-0.18}" \
            texture_far_m:="${ROBOT_TEXTURE_FAR_M:-2.0}" \
            texture_near_width_m:="${ROBOT_TEXTURE_NEAR_WIDTH_M:-0.85}" \
            texture_far_width_m:="${ROBOT_TEXTURE_FAR_WIDTH_M:-1.8}" \
            mapping_maximum_runtime:="${ROBOT_MAPPING_MAXIMUM_RUNTIME:-900.0}" \
            mapping_maximum_radius:="${ROBOT_MAPPING_MAXIMUM_RADIUS:-8.0}"
        ;;
    navigation)
        map_yaml="${ROBOT_MAP_YAML:-/opt/robot-control/maps/orchard_map.yaml}"
        params_template="/opt/robot-control/navigation/dwb_nav_params_fixed.yaml"
        runtime_params="/tmp/dwb_nav_params_runtime.yaml"
        if [[ ! -r "${map_yaml}" ]]; then
            echo "saved navigation map is not readable: ${map_yaml}" >&2
            exit 1
        fi
        python3 /opt/robot-control/navigation/prepare_navigation_params.py \
            --template "${params_template}" \
            --pose /opt/robot-control/maps/last_pose.json \
            --output "${runtime_params}"
        exec ros2 launch /opt/robot-control/navigation/navigation_runtime_launch.py \
            map:="${map_yaml}" \
            params_file:="${runtime_params}"
        ;;
    *)
        echo "usage: entrypoint.sh {bringup|base|bridge|camera|camera-safety|mapping|mapping-server|navigation}" >&2
        exit 2
        ;;
esac
