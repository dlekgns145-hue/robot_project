#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="${VERSION:-$(date +%Y%m%d-%H%M%S)}"
OUTPUT_DIR="${OUTPUT_DIR:-${PROJECT_DIR}/dist}"
STAGING_DIR="$(mktemp -d)"
IMAGE_NAME="robot-control-server-${VERSION}"

cleanup() {
    rm -rf "${STAGING_DIR}"
}
trap cleanup EXIT

install -d "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/docker" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/zenoh" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/robot_app" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/scripts" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/systemd" \
    "${STAGING_DIR}/${IMAGE_NAME}/robot_docker" \
    "${STAGING_DIR}/${IMAGE_NAME}/orchard_mapper"
cp -a "${PROJECT_DIR}/server_image/INSTALL_SERVER.sh" \
    "${STAGING_DIR}/${IMAGE_NAME}/INSTALL_SERVER.sh"
cp -a "${PROJECT_DIR}/server_image/UNINSTALL_SERVER.sh" \
    "${STAGING_DIR}/${IMAGE_NAME}/UNINSTALL_SERVER.sh"
cp -a "${PROJECT_DIR}/server_image/README.md" \
    "${STAGING_DIR}/${IMAGE_NAME}/README.md"
cp -a "${PROJECT_DIR}/orchard_mapper/." \
    "${STAGING_DIR}/${IMAGE_NAME}/orchard_mapper/"
cp -a "${PROJECT_DIR}/ubuntu_v2/docker/." \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/docker/"
cp -a "${PROJECT_DIR}/ubuntu_v2/zenoh/." \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/zenoh/"
cp -a \
    "${PROJECT_DIR}/ubuntu_v2/robot_app/config.py" \
    "${PROJECT_DIR}/ubuntu_v2/robot_app/robot_locator.py" \
    "${PROJECT_DIR}/ubuntu_v2/robot_app/operations_store.py" \
    "${PROJECT_DIR}/ubuntu_v2/robot_app/map_payload.py" \
    "${PROJECT_DIR}/ubuntu_v2/robot_app/robot_gateway.py" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/robot_app/"
cp -a \
    "${PROJECT_DIR}/ubuntu_v2/scripts/run_compute_mapping.sh" \
    "${PROJECT_DIR}/ubuntu_v2/scripts/check_server_environment.sh" \
    "${PROJECT_DIR}/ubuntu_v2/scripts/configure_camera_source.sh" \
    "${PROJECT_DIR}/ubuntu_v2/scripts/phone_camera_layer_test.py" \
    "${PROJECT_DIR}/ubuntu_v2/scripts/run_phone_camera_layer_test.sh" \
    "${PROJECT_DIR}/ubuntu_v2/scripts/phone_scene_capture.py" \
    "${PROJECT_DIR}/ubuntu_v2/scripts/manage_phone_scene_capture.sh" \
    "${PROJECT_DIR}/ubuntu_v2/scripts/analyze_phone_scene_capture.py" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/scripts/"
cp -a "${PROJECT_DIR}/ubuntu_v2/systemd/robot-control-server.service" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/systemd/"
cp -a "${PROJECT_DIR}/ubuntu_v2/compose.yaml" \
    "${PROJECT_DIR}/ubuntu_v2/compose.humble-transport.override.yaml" \
    "${PROJECT_DIR}/ubuntu_v2/.env.example" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/"
install -d "${STAGING_DIR}/${IMAGE_NAME}/robot_docker/recovered"
cp -a \
    "${PROJECT_DIR}/robot_docker/entrypoint.sh" \
    "${PROJECT_DIR}/robot_docker/camera_stream_server.py" \
    "${PROJECT_DIR}/robot_docker/robot_cmd_bridge.py" \
    "${PROJECT_DIR}/robot_docker/navigation_runtime_launch.py" \
    "${PROJECT_DIR}/robot_docker/mapping_runtime_launch.py" \
    "${PROJECT_DIR}/robot_docker/navigation_probe.py" \
    "${PROJECT_DIR}/robot_docker/navigation_run_test.py" \
    "${PROJECT_DIR}/robot_docker/publish_initial_pose.py" \
    "${PROJECT_DIR}/robot_docker/prepare_navigation_params.py" \
    "${PROJECT_DIR}/robot_docker/scan_diagnostics.py" \
    "${PROJECT_DIR}/robot_docker/odom_relay.py" \
    "${PROJECT_DIR}/robot_docker/scan_time_fix.py" \
    "${PROJECT_DIR}/robot_docker/autonomous_mapping.py" \
    "${PROJECT_DIR}/robot_docker/frontier_core.py" \
    "${PROJECT_DIR}/robot_docker/map_texture_core.py" \
    "${PROJECT_DIR}/robot_docker/obstacle_texture_fusion.py" \
    "${PROJECT_DIR}/robot_docker/map_texture_recorder.py" \
    "${PROJECT_DIR}/robot_docker/calibrate_map_texture.py" \
    "${PROJECT_DIR}/robot_docker/camera_obstacle_guard.py" \
    "${PROJECT_DIR}/robot_docker/mapping_slam_params.yaml" \
    "${STAGING_DIR}/${IMAGE_NAME}/robot_docker/"
cp -a \
    "${PROJECT_DIR}/robot_docker/recovered/Broom.yaml" \
    "${PROJECT_DIR}/robot_docker/recovered/Broom.pgm" \
    "${PROJECT_DIR}/robot_docker/recovered/dwb_nav_params_fixed.yaml" \
    "${STAGING_DIR}/${IMAGE_NAME}/robot_docker/recovered/"
install -d "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/data" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/maps"
if [[ -f "${PROJECT_DIR}/images/robot-control-server-images.tar" ]]; then
    install -d "${STAGING_DIR}/${IMAGE_NAME}/images"
    cp -a "${PROJECT_DIR}/images/robot-control-server-images.tar" \
        "${STAGING_DIR}/${IMAGE_NAME}/images/"
fi
chmod 0755 "${STAGING_DIR}/${IMAGE_NAME}/INSTALL_SERVER.sh" \
    "${STAGING_DIR}/${IMAGE_NAME}/UNINSTALL_SERVER.sh" \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/scripts/"*.sh \
    "${STAGING_DIR}/${IMAGE_NAME}/ubuntu_v2/scripts/"*.py

install -d "${OUTPUT_DIR}"
COPYFILE_DISABLE=1 tar --no-xattrs --no-mac-metadata \
    -C "${STAGING_DIR}" \
    -czf "${OUTPUT_DIR}/${IMAGE_NAME}.tar.gz" \
    "${IMAGE_NAME}"

if command -v shasum >/dev/null 2>&1; then
    CHECKSUM="$(shasum -a 256 "${OUTPUT_DIR}/${IMAGE_NAME}.tar.gz" | awk '{print $1}')"
else
    CHECKSUM="$(sha256sum "${OUTPUT_DIR}/${IMAGE_NAME}.tar.gz" | awk '{print $1}')"
fi
printf '%s  %s\n' "${CHECKSUM}" "${IMAGE_NAME}.tar.gz" \
    > "${OUTPUT_DIR}/${IMAGE_NAME}.tar.gz.sha256"

echo "Created ${OUTPUT_DIR}/${IMAGE_NAME}.tar.gz"
echo "Checksum ${OUTPUT_DIR}/${IMAGE_NAME}.tar.gz.sha256"
