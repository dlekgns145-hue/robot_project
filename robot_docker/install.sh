#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run with sudo: sudo bash install.sh" >&2
    exit 1
fi

DEPLOY_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${DEPLOY_DIR}"

ENV_FILE="${DEPLOY_DIR}/.env"

configured_value() {
    local key="$1"
    [[ -f "${ENV_FILE}" ]] || return 0
    sed -n "s/^${key}=//p" "${ENV_FILE}" | tail -n 1
}

detect_serial_device() {
    local configured
    configured="$(configured_value ROBOT_SERIAL_DEVICE)"
    if [[ -n "${configured}" ]]; then
        printf '%s\n' "${configured}"
        return
    fi

    local candidates=()
    if [[ -d /dev/serial/by-id ]]; then
        while IFS= read -r candidate; do
            candidates+=("${candidate}")
        done < <(
            find /dev/serial/by-id -maxdepth 1 -type l \
                \( -iname '*CP210*' -o -iname '*micro*ros*' \) -print | sort
        )
    fi

    if [[ "${#candidates[@]}" -eq 1 ]]; then
        printf '%s\n' "${candidates[0]}"
    else
        printf '%s\n' "/dev/ttyUSB0"
    fi
}

ROBOT_SERIAL_DEVICE="$(detect_serial_device)"
ROBOT_SERIAL_BAUD="$(configured_value ROBOT_SERIAL_BAUD)"
ROBOT_SERIAL_BAUD="${ROBOT_SERIAL_BAUD:-921600}"

if [[ ! -e "${ROBOT_SERIAL_DEVICE}" ]]; then
    echo "Robot controller serial device not found: ${ROBOT_SERIAL_DEVICE}" >&2
    echo "Connect the MCU, then set ROBOT_SERIAL_DEVICE in ${ENV_FILE}." >&2
    exit 1
fi

if [[ ! -f "${ENV_FILE}" ]]; then
    : >"${ENV_FILE}"
fi
if ! grep -q '^ROBOT_SERIAL_DEVICE=' "${ENV_FILE}"; then
    printf 'ROBOT_SERIAL_DEVICE=%s\n' "${ROBOT_SERIAL_DEVICE}" >>"${ENV_FILE}"
fi
if ! grep -q '^ROBOT_SERIAL_BAUD=' "${ENV_FILE}"; then
    printf 'ROBOT_SERIAL_BAUD=%s\n' "${ROBOT_SERIAL_BAUD}" >>"${ENV_FILE}"
fi

export ROBOT_SERIAL_DEVICE ROBOT_SERIAL_BAUD

if ! docker compose version >/dev/null 2>&1; then
    echo "Docker Compose plugin is required." >&2
    exit 1
fi

systemctl enable --now docker.service

# Remove the previous dual-supervisor setup. Docker Compose becomes the only
# owner of robot runtime containers.
OLD_SERVICES=(
    robot-control-bridge.service
    robot-control-camera.service
    robot-control-base.service
    robot-control-agent-container.service
    robot-control-ros-container.service
)
systemctl disable --now "${OLD_SERVICES[@]}" 2>/dev/null || true
for unit in "${OLD_SERVICES[@]}"; do
    rm -f "/etc/systemd/system/${unit}"
done
systemctl daemon-reload
systemctl reset-failed

stop_old_image_containers() {
    local image="$1"
    local container_id
    while IFS= read -r container_id; do
        [[ -n "${container_id}" ]] || continue
        docker update --restart=no "${container_id}" >/dev/null 2>&1 || true
        docker rm -f "${container_id}" >/dev/null 2>&1 || true
    done < <(docker ps -aq --filter "ancestor=${image}")
}

stop_old_named_containers() {
    local container_name
    for container_name in "$@"; do
        if docker container inspect "${container_name}" >/dev/null 2>&1; then
            docker update --restart=no "${container_name}" >/dev/null 2>&1 || true
            docker rm -f "${container_name}" >/dev/null 2>&1 || true
        fi
    done
}

# smart_ros2.sh recreates this legacy all-in-one container. It publishes to
# /cmd_vel independently and conflicts with robot-command-bridge.
stop_old_named_containers yahboom_ros_main
stop_old_image_containers "microros/micro-ros-agent:humble"
stop_old_image_containers "yahboomtechnology/ros-humble:4.1.2"

docker compose build
docker compose up -d --remove-orphans
docker compose ps

serial_ready=false
for _ in {1..10}; do
    if docker exec robot-microros-agent test -c /dev/robot-controller; then
        serial_ready=true
        break
    fi
    sleep 1
done
if [[ "${serial_ready}" != true ]]; then
    echo "MCU serial device is not visible inside robot-microros-agent." >&2
    exit 1
fi

echo "Robot Docker runtime installed."
echo "MCU serial mapping: ${ROBOT_SERIAL_DEVICE} -> /dev/robot-controller"
echo "MCU serial baud: ${ROBOT_SERIAL_BAUD}"
echo "Fixed container names:"
echo "  robot-microros-agent"
echo "  robot-base-node (yahboomcar_bringup)"
echo "  robot-command-bridge"
echo "  robot-camera-stream"
echo "If the agent log stops at 'running... fd: 3', reset the MCU once."
echo "LiDAR check: docker exec robot-base-node bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic hz /scan'"
