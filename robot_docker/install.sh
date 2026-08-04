#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run with sudo: sudo bash install.sh" >&2
    exit 1
fi

DEPLOY_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${DEPLOY_DIR}"

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

stop_old_image_containers "microros/micro-ros-agent:humble"
stop_old_image_containers "yahboomtechnology/ros-humble:4.1.2"

docker compose build
docker compose up -d --remove-orphans
docker compose ps

echo "Robot Docker runtime installed."
echo "Fixed container names:"
echo "  robot-microros-agent"
echo "  robot-base-node"
echo "  robot-command-bridge"
echo "  robot-camera-stream"
