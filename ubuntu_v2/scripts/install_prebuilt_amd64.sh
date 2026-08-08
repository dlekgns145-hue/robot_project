#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run with sudo: sudo ./scripts/install_prebuilt_amd64.sh <image.tar>" >&2
    exit 1
fi

if [[ $# -ne 1 ]]; then
    echo "Usage: sudo ./scripts/install_prebuilt_amd64.sh <gateway-amd64-image.tar>" >&2
    exit 1
fi

IMAGE_ARCHIVE="$(realpath "$1")"
if [[ ! -f "${IMAGE_ARCHIVE}" ]]; then
    echo "Docker image archive not found: ${IMAGE_ARCHIVE}" >&2
    exit 1
fi
if ! command -v docker >/dev/null 2>&1; then
    echo "Docker Engine must be installed first." >&2
    exit 1
fi
if ! docker compose version >/dev/null 2>&1; then
    echo "The Docker Compose plugin is required." >&2
    exit 1
fi

SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_DIR="/opt/robot-control-v2"

docker load --input "${IMAGE_ARCHIVE}"

install -d "${TARGET_DIR}"
shopt -s dotglob nullglob
for source_item in "${SOURCE_DIR}"/*; do
    if [[ "$(basename "${source_item}")" == "data" ]]; then
        continue
    fi
    cp -a "${source_item}" "${TARGET_DIR}/"
done
shopt -u dotglob nullglob
install -d "${TARGET_DIR}/data"

if [[ ! -f "${TARGET_DIR}/.env" ]]; then
    cp "${TARGET_DIR}/.env.robot-ready" "${TARGET_DIR}/.env"
fi

install -m 0644 \
    "${TARGET_DIR}/systemd/robot-control-v2.service" \
    /etc/systemd/system/robot-control-v2.service
systemctl daemon-reload

cd "${TARGET_DIR}"
docker compose config >/dev/null
docker compose up -d --no-build gateway
systemctl enable robot-control-v2.service

docker compose ps
echo "Prebuilt amd64 gateway installed in ${TARGET_DIR}."
