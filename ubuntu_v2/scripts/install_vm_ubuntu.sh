#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run with sudo: sudo ./scripts/install_vm_ubuntu.sh" >&2
    exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker Engine with the compose plugin must be installed first." >&2
    exit 1
fi
if ! docker compose version >/dev/null 2>&1; then
    echo "Docker Compose plugin is required." >&2
    exit 1
fi

SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET_DIR="/opt/robot-control-v2"

install -d "${TARGET_DIR}"
cp -a "${SOURCE_DIR}/." "${TARGET_DIR}/"

if [[ ! -f "${TARGET_DIR}/.env" ]]; then
    if [[ -f "${TARGET_DIR}/.env.robot-ready" ]]; then
        cp "${TARGET_DIR}/.env.robot-ready" "${TARGET_DIR}/.env"
        echo "Created ${TARGET_DIR}/.env from the robot-ready configuration."
    else
        cp "${TARGET_DIR}/.env.example" "${TARGET_DIR}/.env"
        echo "Created ${TARGET_DIR}/.env. Verify robot settings." >&2
    fi
fi

install -m 0644 \
    "${TARGET_DIR}/systemd/robot-control-v2.service" \
    /etc/systemd/system/robot-control-v2.service
systemctl daemon-reload
cd "${TARGET_DIR}"
docker compose config >/dev/null
docker compose build gateway
systemctl enable --now robot-control-v2.service

echo "Ubuntu VM robot gateway installed in ${TARGET_DIR}."
echo "The gateway will now start automatically whenever Ubuntu boots."
