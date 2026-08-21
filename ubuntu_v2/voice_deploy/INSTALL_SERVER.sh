#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run with sudo: sudo ./voice_deploy/INSTALL_SERVER.sh" >&2
    exit 1
fi

SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
INSTALL_DIR="/opt/robot-voice"
GATEWAY_CONTAINER="robot-v2-gateway"

if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
    echo "Docker Engine and Docker Compose are required." >&2
    exit 1
fi

# Reuse the running gateway's token without printing it or placing it in shell history.
COMMAND_TOKEN="$(
    docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "${GATEWAY_CONTAINER}" \
        | sed -n 's/^COMMAND_TOKEN=//p' | tail -n 1
)"
if [[ -z "${COMMAND_TOKEN}" ]]; then
    echo "Could not read COMMAND_TOKEN from ${GATEWAY_CONTAINER}." >&2
    exit 1
fi

install -d -m 0755 "${INSTALL_DIR}/docker" "${INSTALL_DIR}/robot_app" \
    "${INSTALL_DIR}/models"
install -m 0644 "${SOURCE_DIR}/docker/Dockerfile.voice" \
    "${INSTALL_DIR}/docker/Dockerfile.voice"
install -m 0755 "${SOURCE_DIR}/docker/voice-entrypoint.sh" \
    "${INSTALL_DIR}/docker/voice-entrypoint.sh"
install -m 0644 "${SOURCE_DIR}/robot_app/voice_command_server.py" \
    "${INSTALL_DIR}/robot_app/voice_command_server.py"
install -m 0644 "${SOURCE_DIR}/voice_deploy/compose.yaml" \
    "${INSTALL_DIR}/compose.yaml"

umask 077
printf 'COMMAND_TOKEN=%s\n' "${COMMAND_TOKEN}" > "${INSTALL_DIR}/.env"

cd "${INSTALL_DIR}"
docker compose config >/dev/null
docker compose build voice-command
docker compose up -d voice-command

echo "Robot voice service installed in ${INSTALL_DIR}."
echo "The model downloads once on first startup; check:"
echo "  curl http://127.0.0.1:10000/health"
