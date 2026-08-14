#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT_DIR="${OUTPUT_DIR:-${PROJECT_DIR}/images}"

if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
    echo "Docker Engine with the Compose plugin is required." >&2
    exit 1
fi

cd "${PROJECT_DIR}/ubuntu_v2"
docker compose --profile compute build gateway map-postprocessor
install -d "${OUTPUT_DIR}"
docker save -o "${OUTPUT_DIR}/robot-control-server-images.tar" \
    robot-control-v2-gateway:bookworm \
    robot-control-compute:humble

echo "Created ${OUTPUT_DIR}/robot-control-server-images.tar"
echo "Run build_server_image.sh again to include it in an offline bundle."
