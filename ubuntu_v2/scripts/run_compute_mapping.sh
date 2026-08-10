#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${PROJECT_DIR}"
mkdir -p maps data
docker compose --profile compute up -d gateway compute-mapping
docker compose ps gateway compute-mapping

echo "Compute mapping is ready but idle. Start exploration from the GUI."
