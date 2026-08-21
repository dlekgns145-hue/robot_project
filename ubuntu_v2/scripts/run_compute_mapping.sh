#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${PROJECT_DIR}"
mkdir -p maps data models
# Previous server-compute deployments may still have these containers under
# restart policies. They must be stopped before the Pi-local SLAM pipeline is
# exposed, otherwise duplicate mapping actions and sensor paths can coexist.
docker compose --profile compute stop compute-mapping ros-transport >/dev/null 2>&1 || true
docker compose --profile compute up -d gateway map-postprocessor voice-command
docker compose ps gateway map-postprocessor voice-command

echo "Gateway, Whisper voice commands, and Ubuntu map post-processor are ready."
echo "Start exploration from the GUI; only a completed map is transferred."
