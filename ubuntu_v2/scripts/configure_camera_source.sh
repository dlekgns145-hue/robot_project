#!/usr/bin/env bash
set -euo pipefail

INSTALL_DIR="${ROBOT_SERVER_DIR:-/opt/robot-control-server}"
COMPOSE_DIR="${INSTALL_DIR}/ubuntu_v2"
ENV_FILE="${COMPOSE_DIR}/.env"
COMPUTE_CONTAINER="robot-v2-compute-mapping"
APPLY="false"
CAMERA_URL=""

usage() {
    cat <<'EOF'
Usage:
  ./configure_camera_source.sh --url URL
  sudo ./configure_camera_source.sh --url URL --apply

The first form only probes an MJPEG/HTTP camera from inside the compute
container.  --apply updates ROBOT_CAMERA_URL and recreates only the compute
container after a successful probe.

Examples:
  ./configure_camera_source.sh --url http://PHONE_IP:PORT/VIDEO_PATH
  sudo ./configure_camera_source.sh \
    --url http://172.30.1.10:8080/stream.mjpg --apply
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --url)
            CAMERA_URL="${2:?--url requires a value}"
            shift 2
            ;;
        --apply)
            APPLY="true"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            printf 'Unknown option: %s\n' "$1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ ! "${CAMERA_URL}" =~ ^https?://[^[:space:]]+$ ]]; then
    printf 'Camera URL must be a complete http:// or https:// URL.\n' >&2
    exit 2
fi
DISPLAY_URL="$(printf '%s' "${CAMERA_URL}" | sed -E \
    's#(https?://)[^/@]+@#\1<credentials>@#')"
if ! docker inspect "${COMPUTE_CONTAINER}" >/dev/null 2>&1; then
    printf 'Compute container is unavailable: %s\n' "${COMPUTE_CONTAINER}" >&2
    exit 1
fi

printf 'Probing camera from the Ubuntu compute container: %s\n' "${DISPLAY_URL}"
if ! timeout 15s docker exec \
    -e CAMERA_PROBE_URL="${CAMERA_URL}" \
    "${COMPUTE_CONTAINER}" \
    python3 -c '
import os
import cv2

url = os.environ["CAMERA_PROBE_URL"]
capture = cv2.VideoCapture(url)
try:
    if not capture.isOpened():
        raise SystemExit("camera stream did not open")
    frame = None
    for _ in range(8):
        ok, candidate = capture.read()
        if ok and candidate is not None and candidate.size:
            frame = candidate
            break
    if frame is None:
        raise SystemExit("camera opened but no frame was received")
    height, width = frame.shape[:2]
    print(f"camera probe OK: {width}x{height}")
finally:
    capture.release()
'; then
    printf 'Camera probe failed; the server configuration was not changed.\n' >&2
    exit 1
fi

if [[ "${APPLY}" != "true" ]]; then
    printf 'Probe only. Add --apply with sudo to make this source persistent.\n'
    exit 0
fi
if [[ "${EUID}" -ne 0 ]]; then
    printf 'Run with sudo when using --apply.\n' >&2
    exit 1
fi
if [[ ! -f "${ENV_FILE}" ]]; then
    printf 'Server environment file is missing: %s\n' "${ENV_FILE}" >&2
    exit 1
fi

escaped_url="$(printf '%s' "${CAMERA_URL}" | sed 's/[&|\\]/\\&/g')"
if grep -q '^ROBOT_CAMERA_URL=' "${ENV_FILE}"; then
    sed -i "s|^ROBOT_CAMERA_URL=.*|ROBOT_CAMERA_URL=${escaped_url}|" "${ENV_FILE}"
else
    printf 'ROBOT_CAMERA_URL=%s\n' "${CAMERA_URL}" >>"${ENV_FILE}"
fi
chmod 0600 "${ENV_FILE}"

cd "${COMPOSE_DIR}"
docker compose --profile compute up -d --force-recreate compute-mapping
printf 'Persistent camera source updated: %s\n' "${DISPLAY_URL}"
