#!/usr/bin/env bash
set -euo pipefail

INSTALL_DIR="/opt/robot-control-server"
ROBOT_IP="172.30.1.10"
ROBOT_MAC="2c:cf:67:7b:48:d7"
COMMAND_TOKEN=""
INSTALL_DOCKER="true"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -d "${SCRIPT_DIR}/ubuntu_v2" && -d "${SCRIPT_DIR}/robot_docker" ]]; then
    SOURCE_DIR="${SCRIPT_DIR}"
elif [[ -d "${SCRIPT_DIR}/../ubuntu_v2" && -d "${SCRIPT_DIR}/../robot_docker" ]]; then
    SOURCE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
else
    echo "Installer payload is incomplete: ubuntu_v2 or robot_docker is missing." >&2
    exit 1
fi

usage() {
    cat <<'EOF'
Usage: sudo ./INSTALL_SERVER.sh [options]

Options:
  --robot-ip IP       Robot address (default: 172.30.1.10)
  --robot-mac MAC     Robot Wi-Fi MAC (default: 2c:cf:67:7b:48:d7)
  --token TOKEN       GUI command token (default: generate a random token)
  --no-docker-install Fail instead of installing Docker when it is missing
  -h, --help           Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot-ip)
            ROBOT_IP="${2:?--robot-ip requires a value}"
            shift 2
            ;;
        --robot-mac)
            ROBOT_MAC="${2:?--robot-mac requires a value}"
            shift 2
            ;;
        --token)
            COMMAND_TOKEN="${2:?--token requires a value}"
            shift 2
            ;;
        --no-docker-install)
            INSTALL_DOCKER="false"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ "${EUID}" -ne 0 ]]; then
    echo "Run as root: sudo ./INSTALL_SERVER.sh" >&2
    exit 1
fi

if [[ ! -r /etc/os-release ]]; then
    echo "This installer requires Ubuntu Linux." >&2
    exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" ]]; then
    echo "Unsupported distribution: ${PRETTY_NAME:-unknown}. Ubuntu is required." >&2
    exit 1
fi
case "${VERSION_ID:-}" in
    22.04|24.04|26.04) ;;
    *)
        echo "Unsupported Ubuntu version: ${VERSION_ID:-unknown}. Use Ubuntu 22.04, 24.04, or 26.04." >&2
        exit 1
        ;;
esac

case "$(dpkg --print-architecture)" in
    amd64|arm64) ;;
    *)
        echo "Only Ubuntu amd64 and arm64 are supported." >&2
        exit 1
        ;;
esac

if [[ ! "${ROBOT_IP}" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]]; then
    echo "Invalid --robot-ip: ${ROBOT_IP}" >&2
    exit 2
fi
if [[ ! "${ROBOT_MAC}" =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]]; then
    echo "Invalid --robot-mac: ${ROBOT_MAC}" >&2
    exit 2
fi

install_docker_engine() {
    echo "Installing Docker Engine and the Compose plugin..."
    apt-get update
    apt-get install -y ca-certificates curl gnupg openssl
    install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        -o /etc/apt/keyrings/docker.asc
    chmod a+r /etc/apt/keyrings/docker.asc

    local arch codename
    arch="$(dpkg --print-architecture)"
    codename="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
    printf '%s\n' \
        "deb [arch=${arch} signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu ${codename} stable" \
        > /etc/apt/sources.list.d/docker.list
    apt-get update
    apt-get install -y docker-ce docker-ce-cli containerd.io \
        docker-buildx-plugin docker-compose-plugin
    systemctl enable --now docker.service
}

if ! command -v docker >/dev/null 2>&1 || ! docker compose version >/dev/null 2>&1; then
    if [[ "${INSTALL_DOCKER}" != "true" ]]; then
        echo "Docker Engine and the Compose plugin are required." >&2
        exit 1
    fi
    install_docker_engine
fi

if ! systemctl is-active --quiet docker.service; then
    systemctl enable --now docker.service
fi

echo "Installing server application into ${INSTALL_DIR}..."
install -d "${INSTALL_DIR}/ubuntu_v2" "${INSTALL_DIR}/robot_docker"

# Preserve generated maps, the operations database, and the active .env when
# the installer is run again. Application files are refreshed in place.
cp -a "${SOURCE_DIR}/ubuntu_v2/docker" "${INSTALL_DIR}/ubuntu_v2/"
cp -a "${SOURCE_DIR}/ubuntu_v2/robot_app" "${INSTALL_DIR}/ubuntu_v2/"
cp -a "${SOURCE_DIR}/ubuntu_v2/scripts" "${INSTALL_DIR}/ubuntu_v2/"
cp -a "${SOURCE_DIR}/ubuntu_v2/systemd" "${INSTALL_DIR}/ubuntu_v2/"
cp -a "${SOURCE_DIR}/ubuntu_v2/compose.yaml" "${INSTALL_DIR}/ubuntu_v2/compose.yaml"
cp -a "${SOURCE_DIR}/ubuntu_v2/.env.example" "${INSTALL_DIR}/ubuntu_v2/.env.example"
install -d "${INSTALL_DIR}/robot_docker/recovered"
cp -a \
    "${SOURCE_DIR}/robot_docker/entrypoint.sh" \
    "${SOURCE_DIR}/robot_docker/camera_stream_server.py" \
    "${SOURCE_DIR}/robot_docker/robot_cmd_bridge.py" \
    "${SOURCE_DIR}/robot_docker/navigation_runtime_launch.py" \
    "${SOURCE_DIR}/robot_docker/mapping_runtime_launch.py" \
    "${SOURCE_DIR}/robot_docker/navigation_probe.py" \
    "${SOURCE_DIR}/robot_docker/navigation_run_test.py" \
    "${SOURCE_DIR}/robot_docker/publish_initial_pose.py" \
    "${SOURCE_DIR}/robot_docker/prepare_navigation_params.py" \
    "${SOURCE_DIR}/robot_docker/scan_diagnostics.py" \
    "${SOURCE_DIR}/robot_docker/odom_relay.py" \
    "${SOURCE_DIR}/robot_docker/scan_time_fix.py" \
    "${SOURCE_DIR}/robot_docker/autonomous_mapping.py" \
    "${SOURCE_DIR}/robot_docker/frontier_core.py" \
    "${SOURCE_DIR}/robot_docker/map_texture_core.py" \
    "${SOURCE_DIR}/robot_docker/map_texture_recorder.py" \
    "${SOURCE_DIR}/robot_docker/calibrate_map_texture.py" \
    "${SOURCE_DIR}/robot_docker/camera_obstacle_guard.py" \
    "${SOURCE_DIR}/robot_docker/mapping_slam_params.yaml" \
    "${INSTALL_DIR}/robot_docker/"
cp -a \
    "${SOURCE_DIR}/robot_docker/recovered/Broom.yaml" \
    "${SOURCE_DIR}/robot_docker/recovered/Broom.pgm" \
    "${SOURCE_DIR}/robot_docker/recovered/dwb_nav_params_fixed.yaml" \
    "${INSTALL_DIR}/robot_docker/recovered/"
install -d "${INSTALL_DIR}/ubuntu_v2/data" "${INSTALL_DIR}/ubuntu_v2/maps"

ENV_FILE="${INSTALL_DIR}/ubuntu_v2/.env"
if [[ ! -f "${ENV_FILE}" ]]; then
    cp "${INSTALL_DIR}/ubuntu_v2/.env.example" "${ENV_FILE}"
    chmod 0600 "${ENV_FILE}"
else
    echo "Preserving the existing ${ENV_FILE}."
fi

if [[ -z "${COMMAND_TOKEN}" ]]; then
    COMMAND_TOKEN="$(sed -n 's/^COMMAND_TOKEN=//p' "${ENV_FILE}" | tail -n 1)"
    if [[ -z "${COMMAND_TOKEN}" || "${COMMAND_TOKEN}" == "replace-with-a-private-value" || "${COMMAND_TOKEN}" == "change-this-token" ]]; then
        if ! command -v openssl >/dev/null 2>&1; then
            apt-get update
            apt-get install -y openssl
        fi
        COMMAND_TOKEN="$(openssl rand -hex 32)"
    fi
fi
if [[ ! "${COMMAND_TOKEN}" =~ ^[A-Za-z0-9._:@+-]{16,128}$ ]]; then
    echo "The command token must be 16-128 characters using letters, numbers, . _ : @ + or -." >&2
    exit 2
fi

set_env_value() {
    local key="$1"
    local value="$2"
    local escaped_value
    escaped_value="$(printf '%s' "${value}" | sed 's/[&|\\]/\\&/g')"
    if grep -q "^${key}=" "${ENV_FILE}"; then
        sed -i "s|^${key}=.*|${key}=${escaped_value}|" "${ENV_FILE}"
    else
        printf '%s=%s\n' "${key}" "${value}" >> "${ENV_FILE}"
    fi
}

set_env_value ROBOT_IP "${ROBOT_IP}"
set_env_value ROBOT_MAC "${ROBOT_MAC,,}"
set_env_value ROBOT_CAMERA_URL "http://${ROBOT_IP}:8080/stream.mjpg"
set_env_value COMMAND_TOKEN "${COMMAND_TOKEN}"
chmod 0600 "${ENV_FILE}"

HAS_BUNDLED_IMAGES="false"
if [[ -f "${SOURCE_DIR}/images/robot-control-server-images.tar" ]]; then
    echo "Loading bundled Docker images..."
    docker load -i "${SOURCE_DIR}/images/robot-control-server-images.tar"
    HAS_BUNDLED_IMAGES="true"
fi

cd "${INSTALL_DIR}/ubuntu_v2"
docker compose --profile compute config >/dev/null
if [[ "${HAS_BUNDLED_IMAGES}" != "true" ]]; then
    docker compose --profile compute pull ros-transport
    docker compose --profile compute build gateway compute-mapping
fi

install -m 0644 \
    "${INSTALL_DIR}/ubuntu_v2/systemd/robot-control-server.service" \
    /etc/systemd/system/robot-control-server.service
systemctl daemon-reload
systemctl enable --now robot-control-server.service

if command -v ufw >/dev/null 2>&1 && ufw status | grep -q '^Status: active'; then
    ufw allow 9999/tcp
fi

SERVER_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo
echo "Robot compute server installation complete."
echo "  Server address: ${SERVER_IP:-<Ubuntu-IP>}:9999"
echo "  Robot address:  ${ROBOT_IP}"
echo "  Service:        robot-control-server.service"
echo "  Maps:           ${INSTALL_DIR}/ubuntu_v2/maps"
echo "  GUI token:      sudo sed -n 's/^COMMAND_TOKEN=//p' ${ENV_FILE}"
echo
echo "Mapping compute is running in an idle-safe state. Start exploration from the GUI."
