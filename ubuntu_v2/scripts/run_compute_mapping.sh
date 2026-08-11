#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

cd "${PROJECT_DIR}"
mkdir -p maps data
docker compose --profile compute up -d gateway ros-transport compute-mapping

# zenoh-bridge-ros2dds builds its ROS routes from the graph it sees at
# startup.  On a cold boot the bridge can win the race against the compute
# container and leave scan/odom routes missing until it is refreshed.  Wait
# for the compute graph, then refresh only the stateless transport process.
compute_graph_ready="false"
for _ in $(seq 1 30); do
    if docker exec robot-v2-compute-mapping bash -lc \
        'source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null' \
        | grep -qx '/navigation_scan_filter'; then
        compute_graph_ready="true"
        break
    fi
    sleep 1
done

if [[ "${compute_graph_ready}" != "true" ]]; then
    echo "Compute ROS graph did not become ready within 30 seconds." >&2
    docker compose ps gateway ros-transport compute-mapping >&2
    exit 1
fi

docker restart robot-v2-ros-transport >/dev/null
docker compose ps gateway ros-transport compute-mapping

echo "Compute mapping is ready but idle. Start exploration from the GUI."
