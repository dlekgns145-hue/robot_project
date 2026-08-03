#!/usr/bin/env bash
set -eo pipefail

# ROS 2 setup scripts intentionally read optional, possibly unset variables.
# Enabling `set -u` while sourcing them makes otherwise valid setup files fail.
set +u

if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

if [[ -n "${ROBOT_WORKSPACE_SETUP:-}" && -f "${ROBOT_WORKSPACE_SETUP}" ]]; then
    source "${ROBOT_WORKSPACE_SETUP}"
fi

exec "$@"
