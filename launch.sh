#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export IGN_GAZEBO_RESOURCE_PATH="$PROJECT_DIR/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="$PROJECT_DIR/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export IGN_PARTITION="fleetscope_estate_${UID}"
export IGN_IP=127.0.0.1
export QT_LOGGING_RULES='*.debug=false;qt.qml.connections=false'
exec ign gazebo "$PROJECT_DIR/worlds/plantation.sdf" "$@"
