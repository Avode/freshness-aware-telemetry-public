#!/usr/bin/env bash
# Extract signed-repository Debian packages into a user-owned overlay; no sudo.
set -eo pipefail
PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_OVERLAY="${FLEETSCOPE_OVERLAY:-${FLEETSCOPE_ASSETS:-$PROJECT_ROOT/assets}/ros-overlay}"
mkdir -p "$TASK_OVERLAY/debs"
cd "$TASK_OVERLAY/debs"
apt download ros-humble-robot-localization ros-humble-geographic-msgs libgeographic19
for deb in ./*.deb; do dpkg-deb -x "$deb" "$TASK_OVERLAY"; done
dpkg-deb -f ./*robot-localization*.deb Package Version > "$TASK_OVERLAY/versions.txt"
