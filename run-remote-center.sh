#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source autonomy/env.sh
if [[ -z "${FLEETSCOPE_MQTT_CONFIG:-}" ]]; then
  echo 'Set FLEETSCOPE_MQTT_CONFIG to a private broker JSON file outside this repository.' >&2
  exit 2
fi
export FLEETSCOPE_SITE_CONFIG="${FLEETSCOPE_SITE_CONFIG:-$PWD/network/site.json}"
exec /usr/bin/python3 -m autonomy.launch --remote-center --no-rviz "$@"
