#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source autonomy/env.sh
exec /usr/bin/python3 -m autonomy.launch --command-center --no-rviz "$@"
