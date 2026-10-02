#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source milestone/env.sh
exec /usr/bin/python3 -m milestone.launch "$@"
