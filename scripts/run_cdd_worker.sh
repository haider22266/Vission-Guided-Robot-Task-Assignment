#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CDD_PYTHON="$1"
shift
source "$ROOT/scripts/cdd_env.sh"
exec "$CDD_PYTHON" "$ROOT/scripts/cdd_inference_worker.py" "$@"
