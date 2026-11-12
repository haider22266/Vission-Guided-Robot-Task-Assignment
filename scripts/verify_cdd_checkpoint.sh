#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/cdd_env.sh"
exec "$CDD_PYTHON" "$ROOT/scripts/verify_cdd_checkpoint.py"
