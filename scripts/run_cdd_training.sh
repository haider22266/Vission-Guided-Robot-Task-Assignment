#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/scripts/cdd_env.sh"
export KERAS_HOME="$ROOT/results/cdd/keras_cache"
export TF_CPP_MIN_LOG_LEVEL=1
cd "$ROOT"
"$ROOT/.venv-cdd-wsl/bin/python" "$ROOT/scripts/probe_cdd_tensorflow.py"
"$ROOT/.venv-cdd-wsl/bin/python" "$ROOT/scripts/test_cdd_training.py"
"$ROOT/.venv-cdd-wsl/bin/python" "$ROOT/scripts/train_cdd_classifier.py" "$@"
