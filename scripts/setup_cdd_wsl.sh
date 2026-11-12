#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="$ROOT/.venv-cdd-wsl"
mkdir -p "$ROOT/results/cdd"
if [ ! -x "$ENV_DIR/bin/python" ]; then
    python3 -m venv "$ENV_DIR" || python3 -m venv --without-pip "$ENV_DIR"
fi
if ! "$ENV_DIR/bin/python" -m pip --version >/dev/null 2>&1; then
    PIP_WHEEL="${CDD_PIP_WHEEL:-/mnt/c/Users/yutak/AppData/Local/Programs/Python/Python311/Lib/ensurepip/_bundled/pip-24.0-py3-none-any.whl}"
    if [ ! -f "$PIP_WHEEL" ]; then
        echo 'Install python3-venv/pip in WSL or set CDD_PIP_WHEEL to an existing pip wheel.' >&2
        exit 1
    fi
    PYTHONPATH="$PIP_WHEEL" "$ENV_DIR/bin/python" -m pip install --no-cache-dir pip
fi
"$ENV_DIR/bin/python" -m pip install --no-cache-dir 'tensorflow[and-cuda]==2.21.0' Pillow scikit-learn
"$ENV_DIR/bin/python" -m pip freeze > "$ROOT/results/cdd/training_requirements.txt"
