#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$ROOT/src/disaster_mrta:${PYTHONPATH:-}"
python3 "$ROOT/scripts/test_ros2_transport.py"
