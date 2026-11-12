#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
python3 - <<'PY'
import importlib.util
for module in ('rclpy','cv_bridge','torch','tensorflow','pytest','launch_ros'):
    print(module, importlib.util.find_spec(module) is not None)
PY
