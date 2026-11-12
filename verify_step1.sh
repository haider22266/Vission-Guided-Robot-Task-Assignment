#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
cd "$(dirname "$0")"
python3 - <<'PY'
import importlib.util
for name in ('tensorflow', 'cv_bridge', 'rclpy', 'pytest', 'yaml'):
    print(name, importlib.util.find_spec(name) is not None)
PY
PYTHONPATH=src/disaster_mrta python3 -m disaster_mrta.config_loader --config-dir src/disaster_mrta/config
if python3 -m pytest --version; then
    PYTHONPATH=src/disaster_mrta python3 -m pytest src/disaster_mrta/tests -q
fi
if command -v colcon; then
    colcon build --symlink-install --packages-select disaster_mrta
    source install/setup.bash
    ros2 run disaster_mrta validate_config
fi

