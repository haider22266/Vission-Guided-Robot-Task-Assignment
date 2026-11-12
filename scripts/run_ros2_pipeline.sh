#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=42
cd "$ROOT"
colcon --log-base "$ROOT/log_ros2" build --base-paths "$ROOT/src" --packages-select disaster_mrta \
    --build-base "$ROOT/build_ros2" --install-base "$ROOT/install_ros2" --symlink-install
source "$ROOT/install_ros2/setup.bash"
MODE="${1:-demo}"
COUNT="${2:-1}"
if [[ "$MODE" == demo ]]; then
    exec ros2 launch disaster_mrta vision_mrta.launch.py robot_count:="$COUNT" \
        allocator_backend:=baseline perception_backend:=mock execution_backend:=timed_demo
elif [[ "$MODE" == trained ]]; then
    EFFICIENTNET_MODEL="${3:-${EFFICIENTNET_MODEL:-}}"
    QMIX_CHECKPOINT="${4:-${QMIX_CHECKPOINT:-}}"
    : "${EFFICIENTNET_MODEL:?Set EFFICIENTNET_MODEL to a trained Keras model path}"
    : "${QMIX_CHECKPOINT:?Set QMIX_CHECKPOINT to a compatible trained checkpoint path}"
    exec ros2 launch disaster_mrta vision_mrta.launch.py robot_count:="$COUNT" \
        allocator_backend:=qmix perception_backend:=efficientnet execution_backend:=external \
        efficientnet_model:="$EFFICIENTNET_MODEL" qmix_checkpoint:="$QMIX_CHECKPOINT"
else
    echo 'Mode must be demo or trained' >&2
    exit 2
fi
