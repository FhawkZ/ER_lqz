#!/usr/bin/env bash
# Start ROS2 helper nodes and record FR3 + LinkerHand + RealSense + mocap topics to MCAP.
#
# Expected hardware stack, started separately or by your lab launch files:
#   - franka_ros2 / SERL cartesian_impedance_controller
#   - linkerhand_ros2_sdk
#   - realsense-ros for handeye and fixed cameras
#   - mocap/teleop publisher or existing LeRobot teleop process

set -euo pipefail

RECORDING_ID="${RECORDING_ID:-$(date +%Y%m%d_%H%M%S)_fr3_linkerhand}"
TASK="${TASK:-pick the red cube and drop it in box}"
OUTPUT_DIR="${OUTPUT_DIR:-${HOME}/dexterous_records}"
OPERATOR_ID="${OPERATOR_ID:-unknown}"
ENVIRONMENT_ID="${ENVIRONMENT_ID:-default}"
INCLUDE_OPTIONAL_TOPICS="${INCLUDE_OPTIONAL_TOPICS:-true}"
START_HELPER_LAUNCH="${START_HELPER_LAUNCH:-true}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "错误: 未找到 ros2。请先 source ROS2/工作空间环境。"
  exit 1
fi

if [[ "${START_HELPER_LAUNCH}" == "true" ]]; then
  ros2 launch dexterous_platform_ros record_fr3_linkerhand.launch.py \
    recording_id:="${RECORDING_ID}" \
    task:="${TASK}" \
    output_dir:="${OUTPUT_DIR}" \
    operator_id:="${OPERATOR_ID}" \
    environment_id:="${ENVIRONMENT_ID}" &
  HELPER_PID=$!
  trap 'kill "${HELPER_PID}" >/dev/null 2>&1 || true' EXIT
  sleep 2
fi

python3 scripts/dex_start_rosbag2.py \
  --recording-id "${RECORDING_ID}" \
  --task "${TASK}" \
  --output-dir "${OUTPUT_DIR}" \
  --include-optional-topics "${INCLUDE_OPTIONAL_TOPICS}" \
  --execute
