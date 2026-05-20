#!/usr/bin/env bash
# 异步推理客户端（LeRobot 0.5.1 async_inference）
# 需先在同一或另一机器运行 scripts/server.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

SERVER_ADDRESS="${SERVER_ADDRESS:-172.18.5.61:8080}"
POLICY_PATH="${POLICY_PATH:-/home/franka/lqz/models/redcube_act/080000/pretrained_model}"
# 策略若按关节空间训练，请保持 fr3_linker_l6_follower；EE 空间策略才用 fr3_eef
ROBOT_TYPE="${ROBOT_TYPE:-fr3_linker_l6_follower}"

CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

exec python -m lerobot.async_inference.robot_client \
  --robot.type="${ROBOT_TYPE}" \
  --robot.cameras="${CAMERAS}" \
  --server_address="${SERVER_ADDRESS}" \
  --policy_type="${POLICY_TYPE:-act}" \
  --pretrained_name_or_path="${POLICY_PATH}" \
  --actions_per_chunk="${ACTIONS_PER_CHUNK:-100}" \
  --chunk_size_threshold="${CHUNK_SIZE_THRESHOLD:-0.5}" \
  --aggregate_fn_name="${AGGREGATE_FN:-weighted_average}" \
  --fps="${FPS:-30}" \
  --task="${TASK:-pick the red cube and drop it in box}" \
  --policy_device="${POLICY_DEVICE:-cuda}" \
  --client_device="${CLIENT_DEVICE:-cpu}"
