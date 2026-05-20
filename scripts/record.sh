#!/usr/bin/env bash
# LeRobot 0.5.1 数据采集：mocap_leader + fr3_linker_l6_follower
#
# 键盘（窗口需有焦点）: → 结束当前段/进入 reset；← 重录本集；Esc 停止脚本
#
# 环境变量覆盖示例:
#   DATASET_REPO_ID=franka_hand/redcube14
#   DATASET_ROOT=/home/franka/lqz/Data
#   WIPE_DATASET=true          # 仅删除 DATASET_LOCAL_ROOT，再重新录制
#   RESUME=true                # 往同一数据集追加 episode
#   PUSH_TO_HUB=true
#   NUM_EPISODES=10

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

DATASET_REPO_ID="${DATASET_REPO_ID:-franka_hand/redcube}"
DATASET_ROOT="${DATASET_ROOT:-/home/franka/lqz/Data}"
DATASET_LOCAL_ROOT="${DATASET_LOCAL_ROOT:-${DATASET_ROOT%/}/${DATASET_REPO_ID}}"

PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
DATASET_PRIVATE="${DATASET_PRIVATE:-false}"
RESUME="${RESUME:-false}"
NUM_EPISODES="${NUM_EPISODES:-10}"
WIPE_DATASET="${WIPE_DATASET:-false}"

if [[ "${WIPE_DATASET}" == "true" && "${RESUME}" != "true" ]]; then
  if [[ -d "${DATASET_LOCAL_ROOT}" ]]; then
    echo "WIPE_DATASET: 删除 ${DATASET_LOCAL_ROOT}"
    rm -rf "${DATASET_LOCAL_ROOT}"
    sleep 0.5
  fi
fi

CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

exec lerobot-record \
  --robot.type=fr3_linker_l6_follower \
  --robot.cameras="${CAMERAS}" \
  --teleop.type=mocap_leader \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_LOCAL_ROOT}" \
  --dataset.fps=30 \
  --dataset.num_episodes="${NUM_EPISODES}" \
  --dataset.reset_time_s=20 \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --dataset.push_to_hub="${PUSH_TO_HUB}" \
  --dataset.private="${DATASET_PRIVATE}" \
  --dataset.single_task="${SINGLE_TASK:-pick the red cube and drop it in box}" \
  --display_data=true \
  --resume="${RESUME}"
