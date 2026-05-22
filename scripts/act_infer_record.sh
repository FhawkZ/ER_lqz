#!/usr/bin/env bash
# 策略闭环录制评测（LeRobot 0.5.1）：仅 policy，无 teleop
# 环境变量: POLICY_PATH, DATASET_REPO_ID, DATASET_ROOT

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

POLICY_PATH="${POLICY_PATH:-/media/disk/isaac_lqz/models/act_redcube_merged_quat/checkpoints/100000/pretrained_model}"
DATASET_REPO_ID="${DATASET_REPO_ID:-local/eval_act_infer_fr3_l6}"
DATASET_ROOT="${DATASET_ROOT:-/home/franka/lqz/Data/act_infer_fr3_l6}"

CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

exec lerobot-record \
  --robot.type=fr3_linker_l6_follower \
  --robot.cameras="${CAMERAS}" \
  --policy.path="${POLICY_PATH}" \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_ROOT}" \
  --dataset.fps=30 \
  --dataset.num_episodes="${NUM_EPISODES:-1}" \
  --dataset.episode_time_s="${EPISODE_TIME_S:-120}" \
  --dataset.reset_time_s=20 \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --dataset.push_to_hub=false \
  --dataset.single_task="${SINGLE_TASK:-pick the red cube and drop it in box}" \
  --display_data=true
