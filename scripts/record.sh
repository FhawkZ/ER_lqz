#!/usr/bin/env bash
# 数据采集：mocap_retarget_leader → fr3_eef（LeRobot 0.5.1）
# 与 teleop 相同配对；action 仍为 7D EE 四元数 + 6D hand。
# observation.state 默认 7D 关节角 + 6D hand + 7D 外力矩（与 effort 对齐）。
# 录 legacy 13D EE 数据: --robot.arm_observation_rep=eef_quat
#
# 键盘: → 结束当前段/reset；← 重录本集；Esc 停止
# 环境变量: DATASET_REPO_ID, DATASET_ROOT, WIPE_DATASET, RESUME, NUM_EPISODES, SINGLE_TASK

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

DATASET_REPO_ID="${DATASET_REPO_ID:-franka_hand/joint_assemble_20260708}"
DATASET_ROOT="${DATASET_ROOT:-/home/franka/lqz/Data}"
DATASET_LOCAL_ROOT="${DATASET_LOCAL_ROOT:-${DATASET_ROOT%/}/${DATASET_REPO_ID}}"

PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
DATASET_PRIVATE="${DATASET_PRIVATE:-false}"
RESUME="${RESUME:-true}"
NUM_EPISODES="${NUM_EPISODES:-1}"
WIPE_DATASET="${WIPE_DATASET:-false}"
EPISODE_TIME_S="${EPISODE_TIME_S:-120}" 

if [[ "${WIPE_DATASET}" == "true" && "${RESUME}" != "true" ]]; then
  if [[ -d "${DATASET_LOCAL_ROOT}" ]]; then
    echo "WIPE_DATASET: 删除 ${DATASET_LOCAL_ROOT}"
    rm -rf "${DATASET_LOCAL_ROOT}"
    sleep 0.5
  fi
fi

CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

exec lerobot-record \
  --robot.type="${LEROBOT_ROBOT_TYPE}" \
  --robot.cameras="${CAMERAS}" \
  --teleop.type="${LEROBOT_TELEOP_TYPE}" \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_LOCAL_ROOT}" \
  --dataset.fps=30 \
  --dataset.num_episodes="${NUM_EPISODES}" \
  --dataset.episode_time_s="${EPISODE_TIME_S}" \
  --dataset.reset_time_s=20 \
  --dataset.streaming_encoding=false \
  --dataset.encoder_threads=2 \
  --dataset.push_to_hub="${PUSH_TO_HUB}" \
  --dataset.private="${DATASET_PRIVATE}" \
  --dataset.single_task="${SINGLE_TASK:-insert the magnetic encoder disk assembly into the joint motor shaft bore with coaxial alignment and compilant insertion}" \
  --display_data=false \
  --resume="${RESUME}"
