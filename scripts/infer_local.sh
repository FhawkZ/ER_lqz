#!/usr/bin/env bash
# 本地策略推理（LeRobot 0.5.1 lerobot-record + policy，无 teleop / 无 gRPC）
# 与 record.sh 对齐：fr3_eef、相同相机与 task；权重默认 act_redcube_merged @ 60k
#
# 前置: cartesian_impedance_controller +（若策略控手）Linker Hand launch
#
# 用法:
#   bash scripts/infer_local.sh
#   EPISODE_TIME_S=60 NUM_EPISODES=1 bash scripts/infer_local.sh
#   POLICY_PATH=/path/to/pretrained_model bash scripts/infer_local.sh
#
# 键盘（与 record.sh 相同）: → 结束本段/reset；← 重录；Esc 停止
# 环境变量: POLICY_PATH, POLICY_DEVICE, ROBOT_TYPE, DATASET_REPO_ID, DATASET_ROOT,
#           NUM_EPISODES, EPISODE_TIME_S, SINGLE_TASK, WIPE_DATASET, RESUME,
#           INTERPOLATION_MULTIPLIER, DISPLAY_DATA, FPS, INFER_TRACE_DIR
#
# 轨迹日志（默认开启）: 每帧记录 state / policy / sent 的 12 维动作到 CSV
#   INFER_TRACE_DIR=... bash scripts/infer_local.sh
#   python scripts/analyze_infer_trace.py "$INFER_TRACE_DIR"

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

POLICY_PATH="${POLICY_PATH:-${ER_LQZ_ROOT}/outputs/act_redcube_merged/checkpoints/060000/pretrained_model}"
POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
ROBOT_TYPE="${ROBOT_TYPE:-fr3_eef}"

if [[ ! -f "${POLICY_PATH}/config.json" ]]; then
  echo "错误: 未找到策略权重 ${POLICY_PATH}/config.json"
  exit 1
fi

DATASET_REPO_ID="${DATASET_REPO_ID:-franka_hand/eval_act_redcube_merged_060000}"
DATASET_ROOT="${DATASET_ROOT:-/home/franka/lqz/Data}"
DATASET_LOCAL_ROOT="${DATASET_LOCAL_ROOT:-${DATASET_ROOT%/}/${DATASET_REPO_ID}}"

RESUME="${RESUME:-false}"
WIPE_DATASET="${WIPE_DATASET:-false}"
NUM_EPISODES="${NUM_EPISODES:-1}"
EPISODE_TIME_S="${EPISODE_TIME_S:-120}"
INTERPOLATION_MULTIPLIER="${INTERPOLATION_MULTIPLIER:-1}"
DISPLAY_DATA="${DISPLAY_DATA:-true}"
FPS="${FPS:-30}"
INFER_TRACE_DIR="${INFER_TRACE_DIR:-${ER_LQZ_ROOT}/logs/infer_trace/$(date +%Y%m%d_%H%M%S)}"
export INFER_TRACE_DIR

if [[ "${WIPE_DATASET}" == "true" && "${RESUME}" != "true" ]]; then
  if [[ -d "${DATASET_LOCAL_ROOT}" ]]; then
    echo "WIPE_DATASET: 删除 ${DATASET_LOCAL_ROOT}"
    rm -rf "${DATASET_LOCAL_ROOT}"
    sleep 0.5
  fi
fi

# 与 record.sh 相同的 RealSense
CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

echo ">>> 本地推理: policy=${POLICY_PATH}"
echo ">>> 机器人: ${ROBOT_TYPE} @ ${FPS} Hz | 评测数据集: ${DATASET_LOCAL_ROOT}"
echo ">>> 轨迹日志: ${INFER_TRACE_DIR}  (分析: python scripts/analyze_infer_trace.py \"${INFER_TRACE_DIR}\")"

exec lerobot-record \
  --robot.type="${ROBOT_TYPE}" \
  --robot.cameras="${CAMERAS}" \
  --policy.path="${POLICY_PATH}" \
  --policy.device="${POLICY_DEVICE}" \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_LOCAL_ROOT}" \
  --dataset.fps="${FPS}" \
  --dataset.num_episodes="${NUM_EPISODES}" \
  --dataset.episode_time_s="${EPISODE_TIME_S}" \
  --dataset.reset_time_s=20 \
  --dataset.streaming_encoding=true \
  --dataset.encoder_threads=2 \
  --dataset.push_to_hub=false \
  --dataset.single_task="${SINGLE_TASK:-pick the red cube and drop it in box}" \
  --display_data="${DISPLAY_DATA}" \
  --interpolation_multiplier="${INTERPOLATION_MULTIPLIER}" \
  --resume=false
