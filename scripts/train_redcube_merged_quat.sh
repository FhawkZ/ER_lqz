#!/usr/bin/env bash
# 在本地数据集 redcube_merged_quat 上训练策略（默认 Diffusion Policy）
#
# 用法:
#   conda activate lerobot
#   bash scripts/train_redcube_merged_quat.sh
#
# 换策略类型（例如 ACT）:
#   POLICY_TYPE=act JOB_NAME=act_redcube_merged_quat bash scripts/train_redcube_merged_quat.sh
#
# 指定 GPU:
#   CUDA_VISIBLE_DEVICES=1 bash scripts/train_redcube_merged_quat.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

# ========== 常用参数（按需改 / 环境变量覆盖）==========
POLICY_TYPE="${POLICY_TYPE:-diffusion}"

# 本地数据集：root 必须指向含 meta/、data/、videos/ 的那一层
DATASET_REPO_ID="${DATASET_REPO_ID:-redcube_merged_quat}"
DATASET_ROOT="${DATASET_ROOT:-${ER_LQZ_ROOT}/data/${DATASET_REPO_ID}}"

OUTPUT_DIR="${OUTPUT_DIR:-${ER_LQZ_ROOT}/models/${POLICY_TYPE}_${DATASET_REPO_ID}}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"

SEED="${SEED:-1000}"
STEPS="${STEPS:-200000}"
BATCH_SIZE="${BATCH_SIZE:-16}"
EVAL_FREQ="${EVAL_FREQ:-20000}"
SAVE_FREQ="${SAVE_FREQ:-20000}"
NUM_WORKERS="${NUM_WORKERS:-4}"
LOG_FREQ="${LOG_FREQ:-200}"

POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
POLICY_REPO_ID="${POLICY_REPO_ID:-}"
JOB_NAME="${JOB_NAME:-${POLICY_TYPE}_${DATASET_REPO_ID}}"

# ---------- WandB ----------
WANDB_ENABLE="${WANDB_ENABLE:-true}"
WANDB_PROJECT="${WANDB_PROJECT:-fr3}"
WANDB_ENTITY="${WANDB_ENTITY:-}"
WANDB_NOTES="${WANDB_NOTES:-}"
WANDB_MODE="${WANDB_MODE:-offline}"
WANDB_DISABLE_ARTIFACT="${WANDB_DISABLE_ARTIFACT:-true}"
WANDB_ADD_TAGS="${WANDB_ADD_TAGS:-true}"

EXTRA_ARGS=()
# ========================================

if [[ ! -f "${DATASET_ROOT}/meta/info.json" ]]; then
  echo "错误: 数据集不存在或 meta 不完整: ${DATASET_ROOT}"
  echo "请先合并数据集: bash scripts/merge_datasets.sh"
  exit 1
fi

echo ">>> 训练配置"
echo "  policy       : ${POLICY_TYPE}"
echo "  dataset      : ${DATASET_ROOT}"
echo "  output       : ${OUTPUT_DIR}"
echo "  GPU          : ${CUDA_VISIBLE_DEVICES}"
echo "  steps        : ${STEPS}"

CMD=(
  lerobot-train
  --policy.type="${POLICY_TYPE}"
  --policy.device="${POLICY_DEVICE}"
  --policy.push_to_hub="${PUSH_TO_HUB}"
  --dataset.repo_id="${DATASET_REPO_ID}"
  --dataset.root="${DATASET_ROOT}"
  --output_dir="${OUTPUT_DIR}"
  --job_name="${JOB_NAME}"
  --seed="${SEED}"
  --steps="${STEPS}"
  --batch_size="${BATCH_SIZE}"
  --eval_freq="${EVAL_FREQ}"
  --save_freq="${SAVE_FREQ}"
  --log_freq="${LOG_FREQ}"
  --num_workers="${NUM_WORKERS}"
  --wandb.enable="${WANDB_ENABLE}"
  --wandb.project="${WANDB_PROJECT}"
  --wandb.disable_artifact="${WANDB_DISABLE_ARTIFACT}"
  --wandb.add_tags="${WANDB_ADD_TAGS}"
)

if [[ -n "${POLICY_REPO_ID}" ]]; then
  CMD+=(--policy.repo_id="${POLICY_REPO_ID}")
fi
if [[ -n "${WANDB_ENTITY}" ]]; then
  CMD+=(--wandb.entity="${WANDB_ENTITY}")
fi
if [[ -n "${WANDB_NOTES}" ]]; then
  CMD+=(--wandb.notes="${WANDB_NOTES}")
fi
if [[ -n "${WANDB_MODE}" ]]; then
  CMD+=(--wandb.mode="${WANDB_MODE}")
fi

exec "${CMD[@]}" "${EXTRA_ARGS[@]}"
