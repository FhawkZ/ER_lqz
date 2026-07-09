#!/usr/bin/env bash
# joint_assemble_j13：只训 action expert（冻结整块 VLM）
# obs: 13D 关节角+手；action: 13D EE+手
#
#   bash scripts/train_joint_assemble_pi_expert.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

POLICY_TYPE="${POLICY_TYPE:-pi0}"
PRETRAINED_PATH="${PRETRAINED_PATH:-lerobot/pi0_base}"
[[ "${POLICY_TYPE}" == "pi05" ]] && PRETRAINED_PATH="${PRETRAINED_PATH:-lerobot/pi05_base}"

DATASET_REPO_ID="${DATASET_REPO_ID:-joint_assemble_j13}"
DATASET_ROOT="${DATASET_ROOT:-${ER_LQZ_ROOT}/data/${DATASET_REPO_ID}}"
BATCH_SIZE="${BATCH_SIZE:-8}"
OUTPUT_DIR="${OUTPUT_DIR:-${ER_LQZ_ROOT}/models/${POLICY_TYPE}_${DATASET_REPO_ID}_expert_only_bs${BATCH_SIZE}}"
JOB_NAME="${JOB_NAME:-${POLICY_TYPE}_${DATASET_REPO_ID}_expert_only_bs${BATCH_SIZE}}"
RESUME="${RESUME:-false}"

export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-1}"

if [[ ! -f "${DATASET_ROOT}/meta/info.json" ]]; then
  echo "错误: 数据集不存在: ${DATASET_ROOT}"
  exit 1
fi
if [[ "${RESUME}" != "true" && -d "${OUTPUT_DIR}" ]]; then
  echo "错误: 输出目录已存在: ${OUTPUT_DIR}（删除或 RESUME=true）"
  exit 1
fi

echo ">>> expert_only  GPU=${CUDA_VISIBLE_DEVICES}  data=${DATASET_ROOT}  out=${OUTPUT_DIR}"

exec lerobot-train \
  --policy.type="${POLICY_TYPE}" \
  --policy.pretrained_path="${PRETRAINED_PATH}" \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --policy.dtype=bfloat16 \
  --policy.use_amp=false \
  --policy.chunk_size=50 \
  --policy.n_action_steps=50 \
  --policy.gradient_checkpointing=true \
  --policy.freeze_vision_encoder=true \
  --policy.train_expert_only=true \
  --policy.optimizer_lr=0.0002 \
  --policy.scheduler_warmup_steps=1000 \
  --policy.scheduler_decay_steps=100000 \
  --policy.scheduler_decay_lr=0.00002 \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_ROOT}" \
  --output_dir="${OUTPUT_DIR}" \
  --job_name="${JOB_NAME}" \
  --resume="${RESUME}" \
  --seed=1000 \
  --steps=200000 \
  --batch_size="${BATCH_SIZE}" \
  --eval_freq=1000000 \
  --save_freq=40000 \
  --log_freq=200 \
  --num_workers=8 \
  --wandb.enable=true \
  --wandb.project=fr3 \
  --wandb.mode=offline \
  --wandb.disable_artifact=true \
  --wandb.add_tags=true \
  --wandb.notes="${POLICY_TYPE} expert_only joint+hand on ${DATASET_REPO_ID}"
