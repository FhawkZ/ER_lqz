#!/usr/bin/env bash
# 在本地 redcube 数据集上微调 Pi0 / Pi0.5（13D EE 四元数 + 6D hand，双相机 handeye/fixed）
#
# 两种微调模式（FINETUNE_MODE）:
#   lora_vlm     - LoRA 微调 VLM 语言层 + action expert + 投影层（冻结 vision encoder）
#   expert_only  - 冻结整个 VLM，仅全量训练 action expert + 投影层
#
# 用法:
#   conda activate lerobot
#   bash scripts/prefetch_pi0_assets.sh          # 首次建议预下载 pi0_base / tokenizer
#   bash scripts/train_redcube_pi.sh             # 默认 pi0 + lora_vlm
#
# Pi0.5 + 仅 expert:
#   POLICY_TYPE=pi05 FINETUNE_MODE=expert_only bash scripts/train_redcube_pi.sh
#
# 断点续训:
#   RESUME=true OUTPUT_DIR=models/pi0_redcube_lora_vlm_bs8 bash scripts/train_redcube_pi.sh
#
# 指定 GPU:
#   CUDA_VISIBLE_DEVICES=0 bash scripts/train_redcube_pi.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

# ========== 策略与微调模式 ==========
POLICY_TYPE="${POLICY_TYPE:-pi0}"              # pi0 | pi05
FINETUNE_MODE="${FINETUNE_MODE:-lora_vlm}"     # lora_vlm | expert_only

case "${POLICY_TYPE}" in
  pi0)
    PRETRAINED_PATH="${PRETRAINED_PATH:-lerobot/pi0_base}"
    ;;
  pi05)
    PRETRAINED_PATH="${PRETRAINED_PATH:-lerobot/pi05_base}"
    ;;
  *)
    echo "错误: POLICY_TYPE 须为 pi0 或 pi05，当前: ${POLICY_TYPE}"
    exit 1
    ;;
esac

case "${FINETUNE_MODE}" in
  lora_vlm|expert_only) ;;
  *)
    echo "错误: FINETUNE_MODE 须为 lora_vlm 或 expert_only，当前: ${FINETUNE_MODE}"
    exit 1
    ;;
esac

# ========== 数据集 ==========
DATASET_REPO_ID="${DATASET_REPO_ID:-redcube}"
DATASET_ROOT="${DATASET_ROOT:-${ER_LQZ_ROOT}/data/${DATASET_REPO_ID}}"

# ========== 输出（勿 mkdir；lerobot-train 要求目录不存在或 resume=true）==========
BATCH_SIZE="${BATCH_SIZE:-8}"
OUTPUT_DIR="${OUTPUT_DIR:-${ER_LQZ_ROOT}/models/${POLICY_TYPE}_${DATASET_REPO_ID}_${FINETUNE_MODE}_bs${BATCH_SIZE}}"
JOB_NAME="${JOB_NAME:-${POLICY_TYPE}_${DATASET_REPO_ID}_${FINETUNE_MODE}_bs${BATCH_SIZE}}"
RESUME="${RESUME:-false}"

export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"

# ========== 训练规模（redcube ~10 万帧；Pi0 全模型较大，默认 bs=8）==========
SEED="${SEED:-1000}"
STEPS="${STEPS:-200000}"
SAVE_FREQ="${SAVE_FREQ:-40000}"
LOG_FREQ="${LOG_FREQ:-200}"
NUM_WORKERS="${NUM_WORKERS:-8}"
# 无仿真环境，设大值等效关闭 eval
EVAL_FREQ="${EVAL_FREQ:-1000000}"

POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
POLICY_REPO_ID="${POLICY_REPO_ID:-}"

# ---------- Pi0 / Pi0.5 微调 ----------
POLICY_DTYPE="${POLICY_DTYPE:-bfloat16}"
CHUNK_SIZE="${CHUNK_SIZE:-50}"
OPTIMIZER_LR="${OPTIMIZER_LR:-0.0002}"
SCHEDULER_DECAY_LR="${SCHEDULER_DECAY_LR:-0.00002}"
SCHEDULER_WARMUP_STEPS="${SCHEDULER_WARMUP_STEPS:-1000}"
SCHEDULER_DECAY_STEPS="${SCHEDULER_DECAY_STEPS:-100000}"

# LoRA（仅 lora_vlm 模式）
PEFT_R="${PEFT_R:-16}"
# 与 pi0_redcube_merged_quat_lora_vlm_bs8 一致：VLM 语言层 q/v + expert q/v + 动作投影
PEFT_TARGET_MODULES="${PEFT_TARGET_MODULES:-(.*paligemma\\.model\\.language_model.*\\.self_attn\\.(q|v)_proj|.*\\.gemma_expert\\..*\\.self_attn\\.(q|v)_proj|model\\.(state_proj|action_in_proj|action_out_proj|action_time_mlp_in|action_time_mlp_out))}"

# ---------- WandB ----------
WANDB_ENABLE="${WANDB_ENABLE:-true}"
WANDB_PROJECT="${WANDB_PROJECT:-fr3}"
WANDB_ENTITY="${WANDB_ENTITY:-}"
WANDB_NOTES="${WANDB_NOTES:-${POLICY_TYPE} ${FINETUNE_MODE} on ${DATASET_REPO_ID}}"
WANDB_MODE="${WANDB_MODE:-offline}"
WANDB_DISABLE_ARTIFACT="${WANDB_DISABLE_ARTIFACT:-true}"
WANDB_ADD_TAGS="${WANDB_ADD_TAGS:-true}"

EXTRA_ARGS=()
# ========================================

if [[ ! -f "${DATASET_ROOT}/meta/info.json" ]]; then
  echo "错误: 数据集不存在或 meta 不完整: ${DATASET_ROOT}"
  echo "请先合并: SOURCE_DATASETS=\"redcube_merged_quat redcube_add\" OUTPUT_REPO_ID=redcube bash scripts/merge_datasets.sh"
  exit 1
fi

if [[ "${RESUME}" != "true" && -d "${OUTPUT_DIR}" ]]; then
  echo "错误: 输出目录已存在: ${OUTPUT_DIR}"
  echo "  删除后重训，或 RESUME=true 断点续训"
  exit 1
fi

echo ">>> Pi0/Pi0.5 微调配置"
echo "  policy         : ${POLICY_TYPE}"
echo "  finetune_mode  : ${FINETUNE_MODE}"
echo "  pretrained     : ${PRETRAINED_PATH}"
echo "  dataset        : ${DATASET_ROOT}"
echo "  output         : ${OUTPUT_DIR}"
echo "  GPU            : ${CUDA_VISIBLE_DEVICES}"
echo "  steps/batch    : ${STEPS} / ${BATCH_SIZE}"
echo "  resume         : ${RESUME}"

CMD=(
  lerobot-train
  --policy.type="${POLICY_TYPE}"
  --policy.pretrained_path="${PRETRAINED_PATH}"
  --policy.device="${POLICY_DEVICE}"
  --policy.push_to_hub="${PUSH_TO_HUB}"
  --policy.dtype="${POLICY_DTYPE}"
  --policy.use_amp=false
  --policy.chunk_size="${CHUNK_SIZE}"
  --policy.n_action_steps="${CHUNK_SIZE}"
  --policy.gradient_checkpointing=true
  --policy.freeze_vision_encoder=true
  --policy.optimizer_lr="${OPTIMIZER_LR}"
  --policy.scheduler_warmup_steps="${SCHEDULER_WARMUP_STEPS}"
  --policy.scheduler_decay_steps="${SCHEDULER_DECAY_STEPS}"
  --policy.scheduler_decay_lr="${SCHEDULER_DECAY_LR}"
  --dataset.repo_id="${DATASET_REPO_ID}"
  --dataset.root="${DATASET_ROOT}"
  --output_dir="${OUTPUT_DIR}"
  --job_name="${JOB_NAME}"
  --resume="${RESUME}"
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

if [[ "${FINETUNE_MODE}" == "lora_vlm" ]]; then
  CMD+=(
    --policy.train_expert_only=false
    --peft.method_type=LORA
    --peft.r="${PEFT_R}"
    --peft.target_modules="${PEFT_TARGET_MODULES}"
  )
else
  CMD+=(
    --policy.train_expert_only=true
  )
fi

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
