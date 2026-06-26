#!/usr/bin/env bash
# 在 redcube_merged_quat 上微调 X-VLA（lerobot/xvla-base）
#
# 数据集: FR3 EEF + Linker L6，13D 动作 (xyz + quat + 6 hand)，双相机 handeye/fixed
# 约 100 episodes / 52k frames，任务: "pick the red cube and drop it in box"
#
# 依赖（首次使用前）:
#   conda activate lerobot
#   pip install -e "${ER_LQZ_ROOT}/lerobot[xvla]"
#
# 用法:
#   bash scripts/train_redcube_xvla.sh
#
# 显存不足时:
#   LOW_VRAM=1 bash scripts/train_redcube_xvla.sh
#
# 指定 GPU:
#   CUDA_VISIBLE_DEVICES=1 bash scripts/train_redcube_xvla.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

# ========== 常用参数（按需改 / 环境变量覆盖）==========
# 只需 --policy.path；type 会从 checkpoint config 自动加载，不可与 path 同时指定
POLICY_PATH="${POLICY_PATH:-lerobot/xvla-base}"

DATASET_REPO_ID="${DATASET_REPO_ID:-redcube_merged_quat}"
DATASET_ROOT="${DATASET_ROOT:-${ER_LQZ_ROOT}/data/${DATASET_REPO_ID}}"

OUTPUT_DIR="${OUTPUT_DIR:-${ER_LQZ_ROOT}/models/xvla_${DATASET_REPO_ID}}"
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-1}"

SEED="${SEED:-1000}"
# ~52k frames；batch=4 时约 13k steps/epoch，20k steps ≈ 1.5 epoch（官方微调推荐量级）
STEPS="${STEPS:-100000}"
_DEFAULT_BATCH=4
_DEFAULT_CHUNK=32
if [[ "${LOW_VRAM:-0}" == "1" ]]; then
  _DEFAULT_BATCH=2
  _DEFAULT_CHUNK=16
fi
BATCH_SIZE="${BATCH_SIZE:-${_DEFAULT_BATCH}}"
EVAL_FREQ="${EVAL_FREQ:-5000}"
SAVE_FREQ="${SAVE_FREQ:-20000}"
NUM_WORKERS="${NUM_WORKERS:-4}"
LOG_FREQ="${LOG_FREQ:-200}"
unset _DEFAULT_BATCH

POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
POLICY_REPO_ID="${POLICY_REPO_ID:-}"
JOB_NAME="${JOB_NAME:-xvla_${DATASET_REPO_ID}}"

# X-VLA 微调关键参数（见 lerobot/docs/source/xvla.mdx）
POLICY_DTYPE="${POLICY_DTYPE:-bfloat16}"
# auto: 自动识别 13D 动作，内部 pad 到 max_action_dim=20 以兼容预训练权重
ACTION_MODE="${ACTION_MODE:-auto}"
MAX_ACTION_DIM="${MAX_ACTION_DIM:-20}"
# 必须与 xvla-base 一致（20），否则 action_encoder 权重无法加载
# checkpoint: dim_action(20)+dim_time(32)+dim_proprio(20)=72；改成 32 会变成 84
MAX_STATE_DIM="${MAX_STATE_DIM:-20}"
CHUNK_SIZE="${CHUNK_SIZE:-${_DEFAULT_CHUNK:-32}}"
# xvla-base 预训练为 3 视角（不足时零填充）；redcube 有 2 路相机，第 3 路自动 pad
NUM_IMAGE_VIEWS="${NUM_IMAGE_VIEWS:-3}"
RESIZE_IMGS="${RESIZE_IMGS:-224,224}"
# redcube 相机名 -> xvla-base 预训练名；第 3 路 image3 由 num_image_views 零填充
RENAME_MAP="${RENAME_MAP:-{\"observation.images.handeye\":\"observation.images.image\",\"observation.images.fixed\":\"observation.images.image2\"}}"
unset _DEFAULT_CHUNK
# Phase II 推荐：不冻结 VLM，同时训练 policy transformer + soft prompts
FREEZE_VISION="${FREEZE_VISION:-false}"
FREEZE_LANGUAGE="${FREEZE_LANGUAGE:-false}"
TRAIN_TRANSFORMER="${TRAIN_TRANSFORMER:-true}"
TRAIN_SOFT_PROMPTS="${TRAIN_SOFT_PROMPTS:-true}"
# 学习率 schedule 与训练步数对齐（XVLA 默认 warmup=1000, decay_lr=2.5e-6）
SCHEDULER_DECAY_STEPS="${SCHEDULER_DECAY_STEPS:-${STEPS}}"

# ---------- WandB ----------
WANDB_ENABLE="${WANDB_ENABLE:-true}"
WANDB_PROJECT="${WANDB_PROJECT:-fr3}"
WANDB_ENTITY="${WANDB_ENTITY:-}"
WANDB_NOTES="${WANDB_NOTES:-xvla finetune on redcube_merged_quat}"
WANDB_MODE="${WANDB_MODE:-offline}"
WANDB_DISABLE_ARTIFACT="${WANDB_DISABLE_ARTIFACT:-true}"
WANDB_ADD_TAGS="${WANDB_ADD_TAGS:-true}"

EXTRA_ARGS=()
# ========================================

if [[ ! -f "${DATASET_ROOT}/meta/info.json" ]]; then
  echo "错误: 数据集不存在或 meta 不完整: ${DATASET_ROOT}"
  echo "请先合并/迁移数据集:"
  echo "  bash scripts/merge_datasets.sh"
  echo "  python scripts/migrate_fr3_eef_euler_to_quat.py  # 若尚未生成 quat 版本"
  exit 1
fi

echo ">>> X-VLA 微调配置"
echo "  policy.path  : ${POLICY_PATH}"
echo "  dataset      : ${DATASET_ROOT}"
echo "  output       : ${OUTPUT_DIR}"
echo "  GPU          : ${CUDA_VISIBLE_DEVICES}"
echo "  steps        : ${STEPS}"
echo "  batch_size   : ${BATCH_SIZE}"
echo "  chunk_size   : ${CHUNK_SIZE}"
echo "  action_mode  : ${ACTION_MODE} (13D → pad ${MAX_ACTION_DIM}D)"
echo "  cameras      : ${NUM_IMAGE_VIEWS} (handeye->image, fixed->image2, image3 padded)"
echo "  rename_map   : ${RENAME_MAP}"
echo "  dtype        : ${POLICY_DTYPE}"

CMD=(
  lerobot-train
  --policy.path="${POLICY_PATH}"
  --policy.device="${POLICY_DEVICE}"
  --policy.push_to_hub="${PUSH_TO_HUB}"
  --policy.dtype="${POLICY_DTYPE}"
  --policy.action_mode="${ACTION_MODE}"
  --policy.max_action_dim="${MAX_ACTION_DIM}"
  --policy.max_state_dim="${MAX_STATE_DIM}"
  --policy.chunk_size="${CHUNK_SIZE}"
  --policy.n_action_steps="${CHUNK_SIZE}"
  --policy.num_image_views="${NUM_IMAGE_VIEWS}"
  --policy.resize_imgs_with_padding="[${RESIZE_IMGS}]"
  --policy.freeze_vision_encoder="${FREEZE_VISION}"
  --policy.freeze_language_encoder="${FREEZE_LANGUAGE}"
  --policy.train_policy_transformer="${TRAIN_TRANSFORMER}"
  --policy.train_soft_prompts="${TRAIN_SOFT_PROMPTS}"
  --policy.scheduler_decay_steps="${SCHEDULER_DECAY_STEPS}"
  --rename_map="${RENAME_MAP}"
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
