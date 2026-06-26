#!/usr/bin/env bash
# 在本地 redcube_merged_quat 上微调 X-VLA（从 lerobot/xvla-base 出发）
#
# 数据集: 13D EE 四元数 + 6D hand，双相机 handeye/fixed（与 fr3_eef 一致）
# 依赖: pip install -e "${ER_LQZ_ROOT}/lerobot[xvla]"
#
# 用法:
#   conda activate lerobot
#   bash scripts/train_xvla_redcube_merged_quat.sh
#
# 断点续训:
#   RESUME=true bash scripts/train_xvla_redcube_merged_quat.sh
#
# 换 GPU:
#   CUDA_VISIBLE_DEVICES=1 bash scripts/train_xvla_redcube_merged_quat.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

# ========== 数据集（本地 root 必须指向含 meta/data/videos 的目录）==========
DATASET_REPO_ID="${DATASET_REPO_ID:-redcube_merged_quat}"
DATASET_ROOT="${DATASET_ROOT:-${ER_LQZ_ROOT}/data/${DATASET_REPO_ID}}"

# ========== 输出（勿 mkdir；lerobot-train 要求目录不存在或 resume=true）==========
OUTPUT_DIR="${OUTPUT_DIR:-${ER_LQZ_ROOT}/models/xvla_${DATASET_REPO_ID}}"
JOB_NAME="${JOB_NAME:-xvla_${DATASET_REPO_ID}}"
RESUME="${RESUME:-false}"

export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-1}"

# ========== 预训练底座 ==========
# 官方 Phase II 微调入口；可换成本地已下载目录
PRETRAINED_PATH="${PRETRAINED_PATH:-lerobot/xvla-base}"

# ========== 训练规模（~5 万帧、双 640x480 相机；0.9B 模型单卡建议 batch 4）==========
SEED="${SEED:-1000}"
STEPS="${STEPS:-100000}"
BATCH_SIZE="${BATCH_SIZE:-4}"
SAVE_FREQ="${SAVE_FREQ:-5000}"
LOG_FREQ="${LOG_FREQ:-100}"
NUM_WORKERS="${NUM_WORKERS:-4}"
# 仿真 eval 对本项目无意义，设大值等效关闭
EVAL_FREQ="${EVAL_FREQ:-1000000}"

POLICY_DEVICE="${POLICY_DEVICE:-cuda}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
POLICY_REPO_ID="${POLICY_REPO_ID:-}"

# ---------- X-VLA 微调（见 lerobot/docs/source/xvla.mdx）----------
# action_mode=auto: 自动适配 13 维 action，内部 pad 到 max_action_dim=20
# dtype=bfloat16: 官方推荐，显著省显存
# 不冻结 VLM，同时训练 policy transformer + soft prompts（Phase II 推荐）
POLICY_DTYPE="${POLICY_DTYPE:-bfloat16}"
ACTION_MODE="${ACTION_MODE:-auto}"
MAX_ACTION_DIM="${MAX_ACTION_DIM:-20}"
# 必须与 xvla-base 权重一致，不可改为 32（会导致 action_encoder shape mismatch）
MAX_STATE_DIM="${MAX_STATE_DIM:-20}"
CHUNK_SIZE="${CHUNK_SIZE:-32}"
# xvla-base 为 3 视角；redcube 仅 2 路相机，第 3 路由框架零填充
NUM_IMAGE_VIEWS="${NUM_IMAGE_VIEWS:-3}"
RESIZE_IMGS="${RESIZE_IMGS:-224,224}"
# redcube 相机名 -> xvla-base 预训练名；第 3 路 image3 由 num_image_views 零填充
RENAME_MAP="${RENAME_MAP:-{\"observation.images.handeye\":\"observation.images.image\",\"observation.images.fixed\":\"observation.images.image2\"}}"

# ---------- WandB ----------
WANDB_ENABLE="${WANDB_ENABLE:-true}"
WANDB_PROJECT="${WANDB_PROJECT:-fr3}"
WANDB_ENTITY="${WANDB_ENTITY:-}"
WANDB_NOTES="${WANDB_NOTES:-xvla finetune on fr3_eef redcube quat}"
WANDB_MODE="${WANDB_MODE:-offline}"
WANDB_DISABLE_ARTIFACT="${WANDB_DISABLE_ARTIFACT:-true}"
WANDB_ADD_TAGS="${WANDB_ADD_TAGS:-true}"

EXTRA_ARGS=()
# ========================================

if [[ ! -f "${DATASET_ROOT}/meta/info.json" ]]; then
  echo "错误: 数据集不存在或 meta 不完整: ${DATASET_ROOT}"
  exit 1
fi

if [[ "${RESUME}" != "true" && -d "${OUTPUT_DIR}" ]]; then
  echo "错误: 输出目录已存在: ${OUTPUT_DIR}"
  echo "  删除后重训，或 RESUME=true 断点续训"
  exit 1
fi

echo ">>> X-VLA 微调配置"
echo "  pretrained   : ${PRETRAINED_PATH}"
echo "  dataset      : ${DATASET_ROOT}"
echo "  output       : ${OUTPUT_DIR}"
echo "  GPU          : ${CUDA_VISIBLE_DEVICES}"
echo "  steps/batch  : ${STEPS} / ${BATCH_SIZE}"
echo "  action_mode  : ${ACTION_MODE} (dataset 13D -> pad ${MAX_ACTION_DIM}D)"
echo "  cameras      : ${NUM_IMAGE_VIEWS} (handeye->image, fixed->image2, image3 padded)"
echo "  rename_map   : ${RENAME_MAP}"
echo "  resume       : ${RESUME}"

CMD=(
  lerobot-train
  --policy.path="${PRETRAINED_PATH}"
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
  --policy.freeze_vision_encoder=false
  --policy.freeze_language_encoder=false
  --policy.train_policy_transformer=true
  --policy.train_soft_prompts=true
  --rename_map="${RENAME_MAP}"
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
