#!/usr/bin/env bash
# 验证 Pi0 LoRA checkpoint 能否按 policy_server 同款逻辑加载，并在数据集上跑通前向。
#
#   bash scripts/verify_pi0_checkpoint_load.sh
#   CHECKPOINT=.../020000/pretrained_model DEVICE=cuda:0 bash scripts/verify_pi0_checkpoint_load.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

CHECKPOINT="${CHECKPOINT:-${ER_LQZ_ROOT}/models/pi0_redcube_merged_quat_lora_vlm_bs8/checkpoints/080000/pretrained_model}"
DATASET_ROOT="${DATASET_ROOT:-${ER_LQZ_ROOT}/data/redcube_merged_quat}"
DEVICE="${DEVICE:-cuda}"

extra_args=()
if [[ "${SKIP_FORWARD:-false}" == "true" ]]; then
  extra_args+=(--skip-forward)
fi

exec python "${SCRIPT_DIR}/verify_pi0_checkpoint_load.py" \
  --checkpoint "${CHECKPOINT}" \
  --dataset-root "${DATASET_ROOT}" \
  --device "${DEVICE}" \
  "${extra_args[@]}"
