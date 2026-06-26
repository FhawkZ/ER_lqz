#!/usr/bin/env bash
# 将 offline wandb run 同步到 wandb.ai（需已 wandb login）
#
# 用法:
#   bash scripts/sync_wandb_offline.sh
#   RUN_DIR=models/pi0_redcube_merged_quat_lora_vlm_bs8/wandb/offline-run-xxx bash scripts/sync_wandb_offline.sh
#   WANDB_ENTITY=your_team bash scripts/sync_wandb_offline.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

RUN_DIR="${RUN_DIR:-${ER_LQZ_ROOT}/models/pi0_redcube_merged_quat_lora_vlm_bs8/wandb}"
PROJECT="${WANDB_PROJECT:-fr3}"

if ! command -v wandb >/dev/null 2>&1; then
  echo "错误: 未找到 wandb，请先 conda activate lerobot"
  exit 1
fi

shopt -s nullglob
runs=()
if [[ -d "${RUN_DIR}" && "$(basename "${RUN_DIR}")" == offline-run-* ]]; then
  runs=("${RUN_DIR}")
elif [[ -d "${RUN_DIR}" ]]; then
  runs=("${RUN_DIR}"/offline-run-*)
else
  echo "错误: 目录不存在 ${RUN_DIR}"
  exit 1
fi

if ((${#runs[@]} == 0)); then
  echo "错误: 未找到 offline-run-*  under ${RUN_DIR}"
  exit 1
fi

echo ">>> 同步 ${#runs[@]} 个 offline run 到 WandB (project=${PROJECT})"
for run in "${runs[@]}"; do
  echo "--- wandb sync ${run}"
  sync_args=(sync "${run}" --project "${PROJECT}")
  if [[ -n "${WANDB_ENTITY:-}" ]]; then
    sync_args+=(--entity "${WANDB_ENTITY}")
  fi
  wandb "${sync_args[@]}"
done

echo ">>> 完成。在 https://wandb.ai 打开 project '${PROJECT}' 查看 train/loss 曲线。"
