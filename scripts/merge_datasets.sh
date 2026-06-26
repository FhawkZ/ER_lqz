#!/usr/bin/env bash
# 合并多个本地 LeRobot 数据集（v3.0）
#
# 常用：直接运行（默认合并 data/redcube1..4 -> data/redcube_merged）
#   bash scripts/merge_datasets.sh
#
# 更换源数据集 / 输出名（环境变量，一行改完）：
#   SOURCE_DATASETS="redcube1 redcube2 redcube5" OUTPUT_REPO_ID=redcube_subset bash scripts/merge_datasets.sh
#
# 覆盖已有输出：
#   OVERWRITE=true bash scripts/merge_datasets.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

# ---------- 可改配置（也可用环境变量覆盖） ----------
DATASET_ROOT="${DATASET_ROOT:-${ER_LQZ_ROOT}/data}"
SOURCE_DATASETS="${SOURCE_DATASETS:-redcube1 redcube2 redcube3 redcube4}"
OUTPUT_REPO_ID="${OUTPUT_REPO_ID:-redcube_merged}"
OUTPUT_DIR="${OUTPUT_DIR:-}"          # 留空则写到 ${DATASET_ROOT}/${OUTPUT_REPO_ID}
OVERWRITE="${OVERWRITE:-false}"
# ----------------------------------------------------

extra_args=()
if [[ "${OVERWRITE}" == "true" ]]; then
  extra_args+=(--overwrite)
fi
if [[ -n "${OUTPUT_DIR}" ]]; then
  extra_args+=(--output-dir "${OUTPUT_DIR}")
fi

# shellcheck disable=SC2206
repo_ids=(${SOURCE_DATASETS})

exec python "${SCRIPT_DIR}/merge_datasets.py" \
  --dataset-root "${DATASET_ROOT}" \
  --repo-ids "${repo_ids[@]}" \
  --output-repo-id "${OUTPUT_REPO_ID}" \
  "${extra_args[@]}"
