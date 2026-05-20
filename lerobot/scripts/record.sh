#!/usr/bin/env bash
# 与仓库根目录 scripts/record.sh 保持一致（LeRobot 0.5.1）
# 推荐在 ER_lqz 根目录执行: bash scripts/record.sh

set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "${ROOT}/scripts/record.sh" "$@"
