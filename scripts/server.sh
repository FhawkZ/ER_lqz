#!/usr/bin/env bash
# 异步推理策略服务端（LeRobot 0.5.1）

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8080}"
FPS="${FPS:-30}"
TRACE_DIR="${TRACE_DIR:-${ER_LQZ_ROOT}/logs/inference_traces}"
RECORD_INFERENCE="${RECORD_INFERENCE:-true}"

mkdir -p "${TRACE_DIR}"

echo ">>> PolicyServer"
echo "  host              : ${HOST}:${PORT}"
echo "  record_inference  : ${RECORD_INFERENCE}"
echo "  trace_dir         : ${TRACE_DIR}  (每次启动新建 session_* 子目录)"

exec python -m lerobot.async_inference.policy_server \
  --host="${HOST}" \
  --port="${PORT}" \
  --fps="${FPS}" \
  --trace_dir="${TRACE_DIR}" \
  --record_inference="${RECORD_INFERENCE}"
