#!/usr/bin/env bash
# 异步推理客户端（LeRobot 0.5.1 async_inference）
#
# 必须先启动策略服务端（在 GPU 机器上）:
#   HOST=0.0.0.0 PORT=8080 bash scripts/server.sh
# 再在本机:
#   SERVER_ADDRESS=<GPU机IP>:8080 bash scripts/client.sh
# 本机联调: SERVER_ADDRESS=127.0.0.1:8080（server 与 client 同机）
#
# SmolVLA 示例（POLICY_TYPE 会自动从 config.json 读取，也可显式指定）:
#   POLICY_PATH=${ER_LQZ_ROOT}/models/smolvla_redcube_merged_quat/checkpoints/060000/pretrained_model \
#   POLICY_TYPE=smolvla bash scripts/client.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

SERVER_ADDRESS="${SERVER_ADDRESS:-172.18.14.81:8080}"
_server_host="${SERVER_ADDRESS%:*}"
_server_port="${SERVER_ADDRESS##*:}"
if ! command -v nc >/dev/null 2>&1; then
  echo "警告: 未安装 nc，跳过 server 端口检查"
elif ! nc -zv -w 2 "${_server_host}" "${_server_port}" 2>&1 | grep -q succeeded; then
  echo "错误: 策略服务未就绪 ${SERVER_ADDRESS} (Connection refused)"
  echo "请在 GPU 机器执行: HOST=0.0.0.0 PORT=${_server_port} bash scripts/server.sh"
  exit 1
fi
unset _server_host _server_port

# 权重路径必须是 server 主机上的本地目录（client 只传字符串，server 侧加载）
POLICY_PATH="${POLICY_PATH:-${ER_LQZ_ROOT}/models/pi0_redcube_lora_vlm_bs8/checkpoints/160000/pretrained_model}"

# POLICY_TYPE 须与 checkpoint 的 config.json 中 "type" 一致；未设置时从 config.json 自动读取
if [[ -z "${POLICY_TYPE:-}" && -f "${POLICY_PATH}/config.json" ]]; then
  POLICY_TYPE="$(
    python -c "import json,sys; print(json.load(open(sys.argv[1]))['type'])" "${POLICY_PATH}/config.json"
  )"
fi
POLICY_TYPE="${POLICY_TYPE:-pi0}"

# 策略若按关节空间训练，请保持 fr3_linker_l6_follower；EE 空间策略才用 fr3_eef
# fr3_eef 默认 observation.state 为关节角+力矩；旧 EE 四元数 checkpoint 无需改 client，
# policy server 会按 checkpoint 的 state 布局自动从原始观测中取 ee_x/ori_q* 等字段。
# 若需显式录 legacy 13D EE 数据: --robot.arm_observation_rep=eef_quat
ROBOT_TYPE="${ROBOT_TYPE:-fr3_eef}"

CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

exec python -m lerobot.async_inference.robot_client \
  --robot.type="${ROBOT_TYPE}" \
  --robot.cameras="${CAMERAS}" \
  --server_address="${SERVER_ADDRESS}" \
  --policy_type="${POLICY_TYPE}" \
  --pretrained_name_or_path="${POLICY_PATH}" \
  --actions_per_chunk="${ACTIONS_PER_CHUNK:-50}" \
  --chunk_size_threshold="${CHUNK_SIZE_THRESHOLD:-0}" \
  --aggregate_fn_name="${AGGREGATE_FN:-conservative}" \
  --fps="${FPS:-30}" \
  --task="${TASK:-pick the red cube and drop it in box}" \
  --policy_device="${POLICY_DEVICE:-cuda}" \
  --client_device="${CLIENT_DEVICE:-cpu}"
