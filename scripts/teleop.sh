#!/usr/bin/env bash
# 动捕遥操作（LeRobot 0.5.1）
#
# 默认: mocap_retarget_leader + fr3_eef（dex 手部 + 笛卡尔 EE 臂）
# 可选环境变量:
#   TELEOP_TYPE=mocap_eef_leader     # 几何弯曲映射手部
#   TELEOP_TYPE=mocap_leader         # 需配合 ROBOT_TYPE=fr3_linker_l6_follower（IK 臂）
#   ROBOT_TYPE=fr3_linker_l6_follower
#   FPS=30
#
# dex 模式需: pip install dex-retargeting

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

TELEOP_TYPE="${TELEOP_TYPE:-mocap_retarget_leader}"
ROBOT_TYPE="${ROBOT_TYPE:-fr3_eef}"
FPS="${FPS:-30}"

CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

exec lerobot-teleoperate \
  --teleop.type="${TELEOP_TYPE}" \
  --robot.type="${ROBOT_TYPE}" \
  --robot.cameras="${CAMERAS}" \
  --fps="${FPS}" \
  --display_data=true
