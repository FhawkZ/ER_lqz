#!/usr/bin/env bash
# 动捕遥操作：mocap_retarget_leader → fr3_eef（LeRobot 0.5.1）
# 臂：EE 增量（cartesian_impedance）；手：dex-retargeting → hand_0..hand_5 (0–255)
# 依赖: pip install dex-retargeting
#
# IK 臂版本见 scripts/teleop_ik.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

FPS="${FPS:-30}"
CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

exec lerobot-teleoperate \
  --teleop.type=mocap_retarget_leader \
  --robot.type=fr3_eef \
  --robot.cameras="${CAMERAS}" \
  --fps="${FPS}" \
  --display_data=true
