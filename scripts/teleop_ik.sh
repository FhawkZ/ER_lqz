#!/usr/bin/env bash
# 动捕遥操作：mocap_leader + fr3_linker_l6_follower（7 关节 IK 臂 + 几何手部映射）
# LeRobot 0.5.1；需要 pinocchio（conda 环境）

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh

CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

exec lerobot-teleoperate \
  --teleop.type=mocap_leader \
  --robot.type=fr3_linker_l6_follower \
  --robot.cameras="${CAMERAS}" \
  --fps="${FPS:-60}" \
  --display_data=true
