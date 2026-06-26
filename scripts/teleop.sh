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

# 快速检查：sensor_msgs / geometry_msgs 必须从 conda py3.12 加载
python -c "
import geometry_msgs, sensor_msgs
for pkg in (geometry_msgs, sensor_msgs):
    p = pkg.__file__
    if 'python3.10' in p or p.startswith('/opt/ros/'):
        raise SystemExit(
            f'ROS 消息包路径错误: {pkg.__name__} -> {p}\n'
            '请运行: bash scripts/install_lerobot_ros_msgs.sh'
        )
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import rclpy
rclpy.init()
n = rclpy.create_node('_teleop_preflight')
n.create_publisher(JointState, '_preflight_hand', 1)
n.create_publisher(PoseStamped, '_preflight_pose', 1)
n.destroy_node()
rclpy.shutdown()
" || exit 1

FPS="${FPS:-60}"
CAMERAS='{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}'

exec lerobot-teleoperate \
  --teleop.type=mocap_retarget_leader \
  --robot.type=fr3_eef \
  --robot.cameras="${CAMERAS}" \
  --fps="${FPS}" \
  --display_data=false
