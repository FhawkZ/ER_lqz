#!/usr/bin/env bash
# 为 conda lerobot (Python 3.12) 安装与 rclpy 匹配的 ROS2 Humble 消息包（robostack）
# 避免从 /opt/ros/humble/.../python3.10 误导入导致 typesupport 失败
set -euo pipefail

ENV_NAME="${1:-lerobot}"
CHANNEL="${ROS_CONDA_CHANNEL:-robostack-staging}"

if ! command -v conda >/dev/null 2>&1; then
  echo "需要 conda"
  exit 1
fi

echo ">>> 在环境 ${ENV_NAME} 中安装 ROS2 消息包 (py3.12, ${CHANNEL})..."
conda install -n "${ENV_NAME}" -c "${CHANNEL}" -y \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-trajectory-msgs

echo ">>> 验证 import..."
conda run -n "${ENV_NAME}" python -c "
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
import geometry_msgs, sensor_msgs
assert 'site-packages' in geometry_msgs.__file__
assert 'site-packages' in sensor_msgs.__file__
print('geometry_msgs:', geometry_msgs.__file__)
print('sensor_msgs:', sensor_msgs.__file__)
print('OK: all message packages import from conda py3.12')
"

echo "完成。请重新: conda activate ${ENV_NAME} && bash scripts/teleop.sh"
