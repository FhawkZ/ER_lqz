#!/usr/bin/env bash
# 为 conda lerobot (Python 3.12) 安装 rclpy + ROS2 Humble 消息包（robostack）
# 避免从 /opt/ros/humble/.../python3.10 误导入导致 typesupport 失败
#
# 注意: conda-forge 的 ffmpeg=7.1.1 与 robostack rclpy 的 libprotobuf 冲突，
# 若环境中已装 conda ffmpeg，本脚本会先移除它（record 流式编码可改用系统 ffmpeg 或 imageio-ffmpeg）。
set -euo pipefail

ENV_NAME="${1:-lerobot}"
CHANNEL="${ROS_CONDA_CHANNEL:-robostack-staging}"

if ! command -v conda >/dev/null 2>&1; then
  echo "需要 conda"
  exit 1
fi

if conda list -n "${ENV_NAME}" ffmpeg 2>/dev/null | grep -q "^ffmpeg "; then
  _ff_ver="$(conda list -n "${ENV_NAME}" ffmpeg | awk '/^ffmpeg /{print $2}')"
  if [[ "${_ff_ver}" == 7.1.* ]]; then
    echo ">>> 检测到 conda ffmpeg ${_ff_ver}，与 py3.12 rclpy 的 libprotobuf 冲突，先移除..."
    conda remove -n "${ENV_NAME}" -y ffmpeg
  else
    echo ">>> 已存在 conda ffmpeg ${_ff_ver}，跳过移除"
  fi
fi
unset _ff_ver 2>/dev/null || true

echo ">>> 在环境 ${ENV_NAME} 中安装 rclpy + ROS2 消息包 (py3.12, conda-forge + ${CHANNEL})..."
conda install -n "${ENV_NAME}" -c conda-forge -c "${CHANNEL}" -y \
  empy \
  "libprotobuf=6.33.5" \
  "ros2-distro-mutex=0.9" \
  ros-humble-rclpy \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-sensor-msgs \
  ros-humble-trajectory-msgs

echo ">>> 验证 import..."
conda run -n "${ENV_NAME}" python -c "
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
import geometry_msgs, sensor_msgs
for pkg in (rclpy, geometry_msgs, sensor_msgs):
    p = pkg.__file__
    if 'python3.10' in p or p.startswith('/opt/ros/'):
        raise SystemExit(f'ROS 包路径错误: {pkg.__name__} -> {p}')
    assert 'site-packages' in p, p
print('rclpy:', rclpy.__file__)
print('geometry_msgs:', geometry_msgs.__file__)
print('sensor_msgs:', sensor_msgs.__file__)
rclpy.init()
n = rclpy.create_node('_install_lerobot_ros_msgs_check')
n.create_publisher(JointState, '_preflight_hand', 1)
n.create_publisher(PoseStamped, '_preflight_pose', 1)
n.destroy_node()
rclpy.shutdown()
print('OK: rclpy + message packages import from conda py3.12')
"

echo "完成。请: conda activate ${ENV_NAME} && source scripts/env_lerobot.sh"
