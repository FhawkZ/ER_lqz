#!/usr/bin/env bash
# LeRobot 0.5.x + ROS2 Humble 公共环境（被其他 scripts/*.sh source）
# 用法: source scripts/env_lerobot.sh

if [[ -n "${BASH_SOURCE[0]:-}" ]]; then
  _ENV_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
  _ENV_SCRIPT="$(cd "$(dirname "$0")" && pwd)"
fi
export ER_LQZ_ROOT="$(cd "${_ENV_SCRIPT}/.." && pwd)"

# 默认遥操作/采集配对（与 MocapRetargetLeaderConfig + FR3EEFConfig 一致）
export LEROBOT_TELEOP_TYPE="${LEROBOT_TELEOP_TYPE:-mocap_retarget_leader}"
export LEROBOT_ROBOT_TYPE="${LEROBOT_ROBOT_TYPE:-fr3_eef}"

# Hugging Face 镜像（不需要时: unset HF_ENDPOINT）
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"
export HF_HOME="${HF_HOME:-/home/franka/lqz/hf}"

# ROS2 + colcon 安装的 lerobot（优先）；否则使用源码路径
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${ER_LQZ_ROOT}/lerobot/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ER_LQZ_ROOT}/lerobot/install/setup.bash"
fi

# conda site-packages 优先于 ROS，避免 pinocchio / numpy 版本冲突
if command -v python >/dev/null 2>&1; then
  export CONDA_SITE_PACKAGES="$(
    python -c 'import site; print([p for p in site.getsitepackages() if "site-packages" in p][0])' 2>/dev/null \
      || echo "${CONDA_PREFIX}/lib/python3.12/site-packages"
  )"
else
  export CONDA_SITE_PACKAGES="${CONDA_PREFIX}/lib/python3.12/site-packages"
fi
export PYTHONPATH="${CONDA_SITE_PACKAGES}:${ER_LQZ_ROOT}/lerobot/src:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"

unset _ENV_SCRIPT
