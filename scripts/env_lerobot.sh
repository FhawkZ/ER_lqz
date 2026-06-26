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
# franka 机器默认缓存；不可写或不存在时落到项目内 .cache/hf（SmolVLA 需拉 VLM 配置）
if [[ -d /home/franka/lqz/hf && -w /home/franka/lqz/hf ]]; then
  export HF_HOME="${HF_HOME:-/home/franka/lqz/hf}"
elif [[ -n "${HF_HOME:-}" && -d "${HF_HOME}" && -w "${HF_HOME}" ]]; then
  :
else
  export HF_HOME="${ER_LQZ_ROOT}/.cache/hf"
  mkdir -p "${HF_HOME}"
fi

# ROS2 + colcon 安装的 lerobot（优先）；否则使用源码路径
# setup.bash 会读写尚未定义的 ament 变量；调用方若 set -u 会报错，需临时关闭 nounset
_lerobot_restore_u=0
[[ $- == *u* ]] && _lerobot_restore_u=1 && set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${ER_LQZ_ROOT}/lerobot/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ER_LQZ_ROOT}/lerobot/install/setup.bash"
fi
[[ "${_lerobot_restore_u}" == 1 ]] && set -u
unset _lerobot_restore_u

# scipy/transformers 需要较新 libstdc++（CXXABI_1.3.15）；系统 /lib 往往过旧
if [[ -n "${CONDA_PREFIX:-}" && -d "${CONDA_PREFIX}/lib" ]]; then
  case ":${LD_LIBRARY_PATH:-}:" in
    *:"${CONDA_PREFIX}/lib":*) ;;
    *) export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
  esac
fi

# conda site-packages 优先于 ROS，避免 pinocchio / numpy 版本冲突
_conda_prefix="${CONDA_PREFIX:-}"
if command -v python >/dev/null 2>&1; then
  export CONDA_SITE_PACKAGES="$(
    python -c 'import site; print([p for p in site.getsitepackages() if "site-packages" in p][0])' 2>/dev/null \
      || echo "${_conda_prefix}/lib/python3.12/site-packages"
  )"
else
  export CONDA_SITE_PACKAGES="${_conda_prefix}/lib/python3.12/site-packages"
fi
unset _conda_prefix

# conda 的 rclpy 为 py3.12；source setup.bash 会注入 python3.10 的 *_msgs，导致 typesupport 失败
_lerobot_filter_py310_from_pythonpath() {
  local IFS=':'
  local _p _out=()
  for _p in ${PYTHONPATH:-}; do
    [[ -z "${_p}" ]] && continue
    case "${_p}" in
      */python3.10/*|*/python3.10/dist-packages*|*/python3.10/site-packages*)
        continue
        ;;
    esac
    _out+=("${_p}")
  done
  if ((${#_out[@]})); then
    PYTHONPATH="$(IFS=:; echo "${_out[*]}")"
  else
    unset PYTHONPATH
  fi
}
_lerobot_filter_py310_from_pythonpath
unset -f _lerobot_filter_py310_from_pythonpath
export PYTHONPATH="${ER_LQZ_ROOT}/lerobot/src:${CONDA_SITE_PACKAGES}${PYTHONPATH:+:${PYTHONPATH}}"

unset _ENV_SCRIPT
