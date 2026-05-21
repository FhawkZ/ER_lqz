#!/usr/bin/env bash
# Minimal bring-up for move_to_start on real FR3 with RT-friendly settings.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ER_LQZ_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROBOT_IP="${ROBOT_IP:-172.16.0.1}"
NAMESPACE="${NAMESPACE:-NS_1}"
CPU_SET="${CPU_SET:-24-31}"

# setup.bash 会读写尚未定义的 ament 变量；set -u 下会报 AMENT_TRACE_SETUP_FILES
_restore_u=0
[[ $- == *u* ]] && _restore_u=1 && set +u
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${ER_LQZ_ROOT}/franka_ros2_15/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${ER_LQZ_ROOT}/franka_ros2_15/install/setup.bash"
fi
[[ "${_restore_u}" == 1 ]] && set -u
unset _restore_u

LIBFRANKA_BIN="${ER_LQZ_ROOT}/franka_ros2_15/install/libfranka/bin"

echo "[1/5] automatic error recovery..."
"${LIBFRANKA_BIN}/error_recovery" "${ROBOT_IP}"

if [[ -w /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]]; then
  echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1 || true
fi

echo "[2/5] optional: pause heavy desktop processes (SIGSTOP, restored on exit)..."
DESKTOP_PIDS=()
for pattern in gnome-shell "cursor --type=zygote"; do
  while read -r pid _; do
    DESKTOP_PIDS+=("$pid")
    kill -STOP "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
done

cleanup() {
  for pid in "${DESKTOP_PIDS[@]}"; do
    kill -CONT "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "[3/5] launch move_to_start (taskset cpus=${CPU_SET})..."
taskset -c "${CPU_SET}" ros2 launch franka_bringup example.launch.py \
  controller_name:=move_to_start_example_controller \
  arm_id:=fr3 \
  use_fake_hardware:=false \
  robot_ip:="${ROBOT_IP}"
