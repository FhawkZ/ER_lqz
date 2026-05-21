#!/usr/bin/env bash
# Franka FCI 通信诊断（不关闭桌面环境）
set -euo pipefail

ROBOT_IP="${1:-172.16.0.1}"
NIC="${2:-enp12s0}"

echo "========== Franka FCI 诊断 =========="
echo "时间: $(date)"
echo "机械臂 IP: ${ROBOT_IP}"
echo ""

echo "--- 1. 先分清：内存充裕 ≠ CPU 实时性够 ---"
uptime
free -h | head -2
echo "说明: load average 长期 > 数核数量时，1ms 控制周期仍可能偶发超时。"
echo ""

echo "--- 2. CPU 调频（Franka 要求 performance）---"
if command -v cpufreq-info >/dev/null; then
  cpufreq-info 2>/dev/null | grep -E 'analyzing CPU 0|governor|current CPU frequency|maximum transition latency' || true
else
  echo "cpu0 governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"
fi
echo ""

echo "--- 3. 占用 CPU 的进程（看峰值/抢占，不是看内存）---"
ps aux --sort=-%cpu | head -8
echo ""

echo "--- 4. 机器人网卡 ${NIC} ---"
ip -br addr show "${NIC}" 2>/dev/null || echo "网卡 ${NIC} 不存在"
ip route get "${ROBOT_IP}" 2>/dev/null || true
if command -v ethtool >/dev/null; then
  ethtool -i "${NIC}" 2>/dev/null || true
  echo "提示: Realtek r8169 为消费级网卡，FCI 抖动常高于 Intel i210/i219。"
fi
if command -v tc >/dev/null; then
  echo "qdisc: $(tc qdisc show dev "${NIC}" 2>/dev/null | head -1)"
  echo "提示: fq_codel 会增加排队延迟，FCI 专用网口建议 pfifo_fast 或 mq。"
fi
echo ""

echo "--- 5. 网络缓冲区（默认过小会导致 UDP 丢包）---"
sysctl net.core.rmem_max net.core.wmem_max net.core.rmem_default net.core.wmem_default net.core.netdev_max_backlog 2>/dev/null
echo "Franka 建议: RTT + 控制循环 < 1ms；连续丢 20 包即 reflex。"
echo ""

echo "--- 6. 到 Desk/控制柜的 TCP 连接数（与 FCI UDP 抢栈资源）---"
DESK_CONN=$(ss -tn state established 2>/dev/null | grep -c "${ROBOT_IP}:443" || echo 0)
echo "HTTPS 到 ${ROBOT_IP}:443 的连接数: ${DESK_CONN}"
echo "说明: 保留 GNOME 桌面可以；浏览器里长期打开 Desk 网页会占用大量连接与 CPU。"
echo ""

echo "--- 7. 内核 ---"
uname -r
cat /sys/kernel/realtime 2>/dev/null && echo "PREEMPT_RT: 是" || echo "PREEMPT_RT: 否"
echo ""

echo "--- 8. libfranka 只读连接 ---"
LIBFRANKA_BIN="${LIBFRANKA_BIN:-/home/franka/lqz/franka_ros2_15/install/libfranka/bin/echo_robot_state}"
if [[ -x "${LIBFRANKA_BIN}" ]]; then
  timeout 2 "${LIBFRANKA_BIN}" "${ROBOT_IP}" 2>/dev/null | python3 -c "
import sys, json
d = json.loads(sys.stdin.readline())
print('robot_mode:', d.get('robot_mode'))
print('current_errors:', d.get('current_errors'))
print('last_motion_errors:', d.get('last_motion_errors'))
print('control_command_success_rate:', d.get('control_command_success_rate'))
" || echo "无法读取 robot state"
else
  echo "未找到 echo_robot_state: ${LIBFRANKA_BIN}"
fi
echo ""
echo "========== 建议下一步 =========="
echo "1) sudo bash scripts/tune_franka_fci.sh"
echo "2) 跑满 communication_test，看 Min 是否 ≥ 0.99"
echo "3) 若 Min 仍 < 0.99，优先考虑 Intel 千兆网卡直连控制柜"
