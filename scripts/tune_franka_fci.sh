#!/usr/bin/env bash
# Franka FCI 系统调优（保留桌面/GNOME，不强制关图形）
# 需要 root: sudo bash scripts/tune_franka_fci.sh [robot_nic]
set -euo pipefail

NIC="${1:-enp12s0}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "请使用: sudo bash $0 [网卡名]"
  exit 1
fi

echo ">>> CPU governor -> performance（全部核心）"
if command -v cpufreq-set >/dev/null; then
  for ((i = 0; i < $(nproc); i++)); do
    cpufreq-set -c "${i}" -g performance 2>/dev/null || true
  done
else
  for gov in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
    echo performance > "${gov}" 2>/dev/null || true
  done
fi

echo ">>> 持久化 performance（cpufrequtils）"
if [[ -d /etc/default ]]; then
  echo 'GOVERNOR=performance' > /etc/default/cpufrequtils
  systemctl disable ondemand 2>/dev/null || true
  systemctl enable cpufrequtils 2>/dev/null || true
  systemctl restart cpufrequtils 2>/dev/null || true
fi

echo ">>> UDP/网络缓冲区"
SYSCTL_FILE=/etc/sysctl.d/99-franka-fci.conf
cat > "${SYSCTL_FILE}" <<'EOF'
# Franka FCI — 降低 UDP 丢包与网络栈延迟
net.core.rmem_max = 12582912
net.core.wmem_max = 12582912
net.core.rmem_default = 12582912
net.core.wmem_default = 12582912
net.core.netdev_max_backlog = 30000
net.ipv4.udp_mem = 65536 131072 262144
EOF
sysctl --system >/dev/null 2>&1 || sysctl -p "${SYSCTL_FILE}"

if [[ -d "/sys/class/net/${NIC}" ]]; then
  echo ">>> ${NIC} qdisc -> pfifo_fast（降低 fq_codel 排队延迟）"
  tc qdisc replace dev "${NIC}" root pfifo_fast 2>/dev/null || \
    tc qdisc replace dev "${NIC}" root mq 2>/dev/null || true
  if command -v ethtool >/dev/null; then
    ethtool -K "${NIC}" gro off gso off tso off 2>/dev/null || true
  fi
else
  echo "警告: 网卡 ${NIC} 不存在，跳过 qdisc/ethtool"
fi

echo ">>> 完成。当前 governor 与 sysctl:"
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || true
sysctl net.core.rmem_max net.core.wmem_max 2>/dev/null || true
tc qdisc show dev "${NIC}" 2>/dev/null | head -1 || true
echo ""
echo "请运行: bash scripts/diagnose_franka_fci.sh"
echo "然后:   printf '\\n' | communication_test 172.16.0.1"
