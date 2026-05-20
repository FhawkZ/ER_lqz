#!/usr/bin/env bash
# 将 ER_lqz 内嵌的 lerobot 升级到 HuggingFace 指定版本，并恢复 FR3 / mocap 定制。
#
# 用法:
#   bash scripts/upgrade_lerobot.sh              # 默认 v0.5.1
#   bash scripts/upgrade_lerobot.sh v0.5.1
#
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TAG="${1:-v0.5.1}"
CACHE="$ROOT/.upgrade_cache"
UPSTREAM="$CACHE/lerobot-${TAG#v}"
BACKUP="$CACHE/lerobot-backup-$(date +%Y%m%d_%H%M%S)"
LER="$ROOT/lerobot"

CUSTOM_DIRS=(
  src/lerobot/teleoperators/fr3_leader
  src/lerobot/teleoperators/mocap_leader
  src/lerobot/teleoperators/mocap_eef_leader
  src/lerobot/teleoperators/mocap_retarget
  src/lerobot/robots/fr3_follower
  src/lerobot/robots/fr3_eef
  src/lerobot/robots/fr3_linker_l6_follower
  src/lerobot/third_party
  rviz
)

echo "==> 下载 upstream ${TAG} ..."
mkdir -p "$CACHE"
if [[ ! -d "$UPSTREAM" ]]; then
  curl -sL "https://github.com/huggingface/lerobot/archive/refs/tags/${TAG}.tar.gz" \
    -o "$CACHE/lerobot-${TAG}.tar.gz"
  tar -xzf "$CACHE/lerobot-${TAG}.tar.gz" -C "$CACHE"
  # tarball 解压目录名可能是 lerobot-0.5.1
  if [[ ! -d "$UPSTREAM" ]]; then
    UPSTREAM="$CACHE/lerobot-${TAG#v}"
  fi
fi

echo "==> 备份当前 lerobot -> $BACKUP"
cp -a "$LER" "$BACKUP"

echo "==> 用 upstream ${TAG} 覆盖 lerobot（保留即将恢复的定制目录）..."
rsync -a --delete \
  --exclude '.upgrade_cache' \
  "$UPSTREAM/" "$LER/"

echo "==> 恢复定制目录 ..."
for d in "${CUSTOM_DIRS[@]}"; do
  if [[ -d "$BACKUP/$d" ]]; then
    mkdir -p "$(dirname "$LER/$d")"
    rsync -a "$BACKUP/$d/" "$LER/$d/"
    echo "  restored $d"
  fi
done

# 项目内 scripts（非 src）
for f in scripts/record.sh scripts/linkerhand_mocap scripts/merge_redcube_datasets.py; do
  if [[ -e "$BACKUP/$f" ]]; then
    mkdir -p "$(dirname "$LER/$f")"
    cp -a "$BACKUP/$f" "$LER/$f"
    echo "  restored $f"
  fi
done

echo "==> 应用 ER_lqz 集成补丁 ..."
python3 "$ROOT/scripts/apply_lerobot_overlay.py" "$LER" "$BACKUP"

echo ""
echo "完成。备份: $BACKUP"
echo "请在本机 conda 环境中重新安装: cd lerobot && pip install -e ."
echo "并验证: lerobot-teleoperate --help"
