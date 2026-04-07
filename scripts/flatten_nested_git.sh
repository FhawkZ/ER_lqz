#!/usr/bin/env bash
# 将工作区内嵌套的 Git 仓库改为普通目录，便于单一父仓库跟踪全部文件。
# 用法：在仓库根目录执行: bash scripts/flatten_nested_git.sh
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

remove_nested_git() {
  local target="$1"
  if [[ -e "$target" ]]; then
    rm -rf "$target"
    echo "removed: $target"
  fi
}

# franka_ros2_15 内 vendored 包
remove_nested_git "$ROOT/franka_ros2_15/src/franka_description/.git"
remove_nested_git "$ROOT/franka_ros2_15/src/libfranka/.git"
[[ -f "$ROOT/franka_ros2_15/src/libfranka/common/.git" ]] && rm -f "$ROOT/franka_ros2_15/src/libfranka/common/.git" && echo "removed: .../libfranka/common/.git (gitlink)"
[[ -f "$ROOT/franka_ros2_15/src/libfranka/.gitmodules" ]] && rm -f "$ROOT/franka_ros2_15/src/libfranka/.gitmodules" && echo "removed: .../libfranka/.gitmodules"

# 其他子项目
remove_nested_git "$ROOT/mocap_ros_py/.git"
remove_nested_git "$ROOT/lerobot/.git"
remove_nested_git "$ROOT/linker_hand_ros2_sdk/src/linkerhand-ros2-sdk/.git"

echo "Done. 请勿删除 $ROOT/.git 。若索引里曾加入 submodule，可执行:"
echo "  git rm -rf --cached <路径>   # 再 git add ."
