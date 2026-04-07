#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Hugging Face Hub / 镜像
# -----------------------------------------------------------------------------
# 端点：原先 `http:hf-mirror.com` 无效，镜像需带协议。不需要镜像时执行: unset HF_ENDPOINT
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"
export CONDA_SITE_PACKAGES="$(python -c 'import site; print([p for p in site.getsitepackages() if "site-packages" in p][0])')"
export PYTHONPATH="$CONDA_SITE_PACKAGES:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"

# 上传到 Hub 前需登录（任选其一）：
#   huggingface-cli login
export HF_TOKEN="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"
# huggingface_hub 会读取 HF_TOKEN 或 HUGGING_FACE_HUB_TOKEN

# Hugging Face 本地缓存目录（指定到当前项目）
export HF_HOME="${HF_HOME:-/home/franka/lqz/hf}"

# -----------------------------------------------------------------------------
# 数据集与上传（可通过环境变量覆盖，无需改脚本主体）
# -----------------------------------------------------------------------------
# 录制时全局键盘（pynput，焦点需在能收到键的窗口）：→ 提前结束「当前」录制/复位段；← 结束并重录本集；
# Esc 停止整个脚本（不会自动录满 num_episodes）。误触 → 可能导致本集极短或 0 帧。
# -----------------------------------------------------------------------------
# 本地录制：repo_id 可为相对路径（如 Data/try）或 Hub 的 user/name
DATASET_REPO_ID="${DATASET_REPO_ID:-franka_hand/redcube14}"
# 本地「父目录」；实际写入路径为 父目录/repo_id（LeRobot 的 --dataset.root 必须是数据集根目录，不会自动拼 repo_id）
DATASET_ROOT="${DATASET_ROOT:-/home/franka/lqz/Data}"
DATASET_LOCAL_ROOT="${DATASET_LOCAL_ROOT:-${DATASET_ROOT%/}/${DATASET_REPO_ID}}"
PUSH_TO_HUB="${PUSH_TO_HUB:-false}"
DATASET_PRIVATE="${DATASET_PRIVATE:-false}"
# 接续采集（往同一数据集目录追加新 episode）：必须与已有数据使用相同的 DATASET_REPO_ID、DATASET_LOCAL_ROOT、
# --dataset.fps，且机器人/相机特征一致。首次录制用 false；接着录同一套数据用 true。
RESUME="${RESUME:-false}"
# 上传到 Hub 时如需自定义标签，可在下面命令中追加一行（注意方括号转义）:
#   --dataset.tags=\[fr3,LeRobot,linker_l6\] \

lerobot-record \
    --robot.type=fr3_linker_l6_follower \
    --robot.cameras="{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}" \
    --teleop.type=mocap_leader \
    --dataset.repo_id="$DATASET_REPO_ID" \
    --dataset.root="$DATASET_LOCAL_ROOT" \
    --dataset.fps=30 \
    --dataset.num_episodes=10 \
    --dataset.reset_time_s=20 \
    --dataset.push_to_hub="$PUSH_TO_HUB" \
    --dataset.private="$DATASET_PRIVATE" \
    --dataset.single_task="pick the red cube and drop it in box" \
    --display_data=true \
    --resume="$RESUME"
