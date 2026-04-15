#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# Hugging Face Hub / 国内镜像配置
# 作用：解决国内下载/上传模型、数据集慢的问题
# -----------------------------------------------------------------------------
# HF镜像站点：使用国内镜像加速Hugging Face访问
# 不需要镜像时执行命令：unset HF_ENDPOINT 即可关闭
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"

# 解决conda与ROS2 Humble的Python包路径冲突
# 自动获取conda环境的site-packages路径，合并ROS2的Python路径，保证依赖正常加载
export CONDA_SITE_PACKAGES="$(python -c 'import site; print([p for p in site.getsitepackages() if "site-packages" in p][0])')"
export PYTHONPATH="$CONDA_SITE_PACKAGES:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"

# Hugging Face 身份验证令牌（上传数据集必须）
# 提前登录：huggingface-cli login
# 脚本直接读取环境变量中的HF_TOKEN，无需手动登录

# Hugging Face 本地缓存目录
# 所有模型、数据集缓存统一存放在此路径，避免占用系统盘
export HF_HOME="${HF_HOME:-/home/franka/lqz/hf}"

# -----------------------------------------------------------------------------
# 数据集录制参数配置（可通过环境变量覆盖，无需修改脚本主体）
# -----------------------------------------------------------------------------
# 全局键盘控制说明（录制时使用，窗口需要获得焦点）
# → ：提前结束当前录制段/进入复位阶段
# ← ：结束当前片段并重新录制
# Esc：停止整个录制脚本（不会录满设定的集数）
# 注意：误触会导致当前片段帧数极短或为0

# 数据集仓库ID
# 本地录制：可以写相对路径（如 Data/try）
# 云端上传：格式为 用户名/数据集名
DATASET_REPO_ID="${DATASET_REPO_ID:-franka_hand/redcube}"
# 可选示例：
# DATASET_REPO_ID="${DATASET_REPO_ID:-franka_hand/redcubett2}"

# 数据集本地存储的**父目录**
# DATASET_ROOT="${DATASET_ROOT:-/home/franka/lqz/Data}"

# 数据集实际存储路径 = 父目录 + 仓库ID
# DATASET_LOCAL_ROOT="${DATASET_LOCAL_ROOT:-${DATASET_ROOT%/}/${DATASET_REPO_ID}}"

# 是否自动上传到Hugging Face Hub（true=上传，false=只本地保存）
# PUSH_TO_HUB="${PUSH_TO_HUB:-false}"

# 云端数据集是否设为私有（true=私有，false=公开）
# DATASET_PRIVATE="${DATASET_PRIVATE:-false}"

# 如需给数据集添加标签，取消注释并修改下方内容
# --dataset.tags=\[fr3,LeRobot,linker_l6\] \

# -----------------------------------------------------------------------------
# LeRobot 数据集录制主命令（FR3机械臂+L6手爪+双相机）
# -----------------------------------------------------------------------------
lerobot-record \
    --robot.type=fr3_linker_l6_follower \
    --robot.cameras="{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}" \
    --teleop.type=mocap_leader \
    --dataset.repo_id="$DATASET_REPO_ID" \
    --dataset.root="$DATASET_LOCAL_ROOT" \
    --dataset.fps=30 \
    --dataset.num_episodes=10 \
    --dataset.reset_time_s=20 \
    --dataset.private="$DATASET_PRIVATE" \
    --dataset.single_task="pick the red cube and drop it in box" \
    --display_data=true
