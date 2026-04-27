#!/usr/bin/env bash
# 动捕遥操作：mocap_eef_leader → fr3_eef（无 IK，直接发 EE 位姿）

export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"
export CONDA_SITE_PACKAGES="$(python -c 'import site; print([p for p in site.getsitepackages() if "site-packages" in p][0])')"
export PYTHONPATH="$CONDA_SITE_PACKAGES:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"

lerobot-teleoperate \
    --robot.type=fr3_eef \
    --robot.cameras="{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}" \
    --teleop.type=mocap_eef_leader \
    --fps=30 \
    --display_data=true
