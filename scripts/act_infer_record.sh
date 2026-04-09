#!/usr/bin/env bash

lerobot-record \
  --robot.type=fr3_linker_l6_follower \
  --robot.cameras="{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}" \
  --policy.path="/home/lqz/下载/redcube_act/040000/pretrained_model" \
  --dataset.repo_id=local/eval_act_infer_fr3_l6 \
  --dataset.root=/home/lqz/code/ER_lqz/Data/act_infer_fr3_l6 \
  --dataset.fps=30 \
  --dataset.num_episodes=1 \
  --dataset.episode_time_s=120 \
  --dataset.reset_time_s=20 \
  --dataset.push_to_hub=false \
  --dataset.single_task="pick the red cube and drop it in box" \
  --display_data=true
