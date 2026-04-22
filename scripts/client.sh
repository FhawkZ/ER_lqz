python -m lerobot.async_inference.robot_client \
  --robot.type=fr3_linker_l6_follower \
  --robot.cameras="{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}" \
  --server_address=172.18.5.61:8080 \
  --policy_type=act \
  --pretrained_name_or_path="/home/lqz/code/ER_lqz/models/redcube_act/080000/pretrained_model" \
  --actions_per_chunk=100 \
  --fps=30 \
  --task="pick the red cube and drop it in box" \
  --policy_device=cuda
