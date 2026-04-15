python -m lerobot.async_inference.robot_client \
  --robot.type=fr3_linker_l6_follower \
  --robot.cameras="{ handeye: {type: intelrealsense, serial_number_or_name: 242622071515, width: 640, height: 480, fps: 30}, fixed: {type: intelrealsense, serial_number_or_name: 242522071983, width: 640, height: 480, fps: 30}}" \
  --server_address=127.0.0.1:8080 \
  --policy_type=act \
  --pretrained_name_or_path="/home/lqz/下载/redcube_act/040000/pretrained_model" \
  --actions_per_chunk=100 \
  --fps=30 \
  --task="pick the red cube and drop it in box"
