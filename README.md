git add .
git commit -m ""
git push -u origin main

使用三个终端（每个终端都需要source /home/franka/lqz/franka_ros2_15/install/setup.bash），分别启动下面的指令，最后一个需要conda  activate lerobot_fr3_qz
leader臂启动示教模式。
ros2 launch franka_bringup franka.launch.py arm_id:=fr3 robot_ip:=172.16.0.1 namespace:=NS_2 load_gripper:=true
ros2 launch franka_fr3_moveit_config moveit.launch.py namespace:=NS_1 robot_ip:=172.16.0.2 use_fake_hardware:=false

lerobot-teleoperate --teleop.type=fr3_leader --robot.type=fr3_follower


下面指令用于复位，注意需要修改/home/franka/lqz/franka_ros2_15/src/franka_bringup/config/franka.config.yaml中的ip切换需要复位的机械臂
ros2 launch franka_bringup example.launch.py \
  controller_name:=move_to_start_example_controller \
  arm_id:=fr3 \
  use_fake_hardware:=false

下面是手+机械臂的版本
你当前这个 launch 文件里，hand_joint 参数是写死在文件中的。
要启动 L6，请先把这个文件改成：

文件：/home/franka/lqz/linker_hand_ros2_sdk/src/linkerhand-ros2-sdk/linker_hand_ros2_sdk/launch/linker_hand.launch.py
把 hand_joint 改成：
'hand_joint': "L6",
source linker_hand_ros2_sdk/install/setup.bash
sudo /usr/sbin/ip link set can0 up type can bitrate 1000000
ros2 launch linker_hand_ros2_sdk linker_hand.launch.py

source /opt/ros/humble/setup.bash
source ~/lqz/franka_ros2_15/install/setup.bash

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.1 \
  use_fake_hardware:=false \
  namespace:=NS_1

export CONDA_SITE_PACKAGES="$(python -c 'import site; print([p for p in site.getsitepackages() if "site-packages" in p][0])')"
export PYTHONPATH="$CONDA_SITE_PACKAGES:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"

lerobot-teleoperate \
  --teleop.type=mocap_leader \
  --robot.type=fr3_linker_l6_follower \
  --fps=60