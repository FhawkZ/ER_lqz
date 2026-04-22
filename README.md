# 项目使用说明

## 1、代码目录说明

- `franka_ros2_15/`
  - Franka FR3 的 ROS2 工作区，包含底层驱动、MoveIt 配置、启动文件等。
  - 主要负责机械臂连接、状态发布、控制器加载与运动规划。

- `lerobot/`[^developer]
  - LeRobot 主体代码与 CLI（例如 `lerobot-teleoperate`、`lerobot-record`）。
  - 主要负责遥操作映射、数据录制与后续训练/评测链路。

- `linker_hand_ros2_sdk/`
  - 灵巧手（如 L6）ROS2 SDK 与驱动代码。
  - 主要负责手部 CAN 通信、状态读写和手爪控制节点。

- `mocap_ros_py/`
  - 动捕相关 Python 脚本与桥接程序。
  - 主要负责将动捕输入转换为 ROS/机器人可用控制信号。

- `scripts/`
  - 项目辅助脚本（当前主要是采集脚本）。
  - `scripts/record.sh` 用于启动标准化数据采集流程。

## 2、使用说明

### 2.1 硬件启动

#### A) franka 启动（机械臂）

建议使用单独终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/franka/lqz/franka_ros2_15/install/setup.bash
```

启动 follower 侧（常用）：

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=172.16.0.1 \
  use_fake_hardware:=false \
  namespace:=NS_1
```
################# 笛卡尔空间
```bash
source install/setup.bash
ros2 launch serl_franka_controllers cartesian_impedance_controller.launch.py

source install/setup.bash
ros2 topic pub /NS_1/cartesian_impedance_controller/equilibrium_pose \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'fr3_link0'}, pose: {position: {x: 0.3, y: 0.0, z: 0.4}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

若需要 leader 臂示教模式，可额外启动：

```bash
ros2 launch franka_bringup franka.launch.py arm_id:=fr3 robot_ip:=172.16.0.1 namespace:=NS_2 load_gripper:=true
```

> 复位指令（按需）：

```bash
ros2 launch franka_bringup example.launch.py \
  controller_name:=move_to_start_example_controller \
  arm_id:=fr3 \
  use_fake_hardware:=false
```

#### B) hand 启动（灵巧手）

先确认 `linker_hand_ros2_sdk/launch/linker_hand.launch.py` 中 `hand_joint` 设置为目标型号（例如 `"L6"`），再执行：

```bash
source /home/franka/lqz/linker_hand_ros2_sdk/install/setup.bash
sudo /usr/sbin/ip link set can0 up type can bitrate 1000000
ros2 launch linker_hand_ros2_sdk linker_hand.launch.py
```

### 2.2 遥操作

在遥操作终端中，先激活 conda 环境并保证 pinocchio 优先使用 conda 包[^pythonpath]：

```bash
conda activate lerobot_fr3_qz
# export PYTHONPATH="$(python -c 'import site; print(next(p for p in site.getsitepackages() if "site-packages" in p))')${PYTHONPATH:+:$PYTHONPATH}" 如果下面这一段不行再切换成这句
export PYTHONPATH="$CONDA_PREFIX/lib/python3.10/site-packages:$PYTHONPATH"
```

运行动捕遥操作：

如果遇到 `pinocchio` 仍然加载到 ROS 版本（NumPy 2 不兼容），可改用更强制的路径优先级：

```bash
export CONDA_SITE_PACKAGES="/home/franka/miniconda3/envs/lerobot_fr3_qz/lib/python3.10/site-packages"
export PYTHONPATH="$CONDA_SITE_PACKAGES:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages"
```

```bash
lerobot-teleoperate \
  --teleop.type=mocap_leader \
  --robot.type=fr3_linker_l6_follower \
  --fps=60
```

（可选）仅 FR3 leader-follower 遥操作：

```bash
lerobot-teleoperate --teleop.type=fr3_leader --robot.type=fr3_follower
```

### 2.3 数据采集

完成基础启动与遥操作链路后，执行：

```bash
bash scripts/record.sh
```

`record.sh` 中已包含默认的相机、数据集路径、回合数等参数，可按注释修改 `DATASET_REPO_ID`、`DATASET_LOCAL_ROOT`、`num_episodes` 等字段。

在采集数据的时候，使用的是 rerun[^rerun]。基本使用是：如果该条任务走完，可以直接按“➡️”进入 reset 阶段（双人采集可在这时复原场景）；单人操作建议再按一次“➡️”将该条数据存储下来，利用存储时间简单布置场景。若单条数据录制不满意，可以按“⬅️”回到上一个阶段，需要注意当前是 reset 阶段还是录制阶段，终端会有提示。

其中一般需要修改的有
```bash
DATASET_REPO_ID="${DATASET_REPO_ID:-franka_hand/redcube}" #仓库id
--dataset.num_episodes=10 #需要录制多少条（我一般是10条一次，后面merge起来，录完一次后可以校准一下动捕）
--dataset.single_task="pick the red cube and drop it in box"  #任务描述，在VLA任务中就是文本输入
```