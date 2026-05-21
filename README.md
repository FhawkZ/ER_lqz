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
  - 项目辅助脚本（LeRobot **0.5.1**）。
  - `env_lerobot.sh`：ROS2 + lerobot 公共环境（被其他脚本 `source`）。
  - `teleop.sh`：默认 `mocap_retarget_leader` + `fr3_eef`。
  - `teleop_ik.sh`：`mocap_leader` + `fr3_linker_l6_follower`（IK 臂采集/遥操作）。
  - `record.sh`：数据采集（含 `streaming_encoding`）。
  - `server.sh` / `client.sh`：异步推理。

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

在遥操作终端中先激活 conda 环境；`scripts/env_lerobot.sh` 会统一处理 `PYTHONPATH`（conda 优先、过滤 ROS 的 python3.10 路径）[^pythonpath]：

```bash
conda activate lerobot
bash scripts/install_lerobot_ros_msgs.sh   # 首次或 typesupport 报错时执行
```

安装/更新 lerobot 后，脚本会自动 `source` 公共环境（见 `scripts/env_lerobot.sh`）：

```bash
cd /home/franka/lqz/lerobot && pip install -e .
pip install dex-retargeting   # 仅 dex 遥操作需要
```

**默认遥操作**（dex 手部 + 笛卡尔 EE）：

```bash
bash scripts/teleop.sh
```

**IK 臂遥操作 / 与采集一致**（`mocap_leader` + `fr3_linker_l6_follower`，需 pinocchio）：

```bash
bash scripts/teleop_ik.sh
```

等价于手动指定类型：

```bash
TELEOP_TYPE=mocap_eef_leader ROBOT_TYPE=fr3_eef bash scripts/teleop.sh
```

> 内嵌 `lerobot` 为 **v0.5.1**；升级维护见 `scripts/upgrade_lerobot.sh`。若 pinocchio 仍加载到 ROS 版本，检查 `env_lerobot.sh` 中 `PYTHONPATH` 顺序。

（可选）仅 FR3 leader-follower 遥操作：

```bash
lerobot-teleoperate --teleop.type=fr3_leader --robot.type=fr3_follower
```

### 2.3 数据采集

与遥操作相同，默认 **`mocap_retarget_leader` + `fr3_eef`**（数据集为 6D EE + 6 路手部，不是 7 关节臂）。若需 IK 臂数据集，请改用 `teleop_ik.sh` 并自行调整 `record.sh` 中的类型。

```bash
bash scripts/record.sh
```

可按环境变量覆盖 `DATASET_REPO_ID`、`NUM_EPISODES`、`SINGLE_TASK` 等（见脚本头部注释）。

**本地策略推理**（与采集同 `fr3_eef` + 相机，无 gRPC；默认权重 `outputs/act_redcube_merged/checkpoints/060000`）：

```bash
bash scripts/infer_local.sh
```

可选：`EPISODE_TIME_S=60`、`POLICY_PATH=...`、`WIPE_DATASET=true`（见 `scripts/infer_local.sh` 头部）。

在采集数据的时候，使用的是 rerun[^rerun]。基本使用是：如果该条任务走完，可以直接按“➡️”进入 reset 阶段（双人采集可在这时复原场景）；单人操作建议再按一次“➡️”将该条数据存储下来，利用存储时间简单布置场景。若单条数据录制不满意，可以按“⬅️”回到上一个阶段，需要注意当前是 reset 阶段还是录制阶段，终端会有提示。

其中一般需要修改的有（也可通过环境变量传入，见 `scripts/record.sh` 头部注释）：

```bash
DATASET_REPO_ID=franka_hand/redcube NUM_EPISODES=10 \
  SINGLE_TASK="pick the red cube and drop it in box" \
  bash scripts/record.sh
```

清空重来时加 `WIPE_DATASET=true`（只删 `DATASET_LOCAL_ROOT`，不会误删整个 `Data/` 父目录）。