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

### 2.4 会话收尾与残留清理（交接给下一位用户前必读）

采集/遥操作结束后若直接离开，容易留下 **重复的 `franka.launch`、僵死的阻抗控制器、未恢复的 FCI error**，下一位用户更容易出现 `communication ... reflex error`。  
**标准顺序：停 LeRobot → 停 launch 终端（Ctrl+C）→ 清残留 → `error_recovery` →（可选）回 home。**

#### A) ROS 域与公共环境（每台机器/每个终端先设）

实验室建议全员统一 `ROS_DOMAIN_ID`（例如 `42`），避免看到别人机器上的同名 topic：

```bash
# 写入 ~/.bashrc 或每个终端先执行
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0   # 仅本机通信时可设为 1

source /opt/ros/humble/setup.bash
source /home/franka/lqz/franka_ros2_15/install/setup.bash
# 若在 franka_ros2_15 工作区内：
# source install/setup.bash
```

查看当前域与节点：

```bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
ros2 node list
ros2 topic list | head -30
```

#### B) 停止 LeRobot / 推理相关进程

```bash
# 查看残留（record / teleop / client / policy_server / rclpy）
pgrep -af 'lerobot-record|lerobot-teleoperate|robot_client|policy_server|fr3_eef|mocap' || true

# 按需结束（先尝试正常 Ctrl+C，不行再 kill）
pkill -f 'lerobot-record' || true
pkill -f 'lerobot-teleoperate' || true
pkill -f 'lerobot.async_inference' || true
pkill -f 'fr3_eef_' || true
```

#### C) 停止机械臂 / 灵巧手 ROS launch

在运行 `ros2 launch ...` 的终端里 **Ctrl+C** 停掉，不要另开终端再 launch 叠一层。

若终端已关、进程仍在：

```bash
pgrep -af 'cartesian_impedance|move_to_start|franka.launch|controller_manager|ros2_control|linker_hand' || true

pkill -f 'cartesian_impedance_controller.launch.py' || true
pkill -f 'move_to_start_example_controller' || true
pkill -f 'linker_hand.launch.py' || true
# 仍有个别 ros2 节点时（慎用，会关掉本域所有 ros2 相关进程）：
# pkill -f 'ros2 launch' || true
```

#### D) FCI 错误恢复 + 回 home（推荐用脚本，不要叠在阻抗 launch 上）

**必须先停掉 C 中的阻抗/示例控制器**，再执行：

```bash
# 含 error_recovery + move_to_start（勿在 cartesian_impedance 仍运行时执行）
bash scripts/franka_move_to_start.sh
```

仅清除 Desk/reflex 错误、不移动：

```bash
/home/franka/lqz/franka_ros2_15/install/libfranka/bin/error_recovery 172.16.0.1
# 或项目内路径：
# ${ER_LQZ_ROOT}/franka_ros2_15/install/libfranka/bin/error_recovery 172.16.0.1
```

#### E) 清除 ROS 缓存 / daemon / DDS 共享内存

```bash
# 1) 重启 ros2 daemon（清 CLI 缓存、僵尸发现）
ros2 daemon stop
ros2 daemon start

# 2) 日志（可定期删，不影响配置）
rm -rf ~/.ros/log/*

# 3) Fast DDS 共享内存残留（无其他人在用本机 ROS 时执行）
rm -rf /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null || true

# 4) 若改过 RMW 实现，确认与团队一致（默认 humble 多为 Fast DDS）
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
```

#### F) 灵巧手 CAN 重置（仅手异常时）

```bash
sudo ip link set can0 down || true
sudo ip link set can0 up type can bitrate 1000000
```

#### G) 交接前检查清单

```bash
# 1. 本域不应再有 franka 控制节点
ros2 node list | grep -E 'NS_1|controller|franka' || echo "OK: no franka nodes"

# 2. 机械臂状态（无 error、模式正常）
bash scripts/diagnose_franka_fci.sh 172.16.0.1 enp12s0

# 3. 无重复占用 FCI 的进程
pgrep -af 'franka_hardware|ros2_control_node' || echo "OK: no hardware node"
```

**不要做的：**

- `cartesian_impedance_controller` 仍在跑时，再 `ros2 launch ... move_to_start` 或再起一套 `franka.launch`
- MoveIt launch 与笛卡尔阻抗 launch **同时**对 `NS_1` / `172.16.0.1`
- 采完数据直接走人、不做 `error_recovery` 和进程清理

[^developer]: 开发者向说明见各子目录 README。
[^pythonpath]: `scripts/env_lerobot.sh` 将 conda `site-packages` 置于 ROS python3.10 路径之前。
[^rerun]: https://rerun.io/