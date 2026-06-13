# FR3 Dexterous Platform

独立的 FR3 + LinkerHand + RealSense 灵巧操作平台项目。采集、遥操作、
推理部署、ROS2 接入全部独立实现，不依赖 LeRobot。唯一保留的 LeRobot 相关内容是
`--formats lerobot`，用于导出 LeRobot 对接 staging 数据格式。

## 功能

- 遥操作/采集控制循环：读取 observation、计算 action、发送 action、记录完整 step。
- 时间戳严谨性：每个 observation/action 都保存 `source_time`、`receive_time`、
  `sequence_id`，采集时可启用 freshness gate。
- 多格式数据输出：
  - `jsonl`: 原始 step 记录
  - `lerobot`: LeRobot 对接 staging 格式
  - `hdf5`: robomimic/HDF5 对接格式，需要 `h5py`
  - `droid`: DROID-style JSONL staging
  - `openpi`: OpenPI-style JSONL staging
- 推理部署：
  - 本地 policy runtime
  - 标准库 HTTP policy server/client
  - OpenPI server 命令规划入口
- ROS2 接入：
  - episode marker
  - Franka/SERL 与 LinkerHand topic bridge
  - launch 文件
  - native ROS2 FR3 backend
  - native mocap EE retarget action source
- Isaac 接口：
  - UDP/JSON action source
  - 支持 Isaac 发送完整 action
  - 支持 Isaac 发送 mocap wrist pose + hand joints，由本项目转换成 FR3 action

## 非 ROS 测试

```bash
PYTHONPATH=fr3_dexterous_platform/src \
python3 -m unittest discover -s fr3_dexterous_platform/tests
```

## Mock 采集

```bash
PYTHONPATH=fr3_dexterous_platform/src \
python3 -m fr3_dexterous_platform.cli collect \
  --backend mock \
  --formats jsonl,lerobot,droid,openpi \
  --output-dir /tmp/fr3dex_demo \
  --frames 10 \
  --fps 30 \
  --task "pick the red cube"
```

## 真机采集入口

这个路径直接使用 ROS2 topic：

```bash
source /opt/ros/humble/setup.bash

PYTHONPATH=fr3_dexterous_platform/src \
python3 -m fr3_dexterous_platform.cli collect \
  --backend native-ros \
  --action-source native-mocap \
  --formats jsonl,lerobot,hdf5,droid,openpi \
  --output-dir /data/fr3dex/redcube_001 \
  --frames 900 \
  --fps 30 \
  --task "pick the red cube and drop it in box" \
  --drop-stale-frames
```

Native ROS backend 使用这些当前工程 topic：

```text
subscribe:
  /NS_1/cartesian_impedance_controller/current_pose
  /cb_right_hand_state
  /camera/handeye/color/image_rect_raw
  /camera/fixed/color/image_rect_raw

publish:
  /NS_1/cartesian_impedance_controller/equilibrium_pose
  /cb_right_hand_control_cmd
```

Native mocap action source 使用：

```text
subscribe:
  /mocap/right_hand/pose          geometry_msgs/PoseStamped
  /mocap/right_hand/hand_cmd      sensor_msgs/JointState, optional
```

## Isaac 遥操作接口

Isaac 可以通过 UDP/JSON 给机器人侧发送数据。启动采集端：

```bash
PYTHONPATH=fr3_dexterous_platform/src \
python3 -m fr3_dexterous_platform.cli collect \
  --backend native-ros \
  --action-source isaac-udp \
  --isaac-host 0.0.0.0 \
  --isaac-port 15050 \
  --formats jsonl,lerobot,hdf5,droid,openpi \
  --output-dir /data/fr3dex/redcube_001 \
  --frames 900 \
  --fps 30 \
  --task "pick the red cube and drop it in box" \
  --drop-stale-frames
```

Isaac 可以发完整 action：

```json
{
  "type": "action",
  "source_time": 1.23,
  "sequence_id": 7,
  "action": {
    "ee_x.pos": 0.3,
    "ee_y.pos": 0.0,
    "ee_z.pos": 0.4,
    "ori_qx.pos": 0.0,
    "ori_qy.pos": 0.0,
    "ori_qz.pos": 0.0,
    "ori_qw.pos": 1.0,
    "hand_0.pos": 255.0
  }
}
```

也可以只发 mocap wrist pose 和 hand joints：

```json
{
  "type": "mocap",
  "source_time": 1.23,
  "sequence_id": 7,
  "right_hand_pose": {
    "position": [0.0, 0.0, 0.0],
    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0]
  },
  "hand_joints": [255, 255, 255, 255, 255, 255]
}
```

## 远程推理

启动内置 JSON policy server：

```bash
PYTHONPATH=fr3_dexterous_platform/src \
python3 -m fr3_dexterous_platform.cli serve-policy --host 0.0.0.0 --port 8088 --policy echo
```

机器人侧 client 可用 `RemotePolicyClient`。OpenPI server 命令可用：

```bash
python3 -m fr3_dexterous_platform.cli plan-openpi-server \
  --host 0.0.0.0 \
  --port 8090 \
  --model-path /models/openpi
```

## ROS2 Helper Package

见 [ros2/fr3_dexterous_platform_ros](ros2/fr3_dexterous_platform_ros/README.md)。
