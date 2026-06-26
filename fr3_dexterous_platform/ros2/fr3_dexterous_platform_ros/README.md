# fr3_dexterous_platform_ros

ROS2 helper package included inside the independent FR3 Dexterous Platform
project.

It provides:

- `/episode/event` and `/episode/task` marker publisher.
- Topic bridge from the current SERL/LinkerHand topics to canonical
  platform topics.
- `/recording/manifest` publisher with the recommended MCAP topic list.

Build in a ROS2 workspace:

```bash
colcon build --packages-select fr3_dexterous_platform_ros
source install/setup.bash
```

Launch helpers:

```bash
ros2 launch fr3_dexterous_platform_ros fr3_collect.launch.py \
  recording_id:=trial_001 \
  task:="pick the red cube and drop it in box" \
  output_dir:=/data/fr3dex
```
