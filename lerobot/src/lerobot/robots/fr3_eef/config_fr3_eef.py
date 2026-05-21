#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("fr3_eef")
@dataclass
class FR3EEFConfig(RobotConfig):
    """Configuration for FR3 end-effector (Cartesian) follower + Linker L6 hand (ROS2).

    The FR3 arm is driven through the SERL Cartesian impedance controller via
    `~/equilibrium_pose` (a `PoseStamped`), and the current EE pose is read back
    from `~/current_pose`. The Linker L6 hand is still controlled at joint level.
    """

    # ---- FR3 end-effector control (Cartesian impedance controller) ----
    # Target end-effector pose published to the impedance controller.
    arm_pose_command_topic: str = (
        "/NS_1/cartesian_impedance_controller/equilibrium_pose"
    )
    # Current end-effector pose published by the impedance controller.
    arm_pose_state_topic: str = (
        "/NS_1/cartesian_impedance_controller/current_pose"
    )
    # `frame_id` written into the PoseStamped header. Should match the robot
    # base link expected by the impedance controller (typically `<arm_id>_link0`).
    arm_pose_frame_id: str = "fr3_link0"

    # ---- Linker L6 hand ----
    hand_control_topic: str = "/cb_right_hand_control_cmd"
    hand_state_topic: str = "/cb_right_hand_state"

    # If False, only the hand command is published (useful for debugging the
    # hand alone without moving the arm).
    enable_arm_publish: bool = True

    # Match lerobot `dataset.fps` / teleop loop.
    control_hz: float = 25.0

    # EMA smoothing factors applied on top of the impedance controller's own
    # internal filtering, to suppress occasional spikes coming from the policy.
    # Lower = smoother but slower; higher = more responsive but jitterier.
    ema_alpha_pos: float = 0.2
    ema_alpha_rot: float = 0.2

    # Names exposed as observation/action features for the FR3 end-effector.
    # 7D: position (xyz, meters) + orientation quaternion (qx,qy,qz,qw),
    # scipy / geometry_msgs ordering. Components are sign-aligned frame-to-frame
    # for continuous learning signals (see ``FR3EEF._align_obs_quat``).
    arm_pose_names: list[str] = field(
        default_factory=lambda: [
            "ee_x",
            "ee_y",
            "ee_z",
            "ori_qx",
            "ori_qy",
            "ori_qz",
            "ori_qw",
        ]
    )

    hand_joint_names: list[str] = field(
        default_factory=lambda: [
            "hand_0",
            "hand_1",
            "hand_2",
            "hand_3",
            "hand_4",
            "hand_5",
        ]
    )

    timeout_s: float = 5.0
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
