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

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("mocap_eef_leader")
@dataclass
class MocapEefLeaderConfig(TeleoperatorConfig):
    """Configuration for `MocapEefLeader`.

    Pairs with the `fr3_eef` follower: produces an absolute end-effector pose
    command (in the FR3 base frame) plus 6 hand joint commands, **without**
    running any inverse kinematics.

    Coordinate / rotation conventions (must match `fr3_eef`):
      * Pose order: ``[ee_x, ee_y, ee_z, ori_r, ori_p, ori_y]``
      * ``ori_*`` are extrinsic xyz Euler angles (radians), the same convention
        used by `FR3EEF.send_action` and `FR3EEF._arm_pose_cb`.
      * Position units: meters, in the FR3 base link frame
        (``arm_pose_frame_id`` of the follower, default ``fr3_link0``).
    """

    # ------------------------------------------------------------------
    # ROS2 topics
    # ------------------------------------------------------------------
    # Current end-effector pose published by the SERL Cartesian impedance
    # controller (same topic as `FR3EEFConfig.arm_pose_state_topic`). Used to
    # seed the leader's internal command pose so the very first frame matches
    # the follower's measured pose.
    arm_pose_state_topic: str = (
        "/NS_1/cartesian_impedance_controller/current_pose"
    )

    # ------------------------------------------------------------------
    # Output schema (must mirror `FR3EEFConfig`)
    # ------------------------------------------------------------------
    arm_pose_names: list[str] = field(
        default_factory=lambda: [
            "ee_x",
            "ee_y",
            "ee_z",
            "ori_r",
            "ori_p",
            "ori_y",
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

    # ------------------------------------------------------------------
    # Mocap → FR3 alignment
    # ------------------------------------------------------------------
    # Apply the standard mocap → FR3 axis remapping to BOTH the translation
    # delta and the rotation-vector delta extracted from the mocap RightHand:
    #     X_robot =  Y_mocap
    #     Y_robot = -X_mocap
    #     Z_robot =  Z_mocap
    # This matches the option of the same name in `MocapLeaderConfig`. We
    # default to True here because `mocap_eef_leader` is meant to drive the
    # FR3 directly in the robot base frame.
    enable_mocap_to_fr3_axis_mapping: bool = False

    # Per-cycle gains applied to the mocap-derived deltas before accumulation,
    # to make wrist motion reach further on the robot.
    delta_pos_gain: float = 1.4
    delta_rot_gain: float = 1.0

    # Low-pass filter coefficient on the delta stream (alpha closer to 1 =
    # more responsive but jitterier).
    delta_lpf_alpha: float = 0.7

    # Per-cycle clamp on the (filtered) deltas, in meters / radians. Prevents
    # mocap glitches from causing large jumps in the published EE pose.
    max_delta_pos_per_cycle: float = 0.04
    max_delta_rot_per_cycle: float = 0.30

    # ------------------------------------------------------------------
    # Misc
    # ------------------------------------------------------------------
    # Timeout when waiting for mocap or current_pose to arrive.
    timeout_s: float = 5.0

    # Mocap background poll frequency (Hz), same as MocapLeader.
    mocap_poll_hz: float = 120.0
