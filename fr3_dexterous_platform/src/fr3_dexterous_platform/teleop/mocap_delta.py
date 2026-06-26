"""Framework-independent mocap delta retargeting."""

from __future__ import annotations

import math
from dataclasses import dataclass

from fr3_dexterous_platform.adapters.native_ros import ARM_POSE_KEYS, HAND_KEYS
from fr3_dexterous_platform.schemas import Observation
from fr3_dexterous_platform.utils.quaternion import (
    align_to_previous,
    conjugate,
    from_rotvec,
    multiply,
    to_rotvec,
)


@dataclass(frozen=True)
class MocapPose:
    position: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    source_time: float
    sequence_id: int | None = None


class DeltaMocapRetargeter:
    def __init__(
        self,
        delta_pos_gain: float = 1.4,
        delta_rot_gain: float = 1.0,
        max_delta_pos_per_cycle: float = 0.04,
        max_delta_rot_per_cycle: float = 0.30,
    ) -> None:
        self.delta_pos_gain = delta_pos_gain
        self.delta_rot_gain = delta_rot_gain
        self.max_delta_pos_per_cycle = max_delta_pos_per_cycle
        self.max_delta_rot_per_cycle = max_delta_rot_per_cycle
        self.previous_pose: MocapPose | None = None
        self.command_position: tuple[float, float, float] | None = None
        self.command_quaternion: tuple[float, float, float, float] | None = None
        self.previous_emitted_quaternion: tuple[float, float, float, float] | None = None

    def reset(self) -> None:
        self.previous_pose = None
        self.command_position = None
        self.command_quaternion = None
        self.previous_emitted_quaternion = None

    def action_values(
        self,
        observation: Observation,
        pose: MocapPose,
        hand_values: list[float] | None = None,
    ) -> dict[str, float]:
        if self.command_position is None or self.command_quaternion is None:
            arm = observation.values["arm.ee_pose"].value
            self.command_position = (float(arm["x"]), float(arm["y"]), float(arm["z"]))
            self.command_quaternion = (float(arm["qx"]), float(arm["qy"]), float(arm["qz"]), float(arm["qw"]))

        if self.previous_pose is None:
            delta_pos = (0.0, 0.0, 0.0)
            delta_quat = (0.0, 0.0, 0.0, 1.0)
        else:
            delta_pos = tuple(
                (pose.position[i] - self.previous_pose.position[i]) * self.delta_pos_gain
                for i in range(3)
            )
            delta_quat = multiply(pose.quaternion_xyzw, conjugate(self.previous_pose.quaternion_xyzw))
        self.previous_pose = pose

        rotvec = tuple(v * self.delta_rot_gain for v in to_rotvec(delta_quat))
        delta_pos = _clamp_vec(delta_pos, self.max_delta_pos_per_cycle)
        rotvec = _clamp_vec(rotvec, self.max_delta_rot_per_cycle)

        assert self.command_position is not None and self.command_quaternion is not None
        self.command_position = tuple(self.command_position[i] + delta_pos[i] for i in range(3))
        self.command_quaternion = align_to_previous(
            multiply(from_rotvec(rotvec), self.command_quaternion),
            self.previous_emitted_quaternion,
        )
        self.previous_emitted_quaternion = self.command_quaternion

        out: dict[str, float] = {}
        for key, value in zip(ARM_POSE_KEYS, (*self.command_position, *self.command_quaternion)):
            out[f"{key}.pos"] = float(value)
        hands = hand_values or [255.0] * 6
        for key, value in zip(HAND_KEYS, hands):
            out[f"{key}.pos"] = float(value)
        return out


def _clamp_vec(vec: tuple[float, float, float], max_norm: float) -> tuple[float, float, float]:
    norm = math.sqrt(sum(v * v for v in vec))
    if norm > max_norm and norm > 1e-12:
        scale = max_norm / norm
        return tuple(v * scale for v in vec)  # type: ignore[return-value]
    return vec
