"""Isaac UDP/JSON protocol helpers.

Supported packets:

1. Full action packet:
{
  "type": "action",
  "source_time": 1.23,
  "sequence_id": 7,
  "action": {"ee_x.pos": 0.3, ...}
}

2. Mocap packet:
{
  "type": "mocap",
  "source_time": 1.23,
  "sequence_id": 7,
  "right_hand_pose": {
    "position": [x, y, z],
    "quaternion_xyzw": [qx, qy, qz, qw]
  },
  "hand_joints": [255, 255, 255, 255, 255, 255]
}
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from fr3_dexterous_platform.teleop.mocap_delta import MocapPose


@dataclass(frozen=True)
class IsaacPacket:
    packet_type: str
    source_time: float
    sequence_id: int | None
    action: Mapping[str, float] | None = None
    mocap_pose: MocapPose | None = None
    hand_joints: list[float] | None = None


def parse_isaac_packet(payload: Mapping[str, Any]) -> IsaacPacket:
    packet_type = str(payload.get("type", "mocap"))
    source_time = float(payload["source_time"])
    sequence_id = payload.get("sequence_id")
    if packet_type == "action":
        action = {str(k): float(v) for k, v in payload["action"].items()}
        return IsaacPacket(packet_type, source_time, sequence_id, action=action)
    if packet_type == "mocap":
        pose_raw = payload["right_hand_pose"]
        pos = tuple(float(v) for v in pose_raw["position"])
        quat = tuple(float(v) for v in pose_raw["quaternion_xyzw"])
        if len(pos) != 3 or len(quat) != 4:
            raise ValueError("Isaac mocap packet expects position[3] and quaternion_xyzw[4]")
        hand = [float(v) for v in payload.get("hand_joints", [255.0] * 6)]
        return IsaacPacket(
            packet_type,
            source_time,
            sequence_id,
            mocap_pose=MocapPose(pos, quat, source_time, sequence_id),
            hand_joints=hand,
        )
    raise ValueError(f"Unsupported Isaac packet type: {packet_type}")
