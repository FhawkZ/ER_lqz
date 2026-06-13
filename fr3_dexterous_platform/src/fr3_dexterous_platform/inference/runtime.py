"""Policy runtimes."""

from __future__ import annotations

from typing import Protocol

from fr3_dexterous_platform.schemas import Action, Observation
from fr3_dexterous_platform.utils.time import now_s


class PolicyRuntime(Protocol):
    def predict(self, observation: Observation) -> Action: ...


class EchoPolicy:
    """Small test policy that emits a valid FR3 EE + hand action."""

    def predict(self, observation: Observation) -> Action:
        t = now_s()
        arm = observation.values.get("arm.ee_pose")
        hand = observation.values.get("hand.joints")
        arm_value = arm.value if arm else {"x": 0.0, "y": 0.0, "z": 0.4}
        hand_value = hand.value if hand else {"hand_0": 0.0}
        values = {
            "ee_x.pos": float(arm_value.get("x", 0.0)),
            "ee_y.pos": float(arm_value.get("y", 0.0)),
            "ee_z.pos": float(arm_value.get("z", 0.4)),
            "ori_qx.pos": 0.0,
            "ori_qy.pos": 0.0,
            "ori_qz.pos": 0.0,
            "ori_qw.pos": 1.0,
            "hand_0.pos": float(hand_value.get("hand_0", 0.0)),
        }
        return Action(values=values, source_time=t, receive_time=t, trace={"policy": "echo"})
