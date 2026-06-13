from __future__ import annotations

import unittest

from fr3_dexterous_platform.schemas import Observation, StampedValue
from fr3_dexterous_platform.teleop.native_mocap import NativeMocapRetargetSource


class P:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class Q:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.position = P(x, y, z)
        self.orientation = Q()


class PoseMsg:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.pose = Pose(x, y, z)


class JointState:
    position = [1, 2, 3, 4, 5, 6]


class NativeMocapTest(unittest.TestCase):
    def test_mocap_delta_outputs_fr3_schema(self):
        source = NativeMocapRetargetSource()
        source._latest_mocap_pose = PoseMsg(0.0, 0.0, 0.0)
        source._latest_hand_cmd = JointState()
        obs = Observation(
            values={
                "arm.ee_pose": StampedValue(
                    {"x": 0.3, "y": 0.0, "z": 0.4, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
                    source_time=0.0,
                    receive_time=0.0,
                )
            },
            receive_time=0.0,
        )

        first = source.get_action(obs)
        source._latest_mocap_pose = PoseMsg(0.01, 0.0, 0.0)
        second = source.get_action(obs)

        self.assertIn("ee_x.pos", second.values)
        self.assertIn("ori_qw.pos", second.values)
        self.assertEqual(second.values["hand_5.pos"], 6.0)
        self.assertGreater(second.values["ee_x.pos"], first.values["ee_x.pos"])


if __name__ == "__main__":
    unittest.main()
