from __future__ import annotations

import unittest

from fr3_dexterous_platform.adapters.mock import MockFr3LinkerRobot
from fr3_dexterous_platform.isaac.protocol import parse_isaac_packet
from fr3_dexterous_platform.isaac.udp_source import IsaacUdpActionSource


class IsaacInterfaceTest(unittest.TestCase):
    def test_parse_full_action_packet(self):
        packet = parse_isaac_packet(
            {
                "type": "action",
                "source_time": 1.0,
                "sequence_id": 3,
                "action": {"ee_x.pos": 0.3, "ori_qw.pos": 1.0},
            }
        )

        self.assertEqual(packet.packet_type, "action")
        self.assertEqual(packet.action["ee_x.pos"], 0.3)

    def test_isaac_mocap_packet_to_action_without_socket(self):
        robot = MockFr3LinkerRobot()
        obs = robot.get_observation()
        source = IsaacUdpActionSource()
        source._latest_packet = parse_isaac_packet(
            {
                "type": "mocap",
                "source_time": 1.0,
                "sequence_id": 4,
                "right_hand_pose": {
                    "position": [0.0, 0.0, 0.0],
                    "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                "hand_joints": [10, 20, 30, 40, 50, 60],
            }
        )

        action = source.get_action(obs)

        self.assertIn("ee_x.pos", action.values)
        self.assertEqual(action.values["hand_5.pos"], 60.0)
        self.assertEqual(action.trace["source"], "isaac_udp_mocap")


if __name__ == "__main__":
    unittest.main()
