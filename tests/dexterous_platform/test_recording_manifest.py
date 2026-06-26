from __future__ import annotations

import unittest
from pathlib import Path

from dexterous_platform.recording import RecordingManifest, build_rosbag2_command


class RecordingManifestTest(unittest.TestCase):
    def test_build_rosbag2_command_uses_mcap_and_topics(self):
        manifest = RecordingManifest(
            recording_id="trial_001",
            task="pick cube",
            output_dir=Path("/tmp/records"),
            topics=("/franka/ee_pose", "/camera/handeye/color/image_rect_raw"),
        )

        command = build_rosbag2_command(manifest)

        self.assertEqual(command[:4], ["ros2", "bag", "record", "--storage"])
        self.assertIn("mcap", command)
        self.assertIn("/tmp/records/trial_001", command)
        self.assertEqual(command[-2:], ["/franka/ee_pose", "/camera/handeye/color/image_rect_raw"])


if __name__ == "__main__":
    unittest.main()
