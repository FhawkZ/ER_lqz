from __future__ import annotations

import unittest
from pathlib import Path

from dexterous_platform.recording import FR3RecordingSession


class RecordingSessionTest(unittest.TestCase):
    def test_required_session_topics_include_bridged_canonical_topics(self):
        session = FR3RecordingSession(
            recording_id="trial",
            task="pick cube",
            output_dir=Path("/tmp/records"),
            include_optional_topics=False,
        )
        command = session.rosbag2_command()

        self.assertIn("/franka/ee_pose", command)
        self.assertIn("/franka/command/ee_pose", command)
        self.assertIn("/linkerhand/right/joint_states", command)
        self.assertIn("/teleop/retargeted_action", command)
        self.assertIn("/episode/event", command)


if __name__ == "__main__":
    unittest.main()
