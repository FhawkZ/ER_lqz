from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from fr3_dexterous_platform.adapters.mock import MockFr3LinkerRobot, ScriptedTeleopSource
from fr3_dexterous_platform.datasets.writers import MultiFormatWriter
from fr3_dexterous_platform.recording.control_loop import collect_episode
from fr3_dexterous_platform.schemas import CollectionConfig, FreshnessSpec


class ControlLoopTest(unittest.TestCase):
    def test_collect_episode_writes_multiple_formats(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            writer = MultiFormatWriter.from_formats(root, ["jsonl", "lerobot", "droid", "openpi"])
            result = collect_episode(
                MockFr3LinkerRobot(),
                ScriptedTeleopSource(),
                writer,
                CollectionConfig(
                    task="pick cube",
                    fps=1000,
                    frames=5,
                    freshness=FreshnessSpec(
                        required_keys=("arm.ee_pose", "hand.joints", "image.handeye", "image.fixed"),
                        max_age_s={"image.handeye": 0.1, "image.fixed": 0.1},
                    ),
                ),
            )

            self.assertEqual(result.written_frames, 5)
            self.assertTrue((root / "jsonl" / "episode_000000.jsonl").exists())
            self.assertTrue((root / "lerobot" / "dataset_info.json").exists())
            self.assertTrue((root / "droid" / "dataset_info.json").exists())
            self.assertTrue((root / "openpi" / "dataset_info.json").exists())

    def test_collect_episode_drops_stale_observation(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = MultiFormatWriter.from_formats(Path(tmp), ["jsonl"])
            result = collect_episode(
                MockFr3LinkerRobot(stale_camera=True),
                ScriptedTeleopSource(),
                writer,
                CollectionConfig(
                    task="pick cube",
                    fps=1000,
                    frames=3,
                    freshness=FreshnessSpec(
                        required_keys=("image.handeye", "image.fixed"),
                        max_age_s={"image.handeye": 0.1, "image.fixed": 0.1},
                    ),
                    drop_stale_frames=True,
                ),
            )

            self.assertEqual(result.written_frames, 0)
            self.assertEqual(result.dropped_frames, 3)
            self.assertTrue(result.issues)


if __name__ == "__main__":
    unittest.main()
