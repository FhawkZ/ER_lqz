from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from fr3_dexterous_platform.adapters.mock import MockFr3LinkerRobot, ScriptedTeleopSource
from fr3_dexterous_platform.datasets.writers import JsonlWriter
from fr3_dexterous_platform.recording.control_loop import collect_episode
from fr3_dexterous_platform.schemas import CollectionConfig


class WriterTest(unittest.TestCase):
    def test_jsonl_contains_source_timestamps(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            collect_episode(
                MockFr3LinkerRobot(),
                ScriptedTeleopSource(),
                JsonlWriter(root),
                CollectionConfig(task="pick", fps=1000, frames=1),
            )
            line = (root / "jsonl" / "episode_000000.jsonl").read_text(encoding="utf-8").strip()
            payload = json.loads(line)

        self.assertIn("source_time", payload["observation"]["arm.ee_pose"])
        self.assertIn("raw_action", payload)
        self.assertIn("sent_action", payload)


if __name__ == "__main__":
    unittest.main()
