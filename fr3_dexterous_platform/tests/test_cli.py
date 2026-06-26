from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
SRC = PROJECT / "src"


class CliTest(unittest.TestCase):
    def test_mock_collect_cli(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "fr3_dexterous_platform.cli",
                    "collect",
                    "--backend",
                    "mock",
                    "--formats",
                    "jsonl,lerobot",
                    "--output-dir",
                    tmp,
                    "--frames",
                    "2",
                    "--fps",
                    "1000",
                    "--task",
                    "pick cube",
                ],
                cwd=str(PROJECT),
                env={"PYTHONPATH": str(SRC)},
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=True,
            )
            payload = json.loads(proc.stdout)
            self.assertEqual(payload["written_frames"], 2)
            self.assertTrue((Path(tmp) / "lerobot" / "episode_000000.jsonl").exists())


if __name__ == "__main__":
    unittest.main()
