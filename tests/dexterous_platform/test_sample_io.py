from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from dexterous_platform.io import read_jsonl_samples, write_jsonl_samples
from dexterous_platform.schema import TopicSample


class SampleIOTest(unittest.TestCase):
    def test_jsonl_round_trip(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "samples.jsonl"
            samples = [
                TopicSample(
                    topic="/policy/action_filtered",
                    source_time=1.0,
                    publish_time=1.001,
                    receive_time=1.002,
                    sequence_id=7,
                    payload={"x": 1.5},
                )
            ]
            write_jsonl_samples(path, samples)
            loaded = read_jsonl_samples(path)

        self.assertEqual(loaded[0].topic, "/policy/action_filtered")
        self.assertEqual(loaded[0].sequence_id, 7)
        self.assertEqual(loaded[0].payload["x"], 1.5)


if __name__ == "__main__":
    unittest.main()
