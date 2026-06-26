from __future__ import annotations

import unittest

from dexterous_platform.audit import audit_samples
from dexterous_platform.schema import SyncWindow, TopicSample, TopicSpec


def _sample(topic: str, t: float, seq: int | None = None, latency: float = 0.005) -> TopicSample:
    return TopicSample(topic=topic, source_time=t, publish_time=t + 0.001, receive_time=t + latency, sequence_id=seq)


class TimestampAuditTest(unittest.TestCase):
    def test_audit_passes_clean_streams(self):
        samples = [
            *[_sample("/arm", i * 0.01, i) for i in range(10)],
            *[_sample("/policy", i * 0.02, i) for i in range(5)],
        ]
        report = audit_samples(
            samples,
            specs=(
                TopicSpec("/arm", nominal_hz=100, max_p99_period_s=0.02, max_p99_receive_latency_s=0.02),
                TopicSpec("/policy", nominal_hz=50, max_p99_period_s=0.03, max_p99_receive_latency_s=0.02),
            ),
            sync_windows=(SyncWindow("policy", {"/arm": 0.015}),),
            target_topic="/policy",
        )

        self.assertTrue(report.passed)
        self.assertEqual(report.topics["/arm"].count, 10)
        self.assertEqual(report.topics["/arm"].dropped_sequence_count, 0)

    def test_audit_flags_missing_topic_and_dropped_sequence(self):
        samples = [_sample("/camera", 0.0, 0), _sample("/camera", 0.033, 2)]

        report = audit_samples(
            samples,
            specs=(
                TopicSpec("/camera", nominal_hz=30, max_dropped_sequences=0),
                TopicSpec("/hand", nominal_hz=50, required=True),
            ),
        )

        self.assertFalse(report.passed)
        messages = [issue.message for issue in report.issues]
        self.assertTrue(any("dropped sequence count" in message for message in messages))
        self.assertTrue(any("required topic is missing" in message for message in messages))


if __name__ == "__main__":
    unittest.main()
