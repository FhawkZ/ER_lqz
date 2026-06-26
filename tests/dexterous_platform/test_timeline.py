from __future__ import annotations

import unittest

from dexterous_platform.converters import build_frames
from dexterous_platform.schema import ConversionPlan, ConversionTopic, LookupRule, TopicSample


def sample(topic: str, t: float, payload: dict, seq: int = 0) -> TopicSample:
    return TopicSample(topic=topic, source_time=t, publish_time=t, receive_time=t + 0.002, sequence_id=seq, payload=payload)


class TimelineTest(unittest.TestCase):
    def test_build_frames_aligns_previous_camera_and_interpolated_arm(self):
        samples = [
            sample("/policy/action_filtered", 0.10, {"ee_x": 1.0}, 1),
            sample("/policy/action_filtered", 0.20, {"ee_x": 2.0}, 2),
            sample("/arm/ee_pose", 0.00, {"x": 0.0}, 0),
            sample("/arm/ee_pose", 0.20, {"x": 2.0}, 1),
            sample("/camera/color", 0.05, {"frame": "a"}, 10),
            sample("/camera/color", 0.18, {"frame": "b"}, 11),
        ]
        plan = ConversionPlan(
            target_topic="/policy/action_filtered",
            task="pick cube",
            topics=(
                ConversionTopic("/arm/ee_pose", "observation.arm", LookupRule.INTERPOLATE, max_age_s=0.001),
                ConversionTopic("/camera/color", "observation.image", LookupRule.PREVIOUS, max_age_s=0.08),
                ConversionTopic("/policy/action_filtered", "action", LookupRule.NEAREST, max_age_s=0.001),
            ),
        )

        frames = build_frames(samples, plan)

        self.assertEqual(len(frames), 2)
        self.assertEqual(frames[0].values["observation.arm"]["x"], 1.0)
        self.assertEqual(frames[0].values["observation.image"]["frame"], "a")
        self.assertEqual(frames[1].values["observation.image"]["frame"], "b")
        self.assertEqual(frames[0].sources["observation.image"].age_s, 0.05)

    def test_build_frames_drops_stale_modalities(self):
        samples = [
            sample("/policy/action_filtered", 1.0, {"a": 1}),
            sample("/camera/color", 0.0, {"frame": "old"}),
        ]
        plan = ConversionPlan(
            target_topic="/policy/action_filtered",
            task="stale camera",
            topics=(ConversionTopic("/camera/color", "observation.image", LookupRule.PREVIOUS, max_age_s=0.1),),
        )

        self.assertEqual(build_frames(samples, plan), [])


if __name__ == "__main__":
    unittest.main()
