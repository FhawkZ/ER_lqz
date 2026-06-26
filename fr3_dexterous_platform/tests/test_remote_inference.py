from __future__ import annotations

import unittest

from fr3_dexterous_platform.adapters.mock import MockFr3LinkerRobot
from fr3_dexterous_platform.inference.commands import (
    openpi_server_command,
)
from fr3_dexterous_platform.inference.remote import (
    PolicyHttpServer,
    RemotePolicyClient,
    observation_from_payload,
    observation_to_payload,
)
from fr3_dexterous_platform.inference.runtime import EchoPolicy


class RemoteInferenceTest(unittest.TestCase):
    def test_http_policy_roundtrip(self):
        server = PolicyHttpServer("127.0.0.1", 0, EchoPolicy())
        try:
            server.start()
        except PermissionError as exc:
            self.skipTest(f"local socket bind is not allowed in this sandbox: {exc}")
        assert server.httpd is not None
        port = server.httpd.server_address[1]
        try:
            obs = MockFr3LinkerRobot().get_observation()
            action = RemotePolicyClient(f"http://127.0.0.1:{port}").predict(obs)
        finally:
            server.stop()

        self.assertIn("ee_x.pos", action.values)
        self.assertIn("roundtrip_ms", action.trace)

    def test_observation_payload_roundtrip_without_socket(self):
        obs = MockFr3LinkerRobot().get_observation()
        restored = observation_from_payload(observation_to_payload(obs))

        self.assertEqual(set(obs.values), set(restored.values))
        self.assertEqual(restored.values["image.handeye"].topic, "/camera/handeye/color/image_rect_raw")

    def test_external_command_plans(self):
        self.assertIn("openpi.serving.server", openpi_server_command("0.0.0.0", 8090, "/m"))


if __name__ == "__main__":
    unittest.main()
