from __future__ import annotations

import unittest

from dexterous_platform.deployment import (
    RemoteInferenceContract,
    RemotePolicyEndpoint,
    build_lerobot_client_command,
    build_lerobot_server_command,
)


class RemoteInferenceTest(unittest.TestCase):
    def test_remote_endpoint_and_commands(self):
        endpoint = RemotePolicyEndpoint(host="0.0.0.0", port=8080)

        server = build_lerobot_server_command(endpoint, fps=30)
        client = build_lerobot_client_command(
            RemotePolicyEndpoint(host="192.168.1.10", port=8080),
            robot_type="fr3_eef",
            policy_path="/models/policy",
            task="pick cube",
        )

        self.assertEqual(endpoint.address, "0.0.0.0:8080")
        self.assertIn("lerobot.async_inference.policy_server", server)
        self.assertIn("192.168.1.10:8080", client)
        self.assertIn("fr3_eef", client)

    def test_contract_trace_topics(self):
        contract = RemoteInferenceContract(
            endpoint=RemotePolicyEndpoint("gpu-box"),
            observation_keys=("observation.images.handeye", "observation.state.arm_ee"),
            action_keys=("ee_x.pos", "hand_0.pos"),
        )

        self.assertIn("/policy/client_observation", contract.to_trace_topics())
        self.assertIn("/policy/server_response", contract.to_trace_topics())


if __name__ == "__main__":
    unittest.main()
