"""Use a policy runtime as an action source."""

from __future__ import annotations

from fr3_dexterous_platform.inference.runtime import PolicyRuntime
from fr3_dexterous_platform.interfaces import ActionSource
from fr3_dexterous_platform.schemas import Action, Observation


class PolicyActionSource(ActionSource):
    def __init__(self, policy: PolicyRuntime):
        self.policy = policy

    def connect(self) -> None:
        pass

    def disconnect(self) -> None:
        pass

    def reset(self) -> None:
        pass

    def get_action(self, observation: Observation) -> Action:
        return self.policy.predict(observation)
