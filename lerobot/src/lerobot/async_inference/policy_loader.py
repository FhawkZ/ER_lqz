# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Load pretrained policies for async inference (including PEFT / LoRA checkpoints)."""

from __future__ import annotations

import logging
from pathlib import Path

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.pretrained import PreTrainedPolicy


def load_policy_from_checkpoint(
    policy_class: type[PreTrainedPolicy],
    pretrained_path: str | Path,
    logger: logging.Logger | None = None,
) -> PreTrainedPolicy:
    """Load a full checkpoint or a PEFT (LoRA) adapter on top of its base model.

    LoRA checkpoints only contain ``adapter_model.safetensors``, not ``model.safetensors``.
    This helper mirrors ``make_policy`` PEFT loading used during training/eval.
    """
    log = logger or logging.getLogger(__name__)
    path = Path(pretrained_path)
    policy_config = PreTrainedConfig.from_pretrained(pretrained_path)
    adapter_config_path = path / "adapter_config.json"
    adapter_weights_path = path / "adapter_model.safetensors"

    if policy_config.use_peft and adapter_config_path.is_file():
        if not adapter_weights_path.is_file():
            raise FileNotFoundError(
                f"PEFT checkpoint missing adapter weights: {adapter_weights_path}"
            )
        from peft import PeftConfig, PeftModel

        peft_config = PeftConfig.from_pretrained(pretrained_path)
        base_path = peft_config.base_model_name_or_path or policy_config.pretrained_path
        if not base_path:
            raise ValueError(
                f"No base model path in adapter config for checkpoint {pretrained_path}"
            )
        log.info("Loading PI0 base model from %s", base_path)
        policy = policy_class.from_pretrained(base_path, config=policy_config)
        log.info("Loading PEFT adapter from %s", pretrained_path)
        policy = PeftModel.from_pretrained(policy, pretrained_path, config=peft_config)
        return policy

    return policy_class.from_pretrained(pretrained_path, config=policy_config)
