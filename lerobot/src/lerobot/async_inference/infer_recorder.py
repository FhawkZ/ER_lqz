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

"""Save observation (including images) and action chunks during async inference."""

from __future__ import annotations

import json
import logging
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
from PIL import Image

from lerobot.utils.constants import OBS_IMAGES

from .helpers import TimedAction, TimedObservation, is_image_key, make_lerobot_observation

logger = logging.getLogger(__name__)


def _tensor_to_numpy(value: Any) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().numpy()
    if isinstance(value, np.ndarray):
        return value
    return np.asarray(value)


def _save_image_array(path: Path, array: np.ndarray) -> None:
    arr = np.asarray(array)
    if arr.ndim == 3 and arr.shape[0] in (1, 3, 4) and arr.shape[0] < arr.shape[-1]:
        # (C, H, W) -> (H, W, C)
        arr = np.transpose(arr, (1, 2, 0))
    if arr.dtype != np.uint8:
        if arr.max() <= 1.0:
            arr = (arr * 255.0).clip(0, 255).astype(np.uint8)
        else:
            arr = arr.astype(np.uint8)
    if arr.shape[-1] == 1:
        arr = arr[..., 0]
    Image.fromarray(arr).save(path)


class InferRecorder:
    """Write per-step observation and action artifacts under ``record_dir``."""

    def __init__(self, record_dir: str | Path) -> None:
        run_id = time.strftime("%Y%m%d_%H%M%S")
        self.root = Path(record_dir) / f"run_{run_id}"
        self.steps_dir = self.root / "steps"
        self.steps_dir.mkdir(parents=True, exist_ok=True)
        logger.info("InferRecorder writing to %s", self.root)

    @property
    def run_path(self) -> Path:
        return self.root

    def save_step(
        self,
        timed_obs: TimedObservation,
        raw_observation: dict[str, Any],
        lerobot_features: dict[str, dict],
        policy_observation: dict[str, Any],
        action_chunk: list[TimedAction],
    ) -> None:
        timestep = timed_obs.get_timestep()
        step_dir = self.steps_dir / f"{timestep:06d}"
        step_dir.mkdir(parents=True, exist_ok=True)

        meta = {
            "timestep": timestep,
            "timestamp": timed_obs.get_timestamp(),
            "must_go": timed_obs.must_go,
        }
        (step_dir / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")

        self._save_observation_bundle(
            step_dir / "observation_raw",
            raw_observation,
            lerobot_features,
            use_lerobot_keys=False,
        )
        self._save_observation_bundle(
            step_dir / "observation_policy",
            policy_observation,
            lerobot_features=None,
            use_lerobot_keys=True,
        )
        self._save_actions(step_dir / "actions.json", action_chunk)

    def _save_observation_bundle(
        self,
        out_dir: Path,
        observation: dict[str, Any],
        lerobot_features: dict[str, dict] | None,
        *,
        use_lerobot_keys: bool,
    ) -> None:
        out_dir.mkdir(parents=True, exist_ok=True)
        scalars: dict[str, Any] = {}

        if use_lerobot_keys:
            obs_items = observation.items()
        else:
            lerobot_obs = make_lerobot_observation(observation, lerobot_features)
            obs_items = lerobot_obs.items()

        for key, value in obs_items:
            if isinstance(value, str):
                scalars[key] = value
                continue

            if is_image_key(key):
                try:
                    arr = _tensor_to_numpy(value)
                    if arr.ndim == 4:
                        arr = arr[0]
                    safe_name = key.replace("/", "_").replace(".", "_")
                    _save_image_array(out_dir / f"{safe_name}.png", arr)
                except Exception as exc:  # noqa: BLE001
                    logger.warning("Failed to save image %s: %s", key, exc)
                continue

            try:
                arr = _tensor_to_numpy(value)
                if arr.ndim == 0:
                    scalars[key] = arr.item()
                else:
                    scalars[key] = arr.tolist()
            except Exception:  # noqa: BLE001
                scalars[key] = str(value)

        (out_dir / "scalars.json").write_text(json.dumps(scalars, indent=2), encoding="utf-8")

    @staticmethod
    def _save_actions(path: Path, action_chunk: list[TimedAction]) -> None:
        records = []
        for timed_action in action_chunk:
            action = _tensor_to_numpy(timed_action.get_action())
            if action.ndim > 1:
                action = action.reshape(-1)
            records.append(
                {
                    "timestep": timed_action.get_timestep(),
                    "timestamp": timed_action.get_timestamp(),
                    "action": action.tolist(),
                }
            )
        path.write_text(json.dumps(records, indent=2), encoding="utf-8")
