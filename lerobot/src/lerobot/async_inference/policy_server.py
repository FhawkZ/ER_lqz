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

"""
Example:
```shell
python -m lerobot.async_inference.policy_server \
     --host=127.0.0.1 \
     --port=8080 \
     --fps=30 \
     --inference_latency=0.033 \
     --obs_queue_timeout=1
```
"""

import json
import logging
import pickle  # nosec
import threading
import time
import traceback
from pathlib import Path
from concurrent import futures
from dataclasses import asdict
from pprint import pformat
from queue import Empty, Queue
from typing import Any

import draccus
import grpc
import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import get_policy_class, make_pre_post_processors
from lerobot.policies.utils import populate_queues
from lerobot.processor import PolicyProcessorPipeline
from lerobot.utils.constants import ACTION, OBS_IMAGE, OBS_IMAGES
from lerobot.transport import (
    services_pb2,  # type: ignore
    services_pb2_grpc,  # type: ignore
)
from lerobot.transport.utils import receive_bytes_in_chunks
from lerobot.types import PolicyAction

from .policy_loader import load_policy_from_checkpoint
from .configs import PolicyServerConfig
from .constants import POLICIES_REQUIRING_QUEUE_PREP, SUPPORTED_POLICIES
from .inference_trace import InferenceTraceRecorder
from .helpers import (
    FPSTracker,
    Observation,
    RemotePolicyConfig,
    TimedAction,
    TimedObservation,
    align_lerobot_features_to_policy_state,
    get_logger,
    load_observation_state_names_from_checkpoint,
    observations_similar,
    raw_observation_to_observation,
)


def _checkpoint_policy_type(pretrained_path: str) -> str | None:
    """Read ``type`` from ``config.json`` under a local checkpoint directory."""
    config_path = Path(pretrained_path) / "config.json"
    if not config_path.is_file():
        return None
    with config_path.open() as f:
        return json.load(f).get("type")


def _load_rename_map_from_checkpoint(pretrained_path: str) -> dict[str, str]:
    """Read observation rename_map saved in ``policy_preprocessor.json``."""
    preproc_path = Path(pretrained_path) / "policy_preprocessor.json"
    if not preproc_path.is_file():
        return {}
    with preproc_path.open() as f:
        data = json.load(f)
    for step in data.get("steps", []):
        if step.get("registry_name") == "rename_observations_processor":
            return dict(step.get("config", {}).get("rename_map", {}))
    return {}


class PolicyServer(services_pb2_grpc.AsyncInferenceServicer):
    prefix = "policy_server"
    logger = get_logger(prefix)

    def __init__(self, config: PolicyServerConfig):
        self.config = config
        self.shutdown_event = threading.Event()

        # FPS measurement
        self.fps_tracker = FPSTracker(target_fps=config.fps)

        self.observation_queue = Queue(maxsize=1)

        self._predicted_timesteps_lock = threading.Lock()
        self._predicted_timesteps = set()

        self.last_processed_obs = None

        # Attributes will be set by SendPolicyInstructions
        self.device = None
        self.policy_type = None
        self.lerobot_features = None
        self.actions_per_chunk = None
        self.policy = None
        self.preprocessor: PolicyProcessorPipeline[dict[str, Any], dict[str, Any]] | None = None
        self.postprocessor: PolicyProcessorPipeline[PolicyAction, PolicyAction] | None = None
        self.observation_rename_map: dict[str, str] = {}
        self.pretrained_path: str | None = None
        self.trace_recorder = InferenceTraceRecorder(config.trace_dir, enabled=config.record_inference)

    @property
    def running(self):
        return not self.shutdown_event.is_set()

    @property
    def policy_image_features(self):
        return self.policy.config.image_features

    def _reset_server(self) -> None:
        """Flushes server state when new client connects."""
        # only running inference on the latest observation received by the server
        self.shutdown_event.set()
        self.observation_queue = Queue(maxsize=1)

        with self._predicted_timesteps_lock:
            self._predicted_timesteps = set()

        if self.policy is not None and hasattr(self.policy, "reset"):
            self.policy.reset()

    def Ready(self, request, context):  # noqa: N802
        client_id = context.peer()
        self.logger.info(f"Client {client_id} connected and ready")
        self._reset_server()
        self.shutdown_event.clear()

        return services_pb2.Empty()

    def SendPolicyInstructions(self, request, context):  # noqa: N802
        """Receive policy instructions from the robot client"""

        if not self.running:
            self.logger.warning("Server is not running. Ignoring policy instructions.")
            return services_pb2.Empty()

        client_id = context.peer()

        policy_specs = pickle.loads(request.data)  # nosec

        if not isinstance(policy_specs, RemotePolicyConfig):
            raise TypeError(f"Policy specs must be a RemotePolicyConfig. Got {type(policy_specs)}")

        if policy_specs.policy_type not in SUPPORTED_POLICIES:
            raise ValueError(
                f"Policy type {policy_specs.policy_type} not supported. "
                f"Supported policies: {SUPPORTED_POLICIES}"
            )

        self.logger.info(
            f"Receiving policy instructions from {client_id} | "
            f"Policy type: {policy_specs.policy_type} | "
            f"Pretrained name or path: {policy_specs.pretrained_name_or_path} | "
            f"Actions per chunk: {policy_specs.actions_per_chunk} | "
            f"Device: {policy_specs.device}"
        )

        self.device = policy_specs.device
        self.policy_type = policy_specs.policy_type  # act, pi0, etc.
        self.lerobot_features = policy_specs.lerobot_features
        self.actions_per_chunk = policy_specs.actions_per_chunk
        self.pretrained_path = policy_specs.pretrained_name_or_path

        policy_state_names = load_observation_state_names_from_checkpoint(
            policy_specs.pretrained_name_or_path
        )
        if policy_state_names and self.lerobot_features is not None:
            self.lerobot_features = align_lerobot_features_to_policy_state(
                self.lerobot_features, policy_state_names
            )
            preview = ", ".join(policy_state_names[:4])
            if len(policy_state_names) > 4:
                preview += ", ..."
            self.logger.info(
                "Aligned observation.state to checkpoint (%dD): %s",
                len(policy_state_names),
                preview,
            )

        checkpoint_rename_map = _load_rename_map_from_checkpoint(policy_specs.pretrained_name_or_path)
        self.observation_rename_map = {**checkpoint_rename_map, **policy_specs.rename_map}
        if self.observation_rename_map:
            self.logger.info(f"Observation rename_map: {self.observation_rename_map}")

        checkpoint_type = _checkpoint_policy_type(policy_specs.pretrained_name_or_path)
        if checkpoint_type is not None and checkpoint_type != self.policy_type:
            raise ValueError(
                f"policy_type mismatch: client requested '{self.policy_type}' but checkpoint "
                f"'{policy_specs.pretrained_name_or_path}' has type '{checkpoint_type}'. "
                f"Set POLICY_TYPE={checkpoint_type} on the robot client."
            )

        policy_class = get_policy_class(self.policy_type)

        start = time.perf_counter()
        try:
            self.policy = load_policy_from_checkpoint(
                policy_class, policy_specs.pretrained_name_or_path, self.logger
            )
            self.policy.to(self.device)

            # Load preprocessor and postprocessor, overriding device to match requested device
            device_override = {"device": self.device}
            preprocessor_overrides: dict[str, dict] = {"device_processor": device_override}
            if policy_specs.rename_map:
                preprocessor_overrides["rename_observations_processor"] = {
                    "rename_map": policy_specs.rename_map
                }
            self.preprocessor, self.postprocessor = make_pre_post_processors(
                self.policy.config,
                pretrained_path=policy_specs.pretrained_name_or_path,
                preprocessor_overrides=preprocessor_overrides,
                postprocessor_overrides={"device_processor": device_override},
            )
        except Exception as exc:
            tb = traceback.format_exc()
            self.logger.error(f"Failed to load policy: {exc}\n{tb}")
            self.trace_recorder.record_error(
                stage="SendPolicyInstructions",
                error=str(exc),
                policy_type=self.policy_type,
                pretrained_path=policy_specs.pretrained_name_or_path,
                extra={"traceback": tb},
            )
            raise

        end = time.perf_counter()

        self.logger.info(f"Time taken to put policy on {self.device}: {end - start:.4f} seconds")

        return services_pb2.Empty()

    def SendObservations(self, request_iterator, context):  # noqa: N802
        """Receive observations from the robot client"""
        client_id = context.peer()
        self.logger.debug(f"Receiving observations from {client_id}")

        receive_time = time.time()  # comparing timestamps so need time.time()
        start_deserialize = time.perf_counter()
        received_bytes = receive_bytes_in_chunks(
            request_iterator, None, self.shutdown_event, self.logger
        )  # blocking call while looping over request_iterator
        timed_observation = pickle.loads(received_bytes)  # nosec
        deserialize_time = time.perf_counter() - start_deserialize

        self.logger.debug(f"Received observation #{timed_observation.get_timestep()}")

        obs_timestep = timed_observation.get_timestep()
        obs_timestamp = timed_observation.get_timestamp()

        # Calculate FPS metrics
        fps_metrics = self.fps_tracker.calculate_fps_metrics(obs_timestamp)

        self.logger.debug(
            f"Received observation #{obs_timestep} | "
            f"Avg FPS: {fps_metrics['avg_fps']:.2f} | "  # fps at which observations are received from client
            f"Target: {fps_metrics['target_fps']:.2f} | "
            f"One-way latency: {(receive_time - obs_timestamp) * 1000:.2f}ms"
        )

        self.logger.debug(
            f"Server timestamp: {receive_time:.6f} | "
            f"Client timestamp: {obs_timestamp:.6f} | "
            f"Deserialization time: {deserialize_time:.6f}s"
        )

        if not self._enqueue_observation(
            timed_observation  # wrapping a RawObservation
        ):
            self.logger.debug(f"Observation #{obs_timestep} has been filtered out")

        return services_pb2.Empty()

    def GetActions(self, request, context):  # noqa: N802
        """Returns actions to the robot client. Actions are sent as a single
        chunk, containing multiple actions."""
        client_id = context.peer()
        self.logger.debug(f"Client {client_id} connected for action streaming")

        # Generate action based on the most recent observation and its timestep
        try:
            getactions_starts = time.perf_counter()
            obs = self.observation_queue.get(timeout=self.config.obs_queue_timeout)
            self.logger.info(
                f"Running inference for observation #{obs.get_timestep()} (must_go: {obs.must_go})"
            )

            with self._predicted_timesteps_lock:
                if obs.get_timestep() in self._predicted_timesteps:
                    self.logger.debug(
                        f"Skipping observation #{obs.get_timestep()} - already predicted"
                    )
                    return services_pb2.Empty()

            start_time = time.perf_counter()
            action_chunk = self._predict_action_chunk(obs)
            inference_time = time.perf_counter() - start_time

            with self._predicted_timesteps_lock:
                self._predicted_timesteps.add(obs.get_timestep())

            start_time = time.perf_counter()
            actions_bytes = pickle.dumps(action_chunk)  # nosec
            serialize_time = time.perf_counter() - start_time

            # Create and return the action chunk
            actions = services_pb2.Actions(data=actions_bytes)

            self.logger.info(
                f"Action chunk #{obs.get_timestep()} generated | "
                f"Total time: {(inference_time + serialize_time) * 1000:.2f}ms"
            )

            self.logger.debug(
                f"Action chunk #{obs.get_timestep()} generated | "
                f"Inference time: {inference_time:.2f}s |"
                f"Serialize time: {serialize_time:.2f}s |"
                f"Total time: {inference_time + serialize_time:.2f}s"
            )

            time.sleep(
                max(0, self.config.inference_latency - max(0, time.perf_counter() - getactions_starts))
            )  # sleep controls inference latency

            return actions

        except Empty:  # no observation added to queue in obs_queue_timeout
            return services_pb2.Empty()

        except Exception as e:
            tb = traceback.format_exc()
            self.logger.error(f"Error in StreamActions: {e}\n{tb}")
            self.trace_recorder.record_error(
                stage="GetActions",
                error=str(e),
                policy_type=self.policy_type,
                pretrained_path=self.pretrained_path,
                extra={"traceback": tb},
            )

            return services_pb2.Empty()

    def _obs_sanity_checks(self, obs: TimedObservation, previous_obs: TimedObservation) -> bool:
        """Check if the observation is valid to be processed by the policy"""
        with self._predicted_timesteps_lock:
            predicted_timesteps = self._predicted_timesteps

        if obs.get_timestep() in predicted_timesteps:
            self.logger.debug(f"Skipping observation #{obs.get_timestep()} - Timestep predicted already!")
            return False

        elif observations_similar(obs, previous_obs, lerobot_features=self.lerobot_features):
            self.logger.debug(
                f"Skipping observation #{obs.get_timestep()} - Observation too similar to last obs predicted!"
            )
            return False

        else:
            return True

    def _enqueue_observation(self, obs: TimedObservation) -> bool:
        """Enqueue an observation if it must go through processing, otherwise skip it.
        Observations not in queue are never run through the policy network"""

        if (
            obs.must_go
            or self.last_processed_obs is None
            or self._obs_sanity_checks(obs, self.last_processed_obs)
        ):
            last_obs = self.last_processed_obs.get_timestep() if self.last_processed_obs else "None"
            self.logger.debug(
                f"Enqueuing observation. Must go: {obs.must_go} | Last processed obs: {last_obs}"
            )

            # If queue is full, get the old observation to make room
            if self.observation_queue.full():
                # pops from queue
                _ = self.observation_queue.get_nowait()
                self.logger.debug("Observation queue was full, removed oldest observation")

            # Now put the new observation (never blocks as queue is non-full here)
            self.observation_queue.put(obs)
            return True

        return False

    def _time_action_chunk(self, t_0: float, action_chunk: list[torch.Tensor], i_0: int) -> list[TimedAction]:
        """Turn a chunk of actions into a list of TimedAction instances,
        with the first action corresponding to t_0 and the rest corresponding to
        t_0 + i*environment_dt for i in range(len(action_chunk))
        """
        return [
            TimedAction(timestamp=t_0 + i * self.config.environment_dt, timestep=i_0 + i, action=action)
            for i, action in enumerate(action_chunk)
        ]

    def _prepare_observation_for_policy(self, observation: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
        """Populate observation history queues for policies that need them.

        Diffusion and similar policies maintain ``_queues`` of past observations.
        During normal env rollout, ``select_action`` fills these queues before calling
        ``predict_action_chunk``. The async server calls ``predict_action_chunk`` directly,
        so we must mirror that queue population here.
        """
        if self.policy_type not in POLICIES_REQUIRING_QUEUE_PREP:
            return observation

        batch = dict(observation)
        batch.pop(ACTION, None)

        config = self.policy.config
        if config.image_features:
            if hasattr(self.policy, "_queues") and OBS_IMAGE in self.policy._queues:
                batch[OBS_IMAGE] = batch[next(iter(config.image_features))]
            else:
                batch[OBS_IMAGES] = torch.stack([batch[key] for key in config.image_features], dim=-4)

        self.policy._queues = populate_queues(self.policy._queues, batch)
        return batch

    def _get_action_chunk(self, observation: dict[str, torch.Tensor]) -> torch.Tensor:
        """Get an action chunk from the policy. The chunk contains only"""
        observation = self._prepare_observation_for_policy(observation)
        chunk = self.policy.predict_action_chunk(observation)
        if chunk.ndim != 3:
            chunk = chunk.unsqueeze(0)  # adding batch dimension, now shape is (B, chunk_size, action_dim)

        return chunk[:, : self.actions_per_chunk, :]

    def _predict_action_chunk(self, observation_t: TimedObservation) -> list[TimedAction]:
        """Predict an action chunk based on an observation.

        Pipeline:
        1. Convert raw observation to LeRobot format
        2. Apply preprocessor (tokenization, normalization, batching, device placement)
        3. Run policy inference to get action chunk
        4. Apply postprocessor (unnormalization, device movement)
        5. Convert to TimedAction list
        """
        """1. Prepare observation"""
        start_prepare = time.perf_counter()
        observation: Observation = raw_observation_to_observation(
            observation_t.get_observation(),
            self.lerobot_features,
            self.policy_image_features,
            rename_map=self.observation_rename_map,
        )
        observation_for_trace = {k: v.clone() if isinstance(v, torch.Tensor) else v for k, v in observation.items()}
        prepare_time = time.perf_counter() - start_prepare

        """2. Apply preprocessor"""
        start_preprocess = time.perf_counter()
        observation = self.preprocessor(observation)
        self.last_processed_obs: TimedObservation = observation_t
        preprocessing_time = time.perf_counter() - start_preprocess

        """3. Get action chunk"""
        start_inference = time.perf_counter()
        action_tensor = self._get_action_chunk(observation)
        inference_time = time.perf_counter() - start_inference
        self.logger.info(
            f"Preprocessing and inference took {inference_time:.4f}s, action shape: {action_tensor.shape}"
        )

        """4. Apply postprocessor"""
        # Apply postprocessor (handles unnormalization and device movement)
        # Postprocessor expects (B, action_dim) per action, but we have (B, chunk_size, action_dim)
        # So we process each action in the chunk individually
        start_postprocess = time.perf_counter()
        _, chunk_size, _ = action_tensor.shape

        # Process each action in the chunk
        processed_actions = []
        for i in range(chunk_size):
            # Extract action at timestep i: (B, action_dim)
            single_action = action_tensor[:, i, :]
            processed_action = self.postprocessor(single_action)
            processed_actions.append(processed_action)

        # Stack back to (B, chunk_size, action_dim), then remove batch dim
        action_tensor = torch.stack(processed_actions, dim=1).squeeze(0)
        self.logger.debug(f"Postprocessed action shape: {action_tensor.shape}")

        action_tensor = action_tensor.detach().cpu()

        action_names = None
        if hasattr(self.policy.config, "action_feature_names"):
            action_names = getattr(self.policy.config, "action_feature_names", None)
        if action_names:
            action_names = [n.replace(".pos", "") for n in action_names]

        self.trace_recorder.record(
            timestep=observation_t.get_timestep(),
            timestamp=observation_t.get_timestamp(),
            policy_type=self.policy_type or "",
            pretrained_path=self.pretrained_path or "",
            raw_observation=observation_t.get_observation(),
            observation=observation_for_trace,
            action_chunk=action_tensor,
            action_names=action_names,
        )

        """5. Convert to TimedAction list"""
        action_chunk = self._time_action_chunk(
            observation_t.get_timestamp(), list(action_tensor), observation_t.get_timestep()
        )
        postprocess_stops = time.perf_counter()
        postprocessing_time = postprocess_stops - start_postprocess

        self.logger.info(
            f"Observation {observation_t.get_timestep()} | "
            f"Total time: {1000 * (postprocess_stops - start_prepare):.2f}ms"
        )

        self.logger.debug(
            f"Observation {observation_t.get_timestep()} | "
            f"Prepare time: {1000 * prepare_time:.2f}ms | "
            f"Preprocessing time: {1000 * preprocessing_time:.2f}ms | "
            f"Inference time: {1000 * inference_time:.2f}ms | "
            f"Postprocessing time: {1000 * postprocessing_time:.2f}ms | "
            f"Total time: {1000 * (postprocess_stops - start_prepare):.2f}ms"
        )

        return action_chunk

    def stop(self):
        """Stop the server"""
        self._reset_server()
        self.logger.info("Server stopping...")


@draccus.wrap()
def serve(cfg: PolicyServerConfig):
    """Start the PolicyServer with the given configuration.

    Args:
        config: PolicyServerConfig instance. If None, uses default configuration.
    """
    logging.info(pformat(asdict(cfg)))

    # Create the server instance first
    policy_server = PolicyServer(cfg)

    # Setup and start gRPC server
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=4))
    services_pb2_grpc.add_AsyncInferenceServicer_to_server(policy_server, server)
    server.add_insecure_port(f"{cfg.host}:{cfg.port}")

    policy_server.logger.info(f"PolicyServer started on {cfg.host}:{cfg.port}")
    policy_server.logger.info(f"Loaded from: {PolicyServer.__module__}")
    from .constants import SUPPORTED_POLICIES as _supported
    policy_server.logger.info(f"Supported policies: {_supported}")
    server.start()

    server.wait_for_termination()

    policy_server.logger.info("Server terminated")


if __name__ == "__main__":
    serve()
