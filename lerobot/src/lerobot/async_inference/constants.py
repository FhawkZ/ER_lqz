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

"""Client side: The environment evolves with a time resolution equal to 1/fps"""

DEFAULT_FPS = 30

"""Server side: Running inference on (at most) 1/fps"""
DEFAULT_INFERENCE_LATENCY = 1 / DEFAULT_FPS

"""Server side: Timeout for observation queue in seconds"""
DEFAULT_OBS_QUEUE_TIMEOUT = 2

# All action chunking policies
SUPPORTED_POLICIES = ["act", "smolvla", "diffusion", "tdmpc", "vqbet", "pi0", "pi05", "groot", "xvla"]

# Policies whose predict_action_chunk reads from _queues but does not populate them
# (populate_queues is only called from select_action during normal rollout).
POLICIES_REQUIRING_QUEUE_PREP = frozenset({"diffusion", "vqbet", "tdmpc", "multi_task_dit"})

# TODO: Add all other robots
SUPPORTED_ROBOTS = [
    "so100_follower",
    "so101_follower",
    "bi_so_follower",
    "omx_follower",
    "fr3_follower",
    "fr3_linker_l6_follower",
    "fr3_eef",
]
