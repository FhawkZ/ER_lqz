#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
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

"""Quaternion helpers.

Convention (arm EE chain: MCP -> leader -> dataset -> fr3_eef -> ROS/Franka):

    * **LeRobot / scipy / ``geometry_msgs.msg.Quaternion``**: scalar-last
      ``[x, y, z, w]`` (``ori_qx`` … ``ori_qw``).
    * **MCP ``get_local_rotation()``**: scalar-first ``(w, x, y, z)``; converted
      by ``axis_to_ros_quaternion`` in ``mocap_eef_leader`` before FK / deltas.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation


def normalize_quat_xyzw(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return q / norm


def align_quat_hemisphere(q: np.ndarray, q_ref: np.ndarray | None) -> np.ndarray:
    """Pick the quaternion hemisphere continuous w.r.t. ``q_ref``.

    If ``q · q_ref < 0``, return ``-q`` (same rotation, opposite sign).
    """
    q = normalize_quat_xyzw(q)
    if q_ref is None:
        return q
    q_ref = normalize_quat_xyzw(q_ref)
    if float(np.dot(q, q_ref)) < 0.0:
        return -q
    return q


def prepare_quat_xyzw(
    q: np.ndarray,
    *,
    q_prev: np.ndarray | None = None,
    q_anchor: np.ndarray | None = None,
) -> np.ndarray:
    """Normalize and sign-align quaternions for dataset / policy I/O.

    * **Within episode** (``q_prev`` set): ``q_i · q_{i-1} >= 0``.
    * **Episode first frame** (``q_prev is None``): if ``q_anchor`` is set,
      ``q · q_anchor >= 0`` (e.g. leader action vs robot ``current_pose`` /
      observation on frame 0). If both are unset, only ``normalize``.
    """
    q_ref = q_prev if q_prev is not None else q_anchor
    return align_quat_hemisphere(q, q_ref)


def orientation_delta_deg(q0_xyzw: np.ndarray, q1_xyzw: np.ndarray) -> float:
    """Geodesic rotation angle (degrees) between two orientations."""
    r0 = Rotation.from_quat(normalize_quat_xyzw(q0_xyzw))
    r1 = Rotation.from_quat(normalize_quat_xyzw(q1_xyzw))
    return float(np.degrees((r0.inv() * r1).magnitude()))
