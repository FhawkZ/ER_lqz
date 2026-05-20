"""MCP 右手局部系 -> Linker L6 dex joint_pos（21×3）。"""

from __future__ import annotations

import numpy as np

from .mocap_reader import MocapReader, MocapSkeletonFrame

MCP_LOCAL_TO_LINKER_RIGHT = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)
MCP_LOCAL_TO_LINKER_LEFT = np.array(
    [[0.0, 0.0, -1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]],
    dtype=np.float64,
)

_HAND_ROOT = {"right": "RightHand", "left": "LeftHand"}
_ALIGN = {"right": MCP_LOCAL_TO_LINKER_RIGHT, "left": MCP_LOCAL_TO_LINKER_LEFT}


def _prefix(hand_side: str) -> str:
    if hand_side not in _HAND_ROOT:
        raise ValueError("hand_side 只支持 right / left。")
    return "Right" if hand_side == "right" else "Left"


def _keypoints_21(local_pos: dict[str, np.ndarray], hand_side: str) -> np.ndarray:
    p = _prefix(hand_side)

    def v(name: str) -> np.ndarray:
        return np.asarray(local_pos[f"{p}{name}"], dtype=np.float64)

    t1, t2, t3 = v("HandThumb1"), v("HandThumb2"), v("HandThumb3")
    k = np.zeros((21, 3), dtype=np.float64)
    k[0] = v("Hand")
    k[1], k[3], k[4] = t1, t2, t3
    k[2] = 0.5 * (t1 + t2)
    k[5], k[6], k[7], k[8] = v("InHandIndex"), v("HandIndex1"), v("HandIndex2"), v("HandIndex3")
    k[9], k[10], k[11], k[12] = v("InHandMiddle"), v("HandMiddle1"), v("HandMiddle2"), v("HandMiddle3")
    k[13], k[14], k[15], k[16] = v("InHandRing"), v("HandRing1"), v("HandRing2"), v("HandRing3")
    k[17], k[18], k[19], k[20] = v("InHandPinky"), v("HandPinky1"), v("HandPinky2"), v("HandPinky3")
    return k


def frame_to_dex_joint_pos(
    reader: MocapReader,
    frame: MocapSkeletonFrame,
    hand_side: str = "right",
    position_scale: float = 0.01,
) -> np.ndarray:
    root = _HAND_ROOT[hand_side]
    rel = reader.build_subtree_relative_transforms(frame, root)
    scale = float(position_scale)
    local_pos = {n: np.asarray(tf.position, dtype=np.float64) * scale for n, tf in rel.items()}
    return _keypoints_21(local_pos, hand_side) @ _ALIGN[hand_side]
