#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
from pathlib import Path

import numpy as np
from mocap_robotapi import (
    MCPApplication,
    MCPAvatar,
    MCPSettings,
    MCPEventType,
)

links_parent = {
    "Hips": "world",
    "RightShoulder": "Spine2",
    "RightArm": "RightShoulder",
    "RightForeArm": "RightArm",
    "RightHand": "RightForeArm",
    "LeftShoulder": "Spine2",
    "LeftArm": "LeftShoulder",
    "LeftForeArm": "LeftArm",
    "LeftHand": "LeftForeArm",
}


def axis_to_ros_position(position):
    return (position[2] / 100.0, position[0] / 100.0, position[1] / 100.0)


def axis_to_ros_quaternion(rotation):
    return (rotation[3], rotation[1], rotation[2], rotation[0])


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_rotate(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    return quat_multiply(quat_multiply(q, qv), quat_conjugate(q))[:3]


def compose_transform(parent_pos, parent_quat, local_pos, local_quat):
    rot_local = quat_rotate(parent_quat, local_pos)
    pos = (parent_pos[0] + rot_local[0], parent_pos[1] + rot_local[1], parent_pos[2] + rot_local[2])
    quat = quat_multiply(parent_quat, local_quat)
    return pos, quat


def build_local_transforms(avatar):
    transforms = {}
    for joint in avatar.get_joints():
        name = joint.get_name()
        lp = joint.get_local_position()
        lr = joint.get_local_rotation()
        if lp is None or lr is None:
            continue
        transforms[name] = (axis_to_ros_position(lp), axis_to_ros_quaternion(lr))
    return transforms


def get_global_transform(name, local_transforms, cache):
    if name in cache:
        return cache[name]
    if name not in local_transforms:
        return None
    parent = links_parent.get(name, "world")
    if parent == "world":
        cache[name] = local_transforms[name]
        return cache[name]
    p_tf = get_global_transform(parent, local_transforms, cache)
    if p_tf is None:
        return None
    cache[name] = compose_transform(p_tf[0], p_tf[1], local_transforms[name][0], local_transforms[name][1])
    return cache[name]


def aligned_delta(delta):
    # same as MocapLeader: X_robot=Y_mocap, Y_robot=-X_mocap, Z_robot=Z_mocap
    return np.array([delta[1], -delta[0], delta[2]], dtype=np.float64)


def vector_label(v):
    return f"[{v[0]:+.4f}, {v[1]:+.4f}, {v[2]:+.4f}]"


def main():
    app = MCPApplication()
    settings = MCPSettings()
    settings.set_udp(7012)
    settings.set_bvh_rotation(0)
    app.set_settings(settings)
    app.open()

    print("实时模式：等待 mocap 数据，按 Ctrl-C 退出")
    prev_pos = None
    sample = 0

    try:
        while True:
            evts = app.poll_next_event()
            for evt in evts:
                if evt.event_type != MCPEventType.AvatarUpdated:
                    continue
                avatar = MCPAvatar(evt.event_data.avatar_handle)
                local = build_local_transforms(avatar)
                g_right = get_global_transform("RightHand", local, {})
                if g_right is None:
                    continue

                pos = np.array(g_right[0], dtype=np.float64)
                if prev_pos is None:
                    prev_pos = pos
                    print("初始 RightHand 位置：", vector_label(pos))
                    continue

                delta = pos - prev_pos
                aligned = aligned_delta(delta)
                sample += 1
                print(
                    f"#{sample} 位置={vector_label(pos)} | delta={vector_label(delta)} | aligned={vector_label(aligned)}"
                )
                prev_pos = pos

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("已停止")
    finally:
        app.close()


if __name__ == "__main__":
    main()
