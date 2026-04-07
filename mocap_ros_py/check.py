#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import json
from pathlib import Path

import numpy as np
from mocap_robotapi import (
    MCPApplication,
    MCPAvatar,
    MCPRobot,
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
    # ... keep 对应 mocap_leader 关系
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
    # 参考 mocap_leader: X_r=y, Y_r=-x, Z_r=z
    return np.array([delta[1], -delta[0], delta[2]])


def vector_label(v):
    return f"[{v[0]:+.4f}, {v[1]:+.4f}, {v[2]:+.4f}]"


def main():
    app = MCPApplication()
    settings = MCPSettings()
    settings.set_udp(7012)
    settings.set_bvh_rotation(0)
    app.set_settings(settings)
    app.open()

    cfg = Path("unitree_h1/retarget.json")
    if not cfg.exists():
        raise FileNotFoundError("请先准备 retarget.json")
    robot = MCPRobot(cfg.read_text())

    print("等待 mocap 数据（5秒）... 建议先静止，在脚本提示时进行前/上移动")
    start_pose = None
    count = 0
    logged_step = 0
    while True:
        evts = app.poll_next_event()
        for evt in evts:
            if evt.event_type != MCPEventType.AvatarUpdated:
                continue
            avatar = MCPAvatar(evt.event_data.avatar_handle)
            robot.update_robot(avatar)
            robot.run_robot_step()

            local = build_local_transforms(avatar)
            g_right = get_global_transform("RightHand", local, {})
            if g_right is None:
                continue
            pos = np.array(g_right[0], dtype=np.float64)

            if start_pose is None:
                start_pose = pos
                print("基准位置记录:", vector_label(start_pose))
                logged_step += 1
                count += 1
                continue

            if count == 1:
                print("请将右手向前水平移动（保持高度），然后按回车")
                input()
                pos_forward = pos
                delta_forward = pos_forward - start_pose
                logged_step += 1
                print(f"第{logged_step}次记录: pos={vector_label(pos_forward)}, delta={vector_label(delta_forward)}")
                aligned_fwd = aligned_delta(delta_forward)
                print("前移差值（原 mocap->ROS）：", vector_label(delta_forward))
                print("fr3映射对齐：", vector_label(aligned_fwd))
                print(
                    "判断: 认证前进指向 >0分量：",
                    f"X={aligned_fwd[0]:+.4f}, Y={aligned_fwd[1]:+.4f}, Z={aligned_fwd[2]:+.4f}",
                )
                count += 1
                continue

            if count == 2:
                print("请将右手向上移动（保持水平），然后按回车")
                input()
                pos_up = pos
                logged_step += 1
                delta_up_raw = pos_up - start_pose
                print(f"第{logged_step}次记录: pos={vector_label(pos_up)}, delta(相对基准)={vector_label(delta_up_raw)}")
                delta_up = pos_up - pos_forward
                aligned_up = aligned_delta(delta_up)
                print("上移差值（原 mocap->ROS）：", vector_label(delta_up))
                print("fr3映射对齐：", vector_label(aligned_up))
                print(
                    "判断: 认证向上指向 >0分量：",
                    f"X={aligned_up[0]:+.4f}, Y={aligned_up[1]:+.4f}, Z={aligned_up[2]:+.4f}",
                )
                print("完成。")
                app.close()
                return 0

        time.sleep(0.01)


if __name__ == "__main__":
    exit(main())