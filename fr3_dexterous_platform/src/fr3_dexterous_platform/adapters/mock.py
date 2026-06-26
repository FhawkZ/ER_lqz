"""Mock robot and action sources for tests and dry runs."""

from __future__ import annotations

from fr3_dexterous_platform.interfaces import ActionSource, RobotBackend
from fr3_dexterous_platform.schemas import Action, Observation, StampedValue
from fr3_dexterous_platform.utils.time import now_s


class MockFr3LinkerRobot(RobotBackend):
    def __init__(self, stale_camera: bool = False):
        self.connected = False
        self.seq = 0
        self.sent_actions: list[Action] = []
        self.stale_camera = stale_camera

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def get_observation(self) -> Observation:
        t = now_s()
        cam_t = t - 1.0 if self.stale_camera else t
        seq = self.seq
        self.seq += 1
        values = {
            "arm.ee_pose": StampedValue(
                {"x": seq * 0.001, "y": 0.0, "z": 0.4, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
                t,
                t,
                "/franka/ee_pose",
                sequence_id=seq,
            ),
            "hand.joints": StampedValue({"hand_0": float(seq % 255)}, t, t, "/linkerhand/right/joint_states", sequence_id=seq),
            "image.handeye": StampedValue({"frame": seq, "camera": "handeye"}, cam_t, t, "/camera/handeye/color/image_rect_raw", sequence_id=seq),
            "image.fixed": StampedValue({"frame": seq, "camera": "fixed"}, cam_t, t, "/camera/fixed/color/image_rect_raw", sequence_id=seq),
        }
        return Observation(values=values, receive_time=t)

    def send_action(self, action: Action) -> Action:
        sent = Action(values=dict(action.values), source_time=action.source_time, receive_time=now_s(), trace={"sent": True})
        self.sent_actions.append(sent)
        return sent


class ScriptedTeleopSource(ActionSource):
    def __init__(self):
        self.connected = False
        self.seq = 0

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def reset(self) -> None:
        self.seq = 0

    def get_action(self, observation: Observation) -> Action:
        t = now_s()
        arm = observation.values["arm.ee_pose"].value
        hand = observation.values["hand.joints"].value
        action = {
            "ee_x.pos": float(arm["x"]) + 0.001,
            "ee_y.pos": float(arm["y"]),
            "ee_z.pos": float(arm["z"]),
            "ori_qx.pos": 0.0,
            "ori_qy.pos": 0.0,
            "ori_qz.pos": 0.0,
            "ori_qw.pos": 1.0,
            "hand_0.pos": float(hand["hand_0"]),
        }
        self.seq += 1
        return Action(values=action, source_time=t, receive_time=t, trace={"source": "mock_teleop", "sequence_id": self.seq})
