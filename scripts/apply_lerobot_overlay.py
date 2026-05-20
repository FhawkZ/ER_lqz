#!/usr/bin/env python3
"""在 upstream lerobot 上打 ER_lqz 集成补丁（utils / scripts / async_inference）。"""

from __future__ import annotations

import sys
from pathlib import Path


def _insert_before(text: str, marker: str, block: str) -> str:
    if block.strip() in text:
        return text
    idx = text.find(marker)
    if idx < 0:
        raise ValueError(f"marker not found: {marker!r}")
    return text[:idx] + block + text[idx:]


def patch_teleoperators_utils(ler: Path) -> None:
    path = ler / "src/lerobot/teleoperators/utils.py"
    text = path.read_text()
    block = """    elif config.type == "fr3_leader":
        from .fr3_leader import FR3Leader

        return FR3Leader(config)
    elif config.type == "mocap_leader":
        from .mocap_leader import MocapLeader

        return MocapLeader(config)
    elif config.type == "mocap_eef_leader":
        from .mocap_eef_leader import MocapEefLeader

        return MocapEefLeader(config)
    elif config.type == "mocap_retarget_leader":
        from .mocap_retarget import MocapRetargetLeader

        return MocapRetargetLeader(config)
"""
    text = _insert_before(text, '    elif config.type == "openarm_leader":', block)
    path.write_text(text)
    print(f"patched {path.relative_to(ler)}")


def patch_robots_utils(ler: Path) -> None:
    path = ler / "src/lerobot/robots/utils.py"
    text = path.read_text()
    block = """    elif config.type == "fr3_follower":
        from .fr3_follower import FR3Follower

        return FR3Follower(config)
    elif config.type == "fr3_linker_l6_follower":
        from .fr3_linker_l6_follower import FR3LinkerL6Follower

        return FR3LinkerL6Follower(config)
    elif config.type == "fr3_eef":
        from .fr3_eef import FR3EEF

        return FR3EEF(config)
"""
    text = _insert_before(text, '    elif config.type == "lekiwi":', block)
    path.write_text(text)
    print(f"patched {path.relative_to(ler)}")


def patch_async_constants(ler: Path) -> None:
    path = ler / "src/lerobot/async_inference/constants.py"
    text = path.read_text()
    old = 'SUPPORTED_ROBOTS = ["so100_follower", "so101_follower", "bi_so_follower", "omx_follower"]'
    new = """SUPPORTED_ROBOTS = [
    "so100_follower",
    "so101_follower",
    "bi_so_follower",
    "omx_follower",
    "fr3_follower",
    "fr3_linker_l6_follower",
    "fr3_eef",
]"""
    if old in text:
        text = text.replace(old, new)
    path.write_text(text)
    print(f"patched {path.relative_to(ler)}")


def patch_async_robot_client(ler: Path) -> None:
    path = ler / "src/lerobot/async_inference/robot_client.py"
    text = path.read_text()
    block = """    fr3_eef,
    fr3_follower,
    fr3_linker_l6_follower,
"""
    text = _insert_before(text, "    koch_follower,", block)
    path.write_text(text)
    print(f"patched {path.relative_to(ler)}")


def _add_import_line(text: str, anchor: str, line: str) -> str:
    if line in text:
        return text
    return text.replace(anchor, anchor + "\n" + line)


def patch_script_imports(path: Path, robot_lines: list[str], teleop_lines: list[str]) -> None:
    text = path.read_text()
    for line in robot_lines:
        text = _add_import_line(text, "    make_robot_from_config,", f"    {line},")
    for line in teleop_lines:
        text = _add_import_line(text, "    make_teleoperator_from_config,", f"    {line},")
    path.write_text(text)
    print(f"patched {path.name}")


def patch_lerobot_teleoperate(ler: Path, backup: Path) -> None:
    """在 0.5.1 teleoperate 上合并 enable_robot / reset_incremental_pose / FR3 imports。"""
    path = ler / "src/lerobot/scripts/lerobot_teleoperate.py"
    text = path.read_text()

    robot_imports = [
        "fr3_eef",
        "fr3_follower",
        "fr3_linker_l6_follower",
    ]
    teleop_imports = [
        "fr3_leader",
        "mocap_eef_leader",
        "mocap_leader",
        "mocap_retarget",
    ]
    for line in robot_imports:
        text = _add_import_line(text, "    earthrover_mini_plus,", f"    {line},")
    for line in teleop_imports:
        text = _add_import_line(text, "    koch_leader,", f"    {line},")

    if "enable_robot" not in text:
        text = text.replace(
            "    robot: RobotConfig\n    # Limit the maximum frames per second.",
            "    robot: RobotConfig | None = None\n    # Allow teleop-only runs without a follower.\n    enable_robot: bool = True\n    # Limit the maximum frames per second.",
        )

    if "robot: Robot | None" not in text:
        text = text.replace(
            "def teleop_loop(\n    teleop: Teleoperator,\n    robot: Robot,",
            "def teleop_loop(\n    teleop: Teleoperator,\n    robot: Robot | None,",
        )

    if "if robot is None:" not in text or "display_len = 0" not in text:
        text = text.replace(
            "    display_len = max(len(key) for key in robot.action_features)\n    start = time.perf_counter()\n    while True:",
            """    if robot is None:
        display_len = 0
    else:
        display_len = max(len(key) for key in robot.action_features)
    start = time.perf_counter()

    while True:""",
        )
        text = text.replace(
            "        obs = robot.get_observation()\n\n        if robot.name == \"unitree_g1\":",
            """        if robot is None:
            obs = {}
        else:
            obs = robot.get_observation()

        if robot is not None and robot.name == "unitree_g1":""",
        )
        text = text.replace(
            "        _ = robot.send_action(robot_action_to_send)\n\n        if display_data:",
            """        if robot is not None:
            _ = robot.send_action(robot_action_to_send)

        if display_data and robot is not None:""",
        )

    if "reset_incremental_pose" not in text:
        text = text.replace(
            "    teleop.connect()\n    robot.connect()\n\n    try:",
            """    robot = None
    teleop.connect()
    if cfg.enable_robot:
        if cfg.robot is None:
            raise ValueError("enable_robot=True requires robot config.")
        robot = make_robot_from_config(cfg.robot)
        robot.connect()
        if hasattr(robot, "reset_motion_state"):
            robot.reset_motion_state()
    else:
        robot = None
    if hasattr(teleop, "reset_incremental_pose"):
        teleop.reset_incremental_pose()

    try:""",
        )
        text = text.replace(
            "    teleop = make_teleoperator_from_config(cfg.teleop)\n    robot = make_robot_from_config(cfg.robot)\n",
            "    teleop = make_teleoperator_from_config(cfg.teleop)\n",
        )
        text = text.replace(
            "        robot.disconnect()\n\n\ndef main():",
            """        if robot is not None:
            robot.disconnect()


def main():""",
        )
        if "if not cfg.enable_robot" not in text:
            text = text.replace(
                "    logging.info(pformat(asdict(cfg)))\n    if cfg.display_data:",
                """    logging.info(pformat(asdict(cfg)))
    if not cfg.enable_robot and cfg.robot is not None:
        raise ValueError("When enable_robot is False, do not provide robot config.")
    if cfg.display_data and not cfg.enable_robot:
        raise ValueError("display_data requires enable_robot=True.")
    if cfg.display_data:""",
            )

    path.write_text(text)
    print(f"patched {path.relative_to(ler)}")


def patch_lerobot_record(ler: Path) -> None:
    path = ler / "src/lerobot/scripts/lerobot_record.py"
    patch_script_imports(
        path,
        ["fr3_eef", "fr3_follower", "fr3_linker_l6_follower"],
        ["fr3_leader", "mocap_eef_leader", "mocap_leader", "mocap_retarget"],
    )
    text = path.read_text()
    marker = "    # Reset policy and processor if they are provided"
    block = """    # Reset Mocap EE incremental pose at episode boundaries.
    if teleop is not None and hasattr(teleop, "reset_incremental_pose"):
        teleop.reset_incremental_pose()
    elif isinstance(teleop, list):
        for t in teleop:
            if hasattr(t, "reset_incremental_pose"):
                t.reset_incremental_pose()
                break

    if hasattr(robot, "reset_motion_state"):
        robot.reset_motion_state()

"""
    if "reset_incremental_pose" not in text and marker in text:
        text = text.replace(marker, block + marker)
        path.write_text(text)


def patch_other_scripts(ler: Path) -> None:
    for name, robots, teleops in [
        (
            "lerobot_replay.py",
            ["fr3_follower", "fr3_linker_l6_follower"],
            [],
        ),
        (
            "lerobot_calibrate.py",
            ["fr3_follower", "fr3_linker_l6_follower"],
            ["fr3_leader", "mocap_leader"],
        ),
        (
            "lerobot_setup_motors.py",
            ["fr3_follower", "fr3_linker_l6_follower"],
            ["fr3_leader", "mocap_leader"],
        ),
        (
            "lerobot_find_joint_limits.py",
            ["fr3_follower", "fr3_linker_l6_follower"],
            ["fr3_leader", "mocap_leader"],
        ),
    ]:
        path = ler / "src/lerobot/scripts" / name
        if not path.exists():
            continue
        patch_script_imports(path, robots, teleops)
        if name == "lerobot_setup_motors.py":
            text = path.read_text()
            for t in ["fr3_follower", "fr3_leader", "fr3_linker_l6_follower", "mocap_leader"]:
                if f'"{t}"' not in text:
                    text = text.replace(
                        '"so101_follower",',
                        f'"so101_follower",\n    "{t}",',
                        1,
                    )
            path.write_text(text)


def main() -> None:
    if len(sys.argv) < 2:
        print("usage: apply_lerobot_overlay.py LEROBOT_DIR [BACKUP_DIR]", file=sys.stderr)
        sys.exit(1)
    ler = Path(sys.argv[1]).resolve()
    backup = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else None

    patch_teleoperators_utils(ler)
    patch_robots_utils(ler)
    patch_async_constants(ler)
    patch_async_robot_client(ler)
    patch_lerobot_teleoperate(ler, backup or ler)
    patch_lerobot_record(ler)
    patch_other_scripts(ler)


if __name__ == "__main__":
    main()
