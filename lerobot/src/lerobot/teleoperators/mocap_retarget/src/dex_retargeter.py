"""dex-retargeting 封装（Linker L6 vector 模式）。"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


def default_dex_paths() -> tuple[Path | None, Path, Path]:
    """默认路径：URDF 使用包内 ``assets/``；dex 库优先 pip，其次本地 clone。"""
    root = Path(__file__).resolve().parents[1]
    repo_root = root.parent
    dex_repo = (repo_root / "dex-retargeting").resolve()
    if not (dex_repo / "src").is_dir():
        dex_repo = None
    return (
        dex_repo,
        (root / "assets").resolve(),
        (root / "config" / "dex_linker_l6_right.yml").resolve(),
    )


def _ensure_dex_import(dex_repo_path: Path | None) -> None:
    try:
        import dex_retargeting  # noqa: F401
        return
    except ImportError:
        pass
    if dex_repo_path is None:
        raise ImportError(
            "未找到 dex-retargeting：请执行 pip install dex-retargeting，"
            "或将源码 clone 到 lerobot/.../teleoperators/dex-retargeting"
        )
    src = dex_repo_path / "src"
    if not src.is_dir():
        raise FileNotFoundError(f"dex-retargeting 源码目录不存在：{src}")
    p = str(src)
    if p not in sys.path:
        sys.path.insert(0, p)


class DexRetargeter:
    def __init__(
        self,
        dex_repo: str | Path | None,
        urdf_root: str | Path,
        dex_config: str | Path,
    ) -> None:
        self.urdf_root = Path(urdf_root).resolve()
        self.dex_config_path = Path(dex_config).resolve()
        if not self.urdf_root.is_dir():
            raise FileNotFoundError(f"URDF 根目录不存在：{self.urdf_root}")
        if not self.dex_config_path.is_file():
            raise FileNotFoundError(f"dex 配置不存在：{self.dex_config_path}")

        dex_repo_path = Path(dex_repo).resolve() if dex_repo else None
        _ensure_dex_import(dex_repo_path)
        from dex_retargeting.retargeting_config import RetargetingConfig

        RetargetingConfig.set_default_urdf_dir(self.urdf_root)
        self._config = RetargetingConfig.load_from_file(self.dex_config_path)
        self.resolved_urdf_path = Path(self._config.urdf_path).resolve()
        if not self.resolved_urdf_path.is_file():
            raise FileNotFoundError(f"URDF 不存在：{self.resolved_urdf_path}")
        self._rt = self._config.build()
        self._human_indices = np.asarray(self._rt.optimizer.target_link_human_indices)

    @property
    def target_joint_names(self) -> list[str]:
        names = self._config.target_joint_names
        if not names:
            raise ValueError("dex 配置未指定 target_joint_names。")
        return list(names)

    def target_qpos(self, robot_qpos: np.ndarray) -> np.ndarray:
        dof_names = self._rt.optimizer.robot.dof_joint_names
        idx = {n: i for i, n in enumerate(dof_names)}
        q = np.asarray(robot_qpos, dtype=np.float64).reshape(-1)
        return np.array([q[idx[n]] for n in self.target_joint_names], dtype=np.float64)

    def retarget(self, joint_pos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        idx = self._human_indices
        ref = joint_pos[idx[1], :] - joint_pos[idx[0], :]
        qpos = self._rt.retarget(ref)
        return ref, np.asarray(qpos, dtype=np.float64)
