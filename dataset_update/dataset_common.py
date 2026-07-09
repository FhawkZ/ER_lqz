"""数据集上传/删除脚本的公共路径解析与环境变量加载。"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

DEFAULT_DATASET_ROOT = os.environ.get("DATASET_ROOT", "/home/franka/lqz/Data")


def load_env_file(env_path: Path | None = None) -> None:
    """从 dataset_update/.env 加载环境变量（不覆盖已设置的变量）。"""
    if env_path is None:
        env_path = Path(__file__).resolve().parent / ".env"
    if not env_path.is_file():
        return

    for line in env_path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, _, value = stripped.partition("=")
        key = key.strip()
        value = value.strip().strip("'").strip('"')
        if key and key not in os.environ:
            os.environ[key] = value


def add_dataset_args(parser: argparse.ArgumentParser) -> None:
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        "--dataset",
        metavar="PATH",
        help="本地 LeRobot 数据集目录，例如 /home/franka/lqz/Data/franka_hand/joint-assemble-20260708",
    )
    group.add_argument(
        "--repo-id",
        help="数据集 repo_id（需配合 --dataset-root），例如 franka_hand/joint-assemble-20260708",
    )
    parser.add_argument(
        "--dataset-root",
        default=DEFAULT_DATASET_ROOT,
        help=f"数据集根目录（默认: {DEFAULT_DATASET_ROOT}）",
    )


def resolve_dataset(
    dataset: str | None,
    repo_id: str | None,
    dataset_root: str,
) -> tuple[Path, str]:
    root = Path(dataset_root).expanduser().resolve()

    if dataset:
        local_path = Path(dataset).expanduser().resolve()
        if not local_path.is_dir():
            raise FileNotFoundError(f"数据集目录不存在: {local_path}")
        info_json = local_path / "meta" / "info.json"
        if not info_json.is_file():
            raise ValueError(f"不是有效的 LeRobot 数据集（缺少 meta/info.json）: {local_path}")

        if repo_id:
            resolved_repo_id = repo_id
        else:
            try:
                resolved_repo_id = local_path.relative_to(root).as_posix()
            except ValueError:
                resolved_repo_id = local_path.name
        return local_path, resolved_repo_id

    if repo_id:
        local_path = (root / repo_id).resolve()
        if not local_path.is_dir():
            raise FileNotFoundError(f"数据集目录不存在: {local_path}")
        info_json = local_path / "meta" / "info.json"
        if not info_json.is_file():
            raise ValueError(f"不是有效的 LeRobot 数据集（缺少 meta/info.json）: {local_path}")
        return local_path, repo_id

    raise ValueError("必须指定 --dataset 或 --repo-id")


def collect_local_files(local_folder: Path) -> list[tuple[Path, str]]:
    files: list[tuple[Path, str]] = []
    for root, _, filenames in os.walk(local_folder):
        for filename in filenames:
            local_file_path = Path(root) / filename
            relative_path = local_file_path.relative_to(local_folder).as_posix()
            files.append((local_file_path, relative_path))
    return files
