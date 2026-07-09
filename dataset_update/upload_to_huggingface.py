#!/usr/bin/env python3
"""将本地 LeRobot 数据集上传到 Hugging Face Hub。

用法（lerobot conda 环境）::

    conda activate lerobot
    pip install -r dataset_update/requirements.txt
    cp dataset_update/.env.example dataset_update/.env   # 可选：填写 HF_TOKEN

    # 登录（若未在 .env 中设置 HF_TOKEN）
    huggingface-cli login

    # 上传本地数据集（HF repo_id 默认 {HF_REPO_NAMESPACE}/{本地 repo_id}）
    python dataset_update/upload_to_huggingface.py \\
      --dataset /home/franka/lqz/Data/franka_hand/joint-assemble-20260708

    # 指定 Hub 上的完整 repo_id
    python dataset_update/upload_to_huggingface.py \\
      --repo-id franka_hand/joint-assemble-20260708 \\
      --hf-repo-id your-username/joint-assemble-20260708 \\
      --private

    # 仅预览
    python dataset_update/upload_to_huggingface.py \\
      --dataset /home/franka/lqz/Data/franka_hand/joint-assemble-20260708 \\
      --dry-run
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from dataset_common import collect_local_files, load_env_file, resolve_dataset

load_env_file()

REPO_ROOT = Path(__file__).resolve().parents[1]
LEROBOT_SRC = REPO_ROOT / "lerobot" / "src"
if str(LEROBOT_SRC) not in sys.path:
    sys.path.insert(0, str(LEROBOT_SRC))


def resolve_hf_repo_id(hf_repo_id: str | None, local_repo_id: str) -> str:
    if hf_repo_id:
        return hf_repo_id.strip("/")

    namespace = os.environ.get("HF_REPO_NAMESPACE", "").strip().strip("/")
    if not namespace:
        raise ValueError(
            "请通过 --hf-repo-id 指定 Hub 仓库（如 your-username/dataset-name），"
            "或在 .env 中设置 HF_REPO_NAMESPACE。"
        )
    return f"{namespace}/{local_repo_id.strip('/')}"


def build_parser() -> argparse.ArgumentParser:
    from dataset_common import add_dataset_args

    parser = argparse.ArgumentParser(description="上传 LeRobot 数据集到 Hugging Face Hub")
    add_dataset_args(parser)
    parser.add_argument(
        "--hf-repo-id",
        default=None,
        help="Hub 上的数据集 repo_id（默认 {HF_REPO_NAMESPACE}/{本地 repo_id}）",
    )
    parser.add_argument(
        "--private",
        action="store_true",
        help="创建/上传到私有数据集仓库",
    )
    parser.add_argument(
        "--push-videos",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="是否上传 videos/ 目录（默认上传）",
    )
    parser.add_argument(
        "--upload-large-folder",
        action="store_true",
        help="使用 upload_large_folder（适合超大数据集）",
    )
    parser.add_argument(
        "--tag",
        action="append",
        dest="tags",
        default=None,
        help="数据集标签，可重复指定",
    )
    parser.add_argument(
        "--license",
        default="apache-2.0",
        help="数据集许可证（默认 apache-2.0）",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印将要上传的信息，不实际上传",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    local_path, local_repo_id = resolve_dataset(args.dataset, args.repo_id, args.dataset_root)
    hf_repo_id = resolve_hf_repo_id(args.hf_repo_id, local_repo_id)

    files = collect_local_files(local_path)
    print(f"📦 本地目录: {local_path}")
    print(f"🏷️  本地 repo_id: {local_repo_id}")
    print(f"🤗 Hub repo_id: {hf_repo_id}")
    print(f"📄 本地文件数: {len(files)}")
    print(f"🔒 私有仓库: {args.private}")
    print(f"🎬 上传视频: {args.push_videos}")

    if args.dry_run:
        print("🔍 dry-run 模式，不会实际上传")
        for _, relative_path in files[:10]:
            print(f"  -> {relative_path}")
        if len(files) > 10:
            print(f"  ... 还有 {len(files) - 10} 个文件")
        return 0

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(repo_id=hf_repo_id, root=local_path)
    dataset.push_to_hub(
        private=args.private,
        push_videos=args.push_videos,
        upload_large_folder=args.upload_large_folder,
        tags=args.tags,
        license=args.license,
    )

    print(f"\n🎉 已上传到 https://huggingface.co/datasets/{hf_repo_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
