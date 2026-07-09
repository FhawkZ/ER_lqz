#!/usr/bin/env python3
"""将本地 LeRobot 数据集上传到 MinIO。

用法（lerobot conda 环境）::

    conda activate lerobot
    pip install -r dataset_update/requirements.txt
    cp dataset_update/.env.example dataset_update/.env   # 填写 MinIO 凭据

    # 指定本地数据集目录（推荐）
    python dataset_update/upload_to_minio.py \\
      --dataset /home/franka/lqz/Data/franka_hand/joint-assemble-20260708

    # 或使用 repo_id + 根目录
    python dataset_update/upload_to_minio.py \\
      --repo-id franka_hand/joint-assemble-20260708

    # 仅预览，不上传
    python dataset_update/upload_to_minio.py \\
      --dataset /home/franka/lqz/Data/franka_hand/joint-assemble-20260708 \\
      --dry-run
"""

from __future__ import annotations

import argparse
from pathlib import Path

from minio.error import S3Error

from dataset_common import collect_local_files, resolve_dataset
from minio_common import (
    add_minio_args,
    create_minio_client,
    resolve_minio_config,
    resolve_minio_prefix,
)


def upload_folder_to_minio(
    local_folder: Path,
    bucket_name: str,
    minio_client,
    minio_prefix: str,
    *,
    dry_run: bool = False,
) -> tuple[int, int]:
    files = collect_local_files(local_folder)
    if not files:
        print(f"⚠️  目录为空，无可上传文件: {local_folder}")
        return 0, 0

    prefix = minio_prefix.strip("/")
    success_count = 0
    fail_count = 0

    print(f"📦 本地目录: {local_folder}")
    print(f"☁️  MinIO 目标: {bucket_name}/{prefix}/")
    print(f"📄 待上传文件数: {len(files)}")
    if dry_run:
        print("🔍 dry-run 模式，不会实际上传")
        for _, relative_path in files[:10]:
            print(f"  -> {prefix}/{relative_path}")
        if len(files) > 10:
            print(f"  ... 还有 {len(files) - 10} 个文件")
        return len(files), 0

    for local_file_path, relative_path in files:
        minio_file_path = f"{prefix}/{relative_path}"
        try:
            minio_client.fput_object(
                bucket_name=bucket_name,
                object_name=minio_file_path,
                file_path=str(local_file_path),
                part_size=1024 * 1024 * 128,
            )
            success_count += 1
            print(f"✅ {relative_path}")
        except S3Error as exc:
            fail_count += 1
            print(f"❌ {relative_path}: {exc}")

    return success_count, fail_count


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="上传 LeRobot 数据集到 MinIO")
    add_minio_args(parser)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只打印将要上传的对象，不实际上传",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    local_path, repo_id = resolve_dataset(args.dataset, args.repo_id, args.dataset_root)
    endpoint, access_key, secret_key, bucket, user_prefix = resolve_minio_config(
        endpoint=args.minio_endpoint,
        access_key=args.minio_access_key,
        secret_key=args.minio_secret_key,
        bucket=args.minio_bucket,
        user_prefix=args.minio_user_prefix,
    )
    minio_prefix = resolve_minio_prefix(args.minio_prefix, user_prefix, repo_id)
    client = create_minio_client(endpoint, access_key, secret_key)

    print(f"🏷️  repo_id: {repo_id}")
    success, failed = upload_folder_to_minio(
        local_folder=local_path,
        bucket_name=bucket,
        minio_client=client,
        minio_prefix=minio_prefix,
        dry_run=args.dry_run,
    )

    if args.dry_run:
        print(f"\n🔍 dry-run 完成，共 {success} 个文件")
        return 0

    print(f"\n🎉 上传完成: 成功 {success}，失败 {failed}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
