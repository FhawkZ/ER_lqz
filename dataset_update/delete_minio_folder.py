#!/usr/bin/env python3
"""删除 MinIO 中指定 LeRobot 数据集前缀下的所有对象。

用法（lerobot conda 环境）::

    conda activate lerobot
    pip install -r dataset_update/requirements.txt
    cp dataset_update/.env.example dataset_update/.env   # 填写 MinIO 凭据

    # 按本地数据集目录推导 MinIO 前缀
    python dataset_update/delete_minio_folder.py \\
      --dataset /home/franka/lqz/Data/franka_hand/joint-assemble-20260708 \\
      --yes

    # 或使用 repo_id
    python dataset_update/delete_minio_folder.py \\
      --repo-id franka_hand/joint-assemble-20260708 \\
      --yes
"""

from __future__ import annotations

import argparse

from minio.error import S3Error

from dataset_common import resolve_dataset
from minio_common import (
    add_minio_args,
    create_minio_client,
    resolve_minio_config,
    resolve_minio_prefix,
)


def delete_minio_folder(bucket_name: str, prefix: str, minio_client) -> int:
    normalized_prefix = prefix.strip("/")
    if normalized_prefix:
        normalized_prefix = f"{normalized_prefix}/"

    try:
        objects = minio_client.list_objects(
            bucket_name,
            prefix=normalized_prefix,
            recursive=True,
        )
        deleted = 0
        for obj in objects:
            minio_client.remove_object(bucket_name, obj.object_name)
            deleted += 1
            print(f"🗑️  {obj.object_name}")

        print(f"\n✅ 删除完成: {bucket_name}/{normalized_prefix}（共 {deleted} 个对象）")
        return deleted
    except S3Error as exc:
        print(f"❌ 删除失败: {exc}")
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="删除 MinIO 中的 LeRobot 数据集")
    add_minio_args(parser)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="跳过确认，直接删除",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只列出将要删除的对象，不实际删除",
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

    normalized_prefix = minio_prefix.strip("/")
    list_prefix = f"{normalized_prefix}/" if normalized_prefix else ""

    print(f"📦 本地数据集: {local_path}")
    print(f"🏷️  repo_id: {repo_id}")
    print(f"☁️  MinIO 前缀: {bucket}/{list_prefix}")

    if args.dry_run:
        objects = list(
            client.list_objects(bucket, prefix=list_prefix, recursive=True)
        )
        print(f"🔍 dry-run: 将删除 {len(objects)} 个对象")
        for obj in objects[:10]:
            print(f"  -> {obj.object_name}")
        if len(objects) > 10:
            print(f"  ... 还有 {len(objects) - 10} 个对象")
        return 0

    if not args.yes:
        answer = input("确认删除以上 MinIO 前缀下的所有对象? [y/N] ").strip().lower()
        if answer not in {"y", "yes"}:
            print("已取消")
            return 0

    delete_minio_folder(bucket, minio_prefix, client)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
