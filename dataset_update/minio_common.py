"""MinIO 上传/删除脚本的客户端与参数。"""

from __future__ import annotations

import argparse
import os

from minio import Minio
from urllib3 import PoolManager, Timeout

from dataset_common import add_dataset_args, load_env_file

load_env_file()


def _require_env(name: str) -> str:
    value = os.environ.get(name, "").strip()
    if not value:
        raise ValueError(
            f"缺少环境变量 {name}。请设置该变量，或在 dataset_update/.env 中配置（参考 .env.example）。"
        )
    return value


def add_minio_args(parser: argparse.ArgumentParser) -> None:
    add_dataset_args(parser)
    parser.add_argument(
        "--minio-prefix",
        default=None,
        help="MinIO 对象前缀；默认 {MINIO_USER_PREFIX}/{repo_id}",
    )
    parser.add_argument(
        "--minio-endpoint",
        default=os.environ.get("MINIO_ENDPOINT"),
        help="MinIO 端点（默认读取 MINIO_ENDPOINT）",
    )
    parser.add_argument(
        "--minio-access-key",
        default=os.environ.get("MINIO_ACCESS_KEY"),
        help="MinIO Access Key（默认读取 MINIO_ACCESS_KEY）",
    )
    parser.add_argument(
        "--minio-secret-key",
        default=os.environ.get("MINIO_SECRET_KEY"),
        help="MinIO Secret Key（默认读取 MINIO_SECRET_KEY）",
    )
    parser.add_argument(
        "--minio-bucket",
        default=os.environ.get("MINIO_BUCKET"),
        help="MinIO Bucket（默认读取 MINIO_BUCKET）",
    )
    parser.add_argument(
        "--minio-user-prefix",
        default=os.environ.get("MINIO_USER_PREFIX"),
        help="MinIO 用户名前缀（默认读取 MINIO_USER_PREFIX）",
    )


def resolve_minio_prefix(minio_prefix: str | None, user_prefix: str, repo_id: str) -> str:
    if minio_prefix:
        return minio_prefix.strip("/")
    return f"{user_prefix.strip('/')}/{repo_id.strip('/')}"


def resolve_minio_config(
    *,
    endpoint: str | None,
    access_key: str | None,
    secret_key: str | None,
    bucket: str | None,
    user_prefix: str | None,
) -> tuple[str, str, str, str, str]:
    return (
        endpoint or _require_env("MINIO_ENDPOINT"),
        access_key or _require_env("MINIO_ACCESS_KEY"),
        secret_key or _require_env("MINIO_SECRET_KEY"),
        bucket or _require_env("MINIO_BUCKET"),
        user_prefix or _require_env("MINIO_USER_PREFIX"),
    )


def create_minio_client(
    endpoint: str,
    access_key: str,
    secret_key: str,
    *,
    connect_timeout_s: int = 600,
    read_timeout_s: int = 7200,
) -> Minio:
    timeout = Timeout(connect=connect_timeout_s, read=read_timeout_s)
    http_client = PoolManager(timeout=timeout, retries=3)
    return Minio(
        endpoint=endpoint,
        access_key=access_key,
        secret_key=secret_key,
        secure=False,
        http_client=http_client,
    )
