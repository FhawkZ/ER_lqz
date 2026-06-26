#!/usr/bin/env python3
"""Smoke-test loading a Pi0 LoRA checkpoint."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _default_hf_home() -> Path:
    franka_hf = Path("/home/franka/lqz/hf")
    if franka_hf.is_dir() and os.access(franka_hf, os.W_OK):
        return franka_hf
    return ROOT / ".cache" / "hf"


def setup_hf_env(hf_home: Path | None = None, hf_endpoint: str | None = None) -> tuple[str, str]:
    """Configure HF cache/mirror before huggingface_hub is imported."""
    endpoint = hf_endpoint or os.environ.get("HF_ENDPOINT") or "https://hf-mirror.com"
    home = hf_home or Path(os.environ.get("HF_HOME") or _default_hf_home())
    home.mkdir(parents=True, exist_ok=True)

    os.environ["HF_ENDPOINT"] = endpoint
    os.environ["HF_HOME"] = str(home)
    # 部分旧版 huggingface_hub / transformers 仍读此变量
    os.environ.setdefault("HUGGINGFACE_HUB_CACHE", str(home / "hub"))
    return endpoint, str(home)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=ROOT / "models/pi0_redcube_merged_quat_lora_vlm_bs8/checkpoints/080000/pretrained_model",
        help="Path to pretrained_model directory (LoRA adapter + config).",
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=ROOT / "data/redcube_merged_quat",
        help="Dataset root for stats / one sample forward pass.",
    )
    parser.add_argument(
        "--device",
        default="cuda:1",
        help="Device for loading and inference smoke test (default: cuda:1).",
    )
    parser.add_argument(
        "--dtype",
        default="bfloat16",
        choices=("bfloat16", "float32"),
        help="Policy precision (default: bfloat16).",
    )
    parser.add_argument(
        "--hf-endpoint",
        default=None,
        help="HF mirror endpoint (default: https://hf-mirror.com).",
    )
    parser.add_argument(
        "--hf-home",
        type=Path,
        default=None,
        help="HF cache root (default: ER_LQZ_ROOT/.cache/hf).",
    )
    parser.add_argument(
        "--skip-forward",
        action="store_true",
        help="Only load weights, skip one forward pass.",
    )
    return parser.parse_args()


def _print_gpu_status(device: str) -> None:
    import torch

    if not device.startswith("cuda"):
        return
    if not torch.cuda.is_available():
        print("WARN cuda requested but not available")
        return
    idx = int(device.split(":")[1]) if ":" in device else 0
    if idx >= torch.cuda.device_count():
        print(f"WARN cuda:{idx} not found (count={torch.cuda.device_count()})")
        return
    free, total = torch.cuda.mem_get_info(idx)
    print(f">>> GPU {idx} mem  : {free / 2**30:.2f} GiB free / {total / 2**30:.2f} GiB total")


def main() -> int:
    args = parse_args()
    hf_endpoint, hf_home = setup_hf_env(args.hf_home, args.hf_endpoint)

    import torch
    from peft import PeftModel

    sys.path.insert(0, str(ROOT / "lerobot" / "src"))
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.factory import make_policy, make_pre_post_processors

    ckpt = args.checkpoint.resolve()
    if not (ckpt / "config.json").exists():
        print(f"FAIL missing config.json: {ckpt}")
        return 1

    device = args.device
    if device.startswith("cuda") and not torch.cuda.is_available():
        print("WARN falling back to cpu")
        device = "cpu"

    print(f">>> checkpoint  : {ckpt}")
    print(f">>> device      : {device}")
    print(f">>> dtype       : {args.dtype}")
    print(f">>> HF_ENDPOINT : {hf_endpoint}")
    print(f">>> HF_HOME     : {hf_home}")
    _print_gpu_status(device)

    cfg = PreTrainedConfig.from_pretrained(ckpt)
    cfg.pretrained_path = ckpt
    cfg.device = device
    cfg.dtype = args.dtype
    print(f">>> policy type : {cfg.type}")
    print(f">>> use_peft    : {cfg.use_peft}")
    adapter_cfg = ckpt / "adapter_config.json"
    if adapter_cfg.exists():
        import json

        with open(adapter_cfg) as f:
            print(f">>> base model  : {json.load(f).get('base_model_name_or_path')}")
    else:
        print(">>> base model  : (adapter_config.json not found)")

    ds_meta = None
    ds = None
    if args.dataset_root.exists():
        ds = LeRobotDataset("redcube_merged_quat", root=str(args.dataset_root))
        ds_meta = ds.meta
        print(f">>> dataset     : {args.dataset_root} ({ds_meta.total_frames} frames)")
    else:
        print(f">>> dataset     : skip (not found: {args.dataset_root})")

    try:
        policy = make_policy(cfg=cfg, ds_meta=ds_meta)
    except Exception as exc:
        print(f"FAIL make_policy: {exc}")
        return 1

    if not isinstance(policy, PeftModel):
        print(f"WARN expected PeftModel, got {type(policy).__name__}")

    trainable = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    total = sum(p.numel() for p in policy.parameters())
    print(f"OK   policy loaded: {type(policy).__name__}")
    print(f"     params total={total:,} trainable={trainable:,}")

    if ds_meta is not None:
        try:
            preprocessor, postprocessor = make_pre_post_processors(
                policy_cfg=cfg,
                pretrained_path=ckpt,
                dataset_stats=ds_meta.stats,
            )
            print("OK   preprocessor/postprocessor loaded")
        except Exception as exc:
            print(f"FAIL processors: {exc}")
            return 1

        if not args.skip_forward:
            try:
                policy.eval()
                sample = ds[0]
                batch = preprocessor(sample)
                with torch.no_grad():
                    action = policy.select_action(batch)
                action = postprocessor(action)
                print(f"OK   forward pass: action shape={tuple(action.shape)} dtype={action.dtype}")
            except Exception as exc:
                print(f"FAIL forward pass: {exc}")
                return 1

    print("\n=== Result: checkpoint loads successfully ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
