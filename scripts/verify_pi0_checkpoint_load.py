#!/usr/bin/env python3
"""验证 Pi0 LoRA checkpoint 能否像 policy_server 一样正确加载，并在数据集上跑通推理。

检查项：
  1. 旧路径（直接 from_pretrained(checkpoint)）对 LoRA 会失败
  2. server 同款 load_policy_from_checkpoint：base + adapter
  3. 在 redcube_merged_quat 上 preprocessor → predict → postprocessor

示例：
  bash scripts/verify_pi0_checkpoint_load.sh
  CHECKPOINT=.../020000/pretrained_model DEVICE=cuda bash scripts/verify_pi0_checkpoint_load.sh
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
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
    endpoint = hf_endpoint or os.environ.get("HF_ENDPOINT") or "https://hf-mirror.com"
    home = hf_home or Path(os.environ.get("HF_HOME") or _default_hf_home())
    home.mkdir(parents=True, exist_ok=True)
    os.environ["HF_ENDPOINT"] = endpoint
    os.environ["HF_HOME"] = str(home)
    os.environ.setdefault("HUGGINGFACE_HUB_CACHE", str(home / "hub"))
    return endpoint, str(home)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=ROOT / "models/pi0_redcube_merged_quat_lora_vlm_bs8/checkpoints/080000/pretrained_model",
    )
    parser.add_argument(
        "--dataset-root",
        type=Path,
        default=ROOT / "data/redcube_merged_quat",
    )
    parser.add_argument("--dataset-repo-id", type=str, default="redcube_merged_quat")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--sample-index", type=int, default=0)
    parser.add_argument("--hf-endpoint", default=None)
    parser.add_argument("--hf-home", type=Path, default=None)
    parser.add_argument(
        "--skip-forward",
        action="store_true",
        help="只验证加载，不跑数据集前向。",
    )
    return parser.parse_args()


def _adapter_weight_norm(policy) -> float:
    from peft import PeftModel

    if not isinstance(policy, PeftModel):
        return 0.0
    total = 0.0
    for name, param in policy.named_parameters():
        if "lora" in name.lower():
            total += float(param.detach().float().norm().item())
    return total


def _check_broken_direct_load(policy_class, checkpoint: Path) -> None:
    """复现旧 server 行为：对 LoRA 目录直接 from_pretrained 会放弃加载权重。"""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        policy = policy_class.from_pretrained(checkpoint)
    out = buf.getvalue()
    if "Returning model without loading pretrained weights" not in out:
        raise RuntimeError(
            "预期旧加载路径会跳过权重，但未看到警告；请检查 modeling_pi0.from_pretrained 行为是否变化。"
        )
    trainable = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print("OK   [旧路径] 直接 from_pretrained(checkpoint) 会跳过权重（与修复前 server 一致）")
    print(f"     日志片段: Returning model without loading pretrained weights")
    print(f"     trainable params (应接近全量): {trainable:,}")
    del policy


def main() -> int:
    args = parse_args()
    hf_endpoint, hf_home = setup_hf_env(args.hf_home, args.hf_endpoint)

    sys.path.insert(0, str(ROOT / "lerobot" / "src"))

    import torch
    from peft import PeftModel

    from lerobot.async_inference.policy_loader import load_policy_from_checkpoint
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    ckpt = args.checkpoint.resolve()
    if not (ckpt / "config.json").is_file():
        print(f"FAIL missing config.json: {ckpt}")
        return 1
    if not (ckpt / "adapter_model.safetensors").is_file():
        print(f"FAIL missing adapter_model.safetensors: {ckpt}")
        return 1

    device = args.device
    if device.startswith("cuda") and not torch.cuda.is_available():
        print("WARN cuda 不可用，回退到 cpu")
        device = "cpu"

    cfg = PreTrainedConfig.from_pretrained(ckpt)
    policy_class = get_policy_class(cfg.type)

    print("=== Pi0 LoRA checkpoint 验证 ===")
    print(f"checkpoint   : {ckpt}")
    print(f"device       : {device}")
    print(f"HF_ENDPOINT  : {hf_endpoint}")
    print(f"HF_HOME      : {hf_home}")
    print(f"use_peft     : {cfg.use_peft}")
    with open(ckpt / "adapter_config.json") as f:
        print(f"base model   : {json.load(f).get('base_model_name_or_path')}")

    # Step 1: demonstrate broken path
    try:
        _check_broken_direct_load(policy_class, ckpt)
    except Exception as exc:
        print(f"FAIL [旧路径检查] {exc}")
        return 1

    # Step 2: server-style load
    try:
        policy = load_policy_from_checkpoint(policy_class, ckpt)
        policy.to(device)
        policy.eval()
    except Exception as exc:
        print(f"FAIL [server 加载] {exc}")
        return 1

    if not isinstance(policy, PeftModel):
        print(f"FAIL [server 加载] 期望 PeftModel，实际 {type(policy).__name__}")
        return 1

    lora_norm = _adapter_weight_norm(policy)
    if lora_norm <= 0:
        print("FAIL [server 加载] LoRA 权重范数为 0，adapter 可能未加载")
        return 1

    total_params = sum(p.numel() for p in policy.parameters())
    print("OK   [server 加载] PeftModel 加载成功")
    print(f"     params={total_params:,}  lora_weight_norm={lora_norm:.4f}")

    if not args.dataset_root.is_dir():
        print(f"WARN dataset 不存在，跳过前向: {args.dataset_root}")
        print("\n=== Result: 权重加载验证通过（未跑数据集）===")
        return 0

    # Step 3: dataset forward (same processors as server)
    try:
        ds = LeRobotDataset(
            args.dataset_repo_id,
            root=str(args.dataset_root),
            video_backend="pyav",
        )
    except Exception as exc:
        print(f"FAIL [dataset] {exc}")
        return 1
    print(f"dataset      : {args.dataset_root} ({ds.meta.total_frames} frames)")

    try:
        preprocessor, postprocessor = make_pre_post_processors(
            policy_cfg=cfg,
            pretrained_path=ckpt,
            postprocessor_overrides={"device_processor": {"device": device}},
            preprocessor_overrides={"device_processor": {"device": device}},
        )
    except Exception as exc:
        print(f"FAIL [processors] {exc}")
        return 1
    print("OK   [processors] preprocessor/postprocessor 已加载")

    if args.skip_forward:
        print("\n=== Result: 权重加载验证通过（--skip-forward）===")
        return 0

    try:
        sample = ds[args.sample_index]
        if "task" not in sample:
            if len(ds.meta.tasks):
                task = str(ds.meta.tasks.index[0])
            else:
                task = "pick the red cube and drop it in box"
            sample = {**sample, "task": task}

        batch = preprocessor(sample)
        with torch.no_grad():
            if hasattr(policy, "predict_action_chunk"):
                actions = policy.predict_action_chunk(batch)
                if actions.ndim == 3:
                    actions = actions[0]
            else:
                actions = policy.select_action(batch).unsqueeze(0)
            actions = postprocessor(actions)
        if actions.ndim == 3:
            actions = actions[0]
    except Exception as exc:
        print(f"FAIL [forward] {exc}")
        import traceback

        traceback.print_exc()
        return 1

    actions_np = actions.detach().float().cpu()
    print(f"OK   [forward] action chunk shape={tuple(actions_np.shape)} dtype={actions_np.dtype}")

    # Step 4: sanity vs dataset stats
    stats_path = args.dataset_root / "meta/stats.json"
    if stats_path.is_file():
        action_stats = json.load(open(stats_path))["action"]
        hand_mean = action_stats["mean"][7:13]
        hand_std = action_stats["std"][7:13]
        hand_min = action_stats["min"][7:13]
        hand_max = action_stats["max"][7:13]

        step0 = actions_np[0, 7:13].tolist()
        hand_delta = (actions_np[1:, 7:13] - actions_np[:-1, 7:13]).abs().mean().item()
        oob = ((actions_np[:, 7:13] < 0) | (actions_np[:, 7:13] > 255)).any().item()

        print("     step0 hand:", [round(x, 1) for x in step0])
        print(f"     chunk hand |Δ| mean={hand_delta:.2f} (数据集逐步约 0.87)")
        print(f"     hand 数据范围: min={min(hand_min):.0f} max={max(hand_max):.0f}")
        print(f"     预测超 [0,255]: {bool(oob)}")

        if hand_delta > 20:
            print("WARN [forward] chunk 内手指变化仍偏大，但权重已正确加载；推理侧可能还需调参")
        if oob:
            print("WARN [forward] 反归一化后手指有超 0–255，执行端建议 clip")

    print("\n=== Result: checkpoint 加载 + 数据集前向验证通过 ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
