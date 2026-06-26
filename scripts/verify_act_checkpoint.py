#!/usr/bin/env python3
"""加载本地 ACT checkpoint，做一次 dummy 前向，验证权重与代码兼容。"""

from __future__ import annotations

import argparse
import sys
import types
from pathlib import Path


def _setup_lerobot_import_path() -> Path:
    er_root = Path(__file__).resolve().parents[1]
    src = er_root / "lerobot" / "src"
    if str(src) not in sys.path:
        sys.path.insert(0, str(src))
    return src


def _stub_policies_packages(src: Path) -> None:
    """避免 policies/__init__.py 拉取 groot（当前环境 dataclass 会报错）。"""
    policies_dir = src / "lerobot" / "policies"
    act_dir = policies_dir / "act"

    if "lerobot.policies" not in sys.modules:
        pkg = types.ModuleType("lerobot.policies")
        pkg.__path__ = [str(policies_dir)]
        sys.modules["lerobot.policies"] = pkg

    if "lerobot.policies.act" not in sys.modules:
        act_pkg = types.ModuleType("lerobot.policies.act")
        act_pkg.__path__ = [str(act_dir)]
        sys.modules["lerobot.policies.act"] = act_pkg


def main() -> int:
    parser = argparse.ArgumentParser(description="验证 ACT pretrained_model 目录可加载并推理。")
    parser.add_argument(
        "--policy-path",
        type=Path,
        default=Path("/media/disk/isaac_lqz/models/act_redcube_merged_quat/checkpoints/100000/pretrained_model"),
        help="含 config.json 与 model.safetensors 的目录",
    )
    parser.add_argument("--device", default="cpu", choices=("cpu", "cuda", "mps"))
    args = parser.parse_args()

    policy_dir = args.policy_path.resolve()
    if not (policy_dir / "config.json").is_file():
        print(f"[FAIL] 缺少 config.json: {policy_dir}")
        return 1
    if not (policy_dir / "model.safetensors").is_file():
        print(f"[FAIL] 缺少 model.safetensors: {policy_dir}")
        return 1

    src = _setup_lerobot_import_path()
    _stub_policies_packages(src)

    import torch
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy

    print(f"[load] {policy_dir}")
    policy = ACTPolicy.from_pretrained(policy_dir)
    policy.config.device = args.device
    policy.to(args.device)
    policy.eval()

    cfg = policy.config
    print(f"[config] chunk_size={cfg.chunk_size} action_dim={cfg.action_feature.shape[0]}")
    print(f"[config] images={list(cfg.image_features)}")
    print(f"[config] separate_backbones_per_image={cfg.separate_backbones_per_image}")

    batch: dict[str, torch.Tensor] = {}
    if cfg.robot_state_feature:
        batch["observation.state"] = torch.zeros(1, cfg.robot_state_feature.shape[0], device=args.device)
    for key in cfg.image_features:
        batch[key] = torch.zeros(1, 3, 480, 640, device=args.device)

    with torch.no_grad():
        actions = policy.predict_action_chunk(batch)

    print(f"[ok] predict_action_chunk -> shape={tuple(actions.shape)} dtype={actions.dtype}")
    print(f"[ok] action sample (step0, dim0:5): {actions[0, 0, :5].tolist()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
