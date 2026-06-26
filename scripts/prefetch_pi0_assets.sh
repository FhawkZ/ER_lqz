#!/usr/bin/env bash
# 预下载 PI0 推理依赖到 HF 缓存（tokenizer + pi0_base），避免 server 侧 SSL/断网失败
#
# 用法:
#   bash scripts/prefetch_pi0_assets.sh

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_lerobot.sh
source "${SCRIPT_DIR}/env_lerobot.sh"

echo ">>> HF_HOME=${HF_HOME}"
echo ">>> HF_ENDPOINT=${HF_ENDPOINT:-}"

python << 'PY'
from huggingface_hub import snapshot_download

assets = [
    "google/paligemma-3b-pt-224",
    "lerobot/pi0_base",
]
for repo in assets:
    print(f"--- snapshot_download {repo}")
    path = snapshot_download(repo)
    print(f"    -> {path}")
print("OK prefetch complete")
PY

echo ">>> 可用以下命令验证本地 tokenizer:"
echo "python -c \"from transformers import AutoTokenizer; AutoTokenizer.from_pretrained('google/paligemma-3b-pt-224', local_files_only=True); print('tokenizer OK')\""
