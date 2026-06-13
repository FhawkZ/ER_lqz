#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}" \
python3 -m fr3_dexterous_platform.cli collect \
  --backend mock \
  --formats "${FORMATS:-jsonl,lerobot,droid,openpi}" \
  --output-dir "${OUTPUT_DIR:-/tmp/fr3dex_mock}" \
  --frames "${FRAMES:-30}" \
  --fps "${FPS:-30}" \
  --task "${TASK:-pick the red cube}"
