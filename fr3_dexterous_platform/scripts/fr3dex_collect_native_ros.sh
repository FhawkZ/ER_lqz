#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="${ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
python3 -m fr3_dexterous_platform.cli collect \
  --backend native-ros \
  --action-source native-mocap \
  --formats "${FORMATS:-jsonl,lerobot,hdf5,droid,openpi}" \
  --output-dir "${OUTPUT_DIR:-/data/fr3dex/session_native_001}" \
  --frames "${FRAMES:-900}" \
  --fps "${FPS:-30}" \
  --task "${TASK:-pick the red cube and drop it in box}" \
  --drop-stale-frames
