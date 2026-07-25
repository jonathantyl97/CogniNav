#!/usr/bin/env bash
# LingBot-Map side-by-side demo: input camera feed + live point-cloud map.
# Uses the local drive_frames.mp4 in the repo root by default.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source .venv/bin/activate

VIDEO="${1:-drive_frames.mp4}"
OUTPUT="${2:-assets/slam_sidebyside.mp4}"

python -m cogninav.slam_sidebyside \
  --video "$VIDEO" \
  --output "$OUTPUT" \
  --fps 5

echo "Side-by-side SLAM video: $OUTPUT"
