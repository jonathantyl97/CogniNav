#!/usr/bin/env bash
# Download TUM-VI ground-truth for a sequence (optional, for ATE).
#
# Usage:
#   ./scripts/download_tumvi_gt.sh dataset-room1_512_16

set -euo pipefail

SEQ="${1:-dataset-room1_512_16}"
TUMVI_DIR="${TUMVI_DIR:-${HOME}/Downloads/tumvi}"
GT_DIR="$TUMVI_DIR/groundtruth"
mkdir -p "$GT_DIR"

OUT="$GT_DIR/${SEQ}.txt"
if [[ -f "$OUT" ]]; then
  echo "Ground truth ready: $OUT"
  exit 0
fi

BASE="${SEQ%_16}"
BASE="${BASE%_4}"

URLS=(
  "https://vision.in.tum.de/data/datasets/visual-inertial-dataset/eval/groundtruth/${SEQ}.txt"
  "https://vision.in.tum.de/data/datasets/visual-inertial-dataset/eval/groundtruth/${BASE}.txt"
  "https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset/eval/groundtruth/${SEQ}.txt"
)

for url in "${URLS[@]}"; do
  echo "  trying $url"
  if wget -q --timeout=30 "$url" -O "$OUT"; then
    echo "Downloaded $OUT"
    exit 0
  fi
  rm -f "$OUT"
done

echo "WARN: could not download GT for $SEQ (ATE will be skipped)"
