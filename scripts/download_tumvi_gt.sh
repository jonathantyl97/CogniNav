#!/usr/bin/env bash
# Download TUM-VI ground-truth for a sequence (optional, for ATE).
#
# Usage:
#   ./scripts/download_tumvi_gt.sh dataset-room1_512_16
#   ./scripts/download_tumvi_gt.sh room1

set -euo pipefail

SEQ="${1:-dataset-room1_512_16}"
TUMVI_DIR="${TUMVI_DIR:-${HOME}/Downloads/tumvi}"
GT_DIR="$TUMVI_DIR/groundtruth"
mkdir -p "$GT_DIR"

if [[ "$SEQ" =~ ^room[0-9]+$ ]]; then
  SEQ="dataset-${SEQ}_512_16"
fi

OUT="$GT_DIR/${SEQ}.txt"
if [[ -f "$OUT" ]]; then
  echo "Ground truth ready: $OUT"
  exit 0
fi

URLS=(
  "https://raw.githubusercontent.com/rpng/open_vins/master/ov_data/tum_vi/${SEQ}.txt"
  "https://vision.in.tum.de/tumvi/groundtruth/${SEQ}.txt"
  "https://cvg.cit.tum.de/tumvi/groundtruth/${SEQ}.txt"
)

for url in "${URLS[@]}"; do
  echo "  trying $url"
  if wget -q --timeout=60 "$url" -O "$OUT"; then
    if [[ -s "$OUT" ]]; then
      echo "Downloaded $OUT"
      exit 0
    fi
  fi
  rm -f "$OUT"
done

echo "WARN: could not download GT for $SEQ (ATE will be skipped)"
