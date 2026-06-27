#!/usr/bin/env bash
# Run benchmark evaluation (smoke metadata or ATE/RPE when GT is available).
#
# Usage:
#   ./benchmarks/run_benchmark.sh --dataset warehouse --seq aisle_cw_run_1
#
# Writes: benchmarks/results/<timestamp>_warehouse_<seq>.json

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET="warehouse"
SEQ="aisle_cw_run_1"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dataset) DATASET="$2"; shift 2 ;;
    --seq) SEQ="$2"; shift 2 ;;
    --phase|--git-sha|--docker-image|--smoke-status|--smoke-note|--traj|--gt)
      EXTRA_ARGS+=("$1" "$2")
      shift 2
      ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT_DIR="$ROOT/benchmarks/results"
mkdir -p "$OUT_DIR"
OUT_FILE="$OUT_DIR/${STAMP}_${DATASET}_${SEQ}.json"

python3 "$ROOT/benchmarks/eval_ate.py" \
  --dataset "$DATASET" \
  --seq "$SEQ" \
  --output "$OUT_FILE" \
  "${EXTRA_ARGS[@]}"

echo "Wrote $OUT_FILE"
