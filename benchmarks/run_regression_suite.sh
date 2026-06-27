#!/usr/bin/env bash
# Open-dataset regression suite — run after rig/calibration changes (Phase 4 gate).
#
# Usage:
#   ./benchmarks/run_regression_suite.sh
#   ./benchmarks/run_regression_suite.sh --quick   # EuRoC only

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
QUICK=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --quick) QUICK=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

echo "==> CogniNav open-dataset regression (Phase 4)"

"$ROOT/benchmarks/run_euroc_slam.sh" --seq MH_01_easy

if [[ "$QUICK" == false ]]; then
  TUMVI_DIR="$(cogninav_downloads_dir)/tumvi"
  if [[ -f "$TUMVI_DIR/dataset-room1_512_16.bag" ]] \
    || [[ -d "$TUMVI_DIR/dataset-room1_512_16_ros2" ]] \
    || [[ -d "$TUMVI_DIR/dataset-room1_512_16_ros2_jazzy" ]]; then
    "$ROOT/benchmarks/run_tumvi_slam.sh" --seq dataset-room1_512_16
  else
    echo "WARN: TUM-VI bag missing — skip (./scripts/download_tumvi.sh dataset-room1_512_16)"
  fi
fi

echo "==> Regression suite complete. See benchmarks/results/"
