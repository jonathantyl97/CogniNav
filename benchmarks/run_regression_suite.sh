#!/usr/bin/env bash
# Full warehouse regression — run after calibration or rig changes.
#
# Usage:
#   ./benchmarks/run_regression_suite.sh
#   ./benchmarks/run_regression_suite.sh --source r2b

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_docker "benchmarks/run_regression_suite.sh" "$@"

SOURCE="r2b"
SEQ="aisle_cw_run_1"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE="$2"; shift 2 ;;
    --seq) SEQ="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

echo "==> CogniNav regression ($SOURCE)"
"$ROOT/benchmarks/run_warehouse_slam.sh" --source "$SOURCE" --seq "$SEQ"
"$ROOT/benchmarks/run_warehouse_perception.sh" --source "$SOURCE" --seq "$SEQ"
echo "==> Regression complete. See benchmarks/results/"
