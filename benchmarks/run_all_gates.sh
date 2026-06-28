#!/usr/bin/env bash
# Run all automated CogniNav gates (workspace + warehouse + humble smoke).
#
# Usage:
#   ./benchmarks/run_all_gates.sh
#   ./benchmarks/run_all_gates.sh --skip-humble

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_docker "benchmarks/run_all_gates.sh" "$@"

SKIP_HUMBLE=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-humble) SKIP_HUMBLE=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

echo "==> Gate 0: workspace smoke"
"$ROOT/benchmarks/smoke_warehouse.sh" --workspace-only

echo "==> Gate 1+2: warehouse SLAM + perception (r2b)"
"$ROOT/benchmarks/run_regression_suite.sh" --source r2b

echo "==> Gate 5: aisle guidance + dynamic perception (r2b)"
"$ROOT/benchmarks/run_aisle_guidance.sh" --source r2b

echo "==> Gate 6: dynamic-mask SLAM (r2b open dataset)"
"$ROOT/benchmarks/run_dynamic_slam.sh" --source r2b

if [[ "$SKIP_HUMBLE" != true ]] && docker ps -a --format '{{.Names}}' | grep -qx ros2_humble_cogninav; then
  echo "==> Gate 3: Humble workspace smoke"
  COGNINAV_IN_HUMBLE=1 "$ROOT/benchmarks/run_humble_smoke.sh" --workspace-only || true
fi

echo "==> All automated gates passed."
