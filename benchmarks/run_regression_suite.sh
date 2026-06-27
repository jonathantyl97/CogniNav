#!/usr/bin/env bash
# Warehouse regression — run after rig/calibration changes.
#
# Usage:
#   ./benchmarks/run_regression_suite.sh

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SEQ="${WAREHOUSE_SEQ:-aisle_cw_run_1}"

echo "==> CogniNav warehouse regression"
"$ROOT/benchmarks/run_warehouse_slam.sh" --seq "$SEQ"
echo "==> Regression complete. See benchmarks/results/"
