#!/usr/bin/env bash
# Smoke test: ORB-SLAM3 + workspace on warehouse ROS 2 bags.
#
# Usage:
#   ./scripts/smoke_warehouse.sh --workspace-only
#   ./scripts/smoke_warehouse.sh --source torwic --seq aisle_cw_run_1
#   ./scripts/smoke_warehouse.sh --source r2b

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_docker "scripts/smoke_warehouse.sh" "$@"

SOURCE="${WAREHOUSE_SOURCE:-torwic}"
SEQ="${WAREHOUSE_SEQ:-aisle_cw_run_1}"
ORB_LIB="$ROOT/third_party/ORB_SLAM3/lib/libORB_SLAM3.so"
WORKSPACE_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE="$2"; shift 2 ;;
    --seq) SEQ="$2"; shift 2 ;;
    --workspace-only) WORKSPACE_ONLY=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

if [[ "$SOURCE" == "r2b" ]]; then
  SEQ="r2b_storage"
fi

echo "==> Warehouse smoke: $SOURCE / $SEQ"

if [[ ! -f "$ORB_LIB" ]]; then
  echo "Missing $ORB_LIB — run ./scripts/setup_deps.sh first"
  exit 1
fi

set +u
cogninav_ros_setup
set -u
DOCKER_IMAGE="$(cogninav_docker_image)"

cd "$ROOT/ros2_ws"
colcon build
set +u
source install/setup.bash
set -u

GIT_SHA="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
DATASET="warehouse_${SOURCE}"

record_result() {
  local status="$1"
  local note="$2"
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset "$DATASET" \
    --seq "$SEQ" \
    --phase 0 \
    --git-sha "$GIT_SHA" \
    --docker-image "$DOCKER_IMAGE" \
    --smoke-status "$status" \
    --smoke-note "$note"
}

if [[ "$WORKSPACE_ONLY" == true ]]; then
  record_result "workspace_ok" "colcon build + libORB_SLAM3.so verified; warehouse SLAM run skipped."
  echo "==> Workspace smoke passed."
  exit 0
fi

"$ROOT/scripts/download_warehouse.sh" --source "$SOURCE" --seq "$SEQ"
export COGNINAV_BENCHMARK_PHASE=0
"$ROOT/benchmarks/run_warehouse_slam.sh" --source "$SOURCE" --seq "$SEQ"

echo "==> Warehouse smoke passed."
