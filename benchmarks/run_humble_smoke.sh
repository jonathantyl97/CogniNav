#!/usr/bin/env bash
# ROS 2 Humble parity — warehouse SLAM smoke in cogninav_humble container.
#
# Usage:
#   ./benchmarks/run_humble_smoke.sh --workspace-only
#   ./benchmarks/run_humble_smoke.sh --seq aisle_cw_run_1

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_humble "benchmarks/run_humble_smoke.sh" "$@"

SEQ="aisle_cw_run_1"
WORKSPACE_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --workspace-only) WORKSPACE_ONLY=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

ORB_LIB="$ROOT/third_party/ORB_SLAM3/lib/libORB_SLAM3.so"

echo "==> Humble smoke (warehouse $SEQ)"

if [[ ! -f "$ORB_LIB" ]]; then
  echo "==> Building ORB-SLAM3 for Humble (first time)..."
  "$ROOT/docker/setup_deps.sh"
fi

set +u
cogninav_ros_setup
set -u

cd "$ROOT/ros2_ws"
colcon build
set +u
source install/setup.bash
set -u

GIT_SHA="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
DOCKER_IMAGE="$(cogninav_docker_image)"

if [[ "$WORKSPACE_ONLY" == true ]]; then
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset warehouse --seq "$SEQ" --phase 3 \
    --git-sha "$GIT_SHA" --docker-image "$DOCKER_IMAGE" \
    --smoke-status "workspace_ok" \
    --smoke-note "Humble: colcon build + libORB_SLAM3.so verified."
  echo "==> Humble workspace smoke passed."
  exit 0
fi

export WAREHOUSE_DIR="${WAREHOUSE_DIR:-$(cogninav_downloads_dir)/warehouse}"
export COGNINAV_IN_HUMBLE=1
export COGNINAV_IN_DOCKER=1
export COGNINAV_BENCHMARK_PHASE=3

PLAY_BAG="$WAREHOUSE_DIR/${SEQ}_ros2"
if [[ ! -d "$PLAY_BAG" ]]; then
  echo "Missing bag $PLAY_BAG — see README.md (Datasets)"
  exit 1
fi

"$ROOT/benchmarks/run_warehouse_slam.sh" --source torwic --seq "$SEQ"

echo "==> Humble warehouse smoke complete."
