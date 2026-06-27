#!/usr/bin/env bash
# Phase 3: ROS 2 Humble parity — EuRoC SLAM smoke in cogninav_humble container.
#
# Usage (host or inside Humble container):
#   ./benchmarks/run_humble_smoke.sh
#   ./benchmarks/run_humble_smoke.sh --seq MH_01_easy --workspace-only

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_humble "benchmarks/run_humble_smoke.sh" "$@"

SEQ="MH_01_easy"
WORKSPACE_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --workspace-only) WORKSPACE_ONLY=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

ORB_LIB="$ROOT/third_party/ORB_SLAM3/lib/libORB_SLAM3.so"

echo "==> Phase 3 Humble smoke (EuRoC $SEQ)"

if [[ ! -f "$ORB_LIB" ]]; then
  echo "==> Building ORB-SLAM3 for Humble (first time)..."
  "$ROOT/scripts/setup_deps.sh"
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
    --dataset euroc --seq "$SEQ" --phase 3 \
    --git-sha "$GIT_SHA" --docker-image "$DOCKER_IMAGE" \
    --smoke-status "workspace_ok" \
    --smoke-note "Humble: colcon build + libORB_SLAM3.so verified."
  echo "==> Phase 3 Humble workspace smoke passed."
  exit 0
fi

export EUROC_DIR="${EUROC_DIR:-$(cogninav_downloads_dir)/euroc}"
export COGNINAV_IN_HUMBLE=1
export COGNINAV_IN_DOCKER=1
export COGNINAV_BENCHMARK_PHASE=3

"$ROOT/benchmarks/run_euroc_slam.sh" --seq "$SEQ"

echo "==> Phase 3 Humble EuRoC smoke complete."
