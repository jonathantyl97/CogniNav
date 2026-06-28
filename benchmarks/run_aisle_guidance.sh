#!/usr/bin/env bash
# Phase 5 gate: aisle guidance + dynamic perception on warehouse bag replay.
#
# Usage:
#   ./benchmarks/run_aisle_guidance.sh
#   ./benchmarks/run_aisle_guidance.sh --source r2b

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_docker "benchmarks/run_aisle_guidance.sh" "$@"

SOURCE="r2b"
SEQ="aisle_cw_run_1"
RATE="1.0"
PROBE_TIMEOUT="${AISLE_PROBE_TIMEOUT:-90}"
PHASE="${COGNINAV_BENCHMARK_PHASE:-5}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE="$2"; shift 2 ;;
    --seq) SEQ="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
  esac
done

WAREHOUSE_DIR="${WAREHOUSE_DIR:-$(cogninav_downloads_dir)/warehouse}"

case "$SOURCE" in
  torwic)
    PLAY_BAG="$WAREHOUSE_DIR/${SEQ}_ros2"
    LAUNCH="warehouse.launch.py"
    DATASET="warehouse_torwic"
    ;;
  r2b)
    PLAY_BAG="$WAREHOUSE_DIR/r2b_storage"
    LAUNCH="r2b_storage.launch.py"
    DATASET="warehouse_r2b"
    SEQ="r2b_storage"
    ;;
  *)
    echo "Unknown source: $SOURCE (use torwic or r2b)"
    exit 1
    ;;
esac

PROBE_TOPICS=(
  /cogninav/aisle_guidance
  /cogninav/dynamic_detections
  /cogninav/lane_markers
)

if [[ ! -d "$PLAY_BAG" ]]; then
  echo "Missing bag $PLAY_BAG — see README.md (Datasets)"
  exit 1
fi

set +u
cogninav_ros_setup
set -u

cd "$ROOT/ros2_ws"
colcon build --packages-select \
  cogninav_vslam cogninav_depth cogninav_lanes cogninav_bringup cogninav_viz
set +u
source install/setup.bash
set -u

export LD_LIBRARY_PATH="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"

LAUNCH_PID=""
cleanup() {
  [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

ros2 launch cogninav_bringup "$LAUNCH" \
  bag_path:="$PLAY_BAG" \
  rate:="$RATE" \
  bag_play_delay:=10.0 \
  bag_loop:=true \
  use_viz:=false \
  use_vslam:=true \
  use_depth:=true \
  use_lanes:=true &
LAUNCH_PID=$!

sleep 18
set +e
"$ROOT/benchmarks/wait_for_topics.sh" --timeout "$PROBE_TIMEOUT" "${PROBE_TOPICS[@]}"
PROBE_RC=$?
set -e

kill "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
LAUNCH_PID=""

GIT_SHA="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
if [[ "$PROBE_RC" -eq 0 ]]; then
  STATUS="ok"
  NOTE="Phase 5: aisle guidance + dynamic detections ($SOURCE): ${PROBE_TOPICS[*]}"
else
  STATUS="fail"
  NOTE="Phase 5: topic probe failed ($SOURCE)."
fi

"$ROOT/benchmarks/run_benchmark.sh" \
  --dataset "$DATASET" --seq "$SEQ" --phase "$PHASE" \
  --git-sha "$GIT_SHA" \
  --docker-image "$(cogninav_docker_image)" \
  --smoke-status "$STATUS" \
  --smoke-note "$NOTE"

if [[ "$PROBE_RC" -ne 0 ]]; then
  echo "ERROR: aisle guidance gate failed"
  exit 1
fi

echo "==> Phase 5 aisle guidance gate passed ($SOURCE / $SEQ)"
