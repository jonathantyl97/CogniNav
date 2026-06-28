#!/usr/bin/env bash
# Phase 6 gate: dynamic-mask SLAM on open-dataset bag replay.
#
# Usage:
#   ./benchmarks/run_dynamic_slam.sh                    # r2b (default)
#   ./benchmarks/run_dynamic_slam.sh --source kitti --seq 00

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_docker "benchmarks/run_dynamic_slam.sh" "$@"

SOURCE="r2b"
SEQ="00"
RATE="1.0"
PROBE_TIMEOUT="${DYNAMIC_SLAM_PROBE_TIMEOUT:-120}"
PHASE="${COGNINAV_BENCHMARK_PHASE:-6}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE="$2"; shift 2 ;;
    --seq) SEQ="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
  esac
done

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

case "$SOURCE" in
  r2b)
    WAREHOUSE_DIR="$(cogninav_downloads_dir)/warehouse"
    BAG_PATH="$WAREHOUSE_DIR/r2b_storage"
    LAUNCH="r2b_storage.launch.py"
    DATASET="warehouse_r2b"
    SEQ="r2b_storage"
    TRAJ="/tmp/cogninav_r2b_trajectory.txt"
    ;;
  kitti)
    SEQ="$(printf '%02d' "$((10#$SEQ))")"
    KITTI_DIR="$(cogninav_downloads_dir)/kitti"
    BAG_PATH="$KITTI_DIR/${SEQ}_ros2"
    LAUNCH="kitti.launch.py"
    DATASET="kitti_odometry"
    TRAJ="/tmp/cogninav_kitti_trajectory.txt"
    ;;
  *)
    echo "Unknown source: $SOURCE (use r2b or kitti)"
    exit 1
    ;;
esac

if [[ ! -d "$BAG_PATH" ]]; then
  echo "Missing bag $BAG_PATH — see README.md (Datasets)"
  exit 1
fi

PROBE_TOPICS=(
  /cogninav/odom
  /cogninav/map_points
  /cogninav/dynamic_mask
  /cogninav/aisle_guidance
  /cogninav/slam_mask_stats
)

LAUNCH_PID=""
cleanup() {
  [[ -n "$LAUNCH_PID" ]] && kill "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

rm -f "$TRAJ"
ros2 launch cogninav_bringup "$LAUNCH" \
  bag_path:="$BAG_PATH" \
  rate:="$RATE" \
  bag_play_delay:=12.0 \
  bag_loop:=true \
  use_viz:=false \
  use_vslam:=true \
  use_depth:=true \
  use_lanes:=true &
LAUNCH_PID=$!

sleep 20
set +e
"$ROOT/benchmarks/wait_for_topics.sh" --timeout "$PROBE_TIMEOUT" "${PROBE_TOPICS[@]}"
PROBE_RC=$?
set -e

kill -INT "$LAUNCH_PID" 2>/dev/null || true
sleep 3
kill "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
LAUNCH_PID=""

TRAJ_OK=false
if [[ -f "$TRAJ" ]] && [[ "$(wc -l <"$TRAJ")" -gt 5 ]]; then
  TRAJ_OK=true
fi

GIT_SHA="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
if [[ "$PROBE_RC" -eq 0 ]]; then
  STATUS="ok"
  NOTE="Phase 6: dynamic-mask SLAM ($SOURCE) topics OK (traj_saved=$TRAJ_OK)."
else
  STATUS="fail"
  NOTE="Phase 6: dynamic SLAM gate failed ($SOURCE probe=$PROBE_RC traj=$TRAJ_OK)."
fi

"$ROOT/benchmarks/run_benchmark.sh" \
  --dataset "$DATASET" --seq "$SEQ" --phase "$PHASE" \
  --git-sha "$GIT_SHA" \
  --docker-image "$(cogninav_docker_image)" \
  --smoke-status "$STATUS" \
  --smoke-note "$NOTE"

if [[ "$STATUS" != "ok" ]]; then
  echo "ERROR: dynamic SLAM gate failed"
  exit 1
fi

echo "==> Phase 6 dynamic-mask SLAM gate passed ($SOURCE / $SEQ)"
