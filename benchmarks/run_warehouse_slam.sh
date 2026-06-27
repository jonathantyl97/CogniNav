#!/usr/bin/env bash
# Run CogniNav SLAM on a warehouse ROS 2 bag.
#
# Usage:
#   ./benchmarks/run_warehouse_slam.sh
#   ./benchmarks/run_warehouse_slam.sh --source torwic --seq aisle_ccw_run_1
#   ./benchmarks/run_warehouse_slam.sh --source r2b

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_docker "benchmarks/run_warehouse_slam.sh" "$@"

SOURCE="torwic"
SEQ="aisle_cw_run_1"
RATE="1.0"
TIMEOUT_SEC="${SLAM_TIMEOUT_SEC:-600}"
PHASE="${COGNINAV_BENCHMARK_PHASE:-4}"

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
    TRAJ="/tmp/cogninav_warehouse_trajectory.txt"
    LAUNCH="warehouse.launch.py"
    DATASET="warehouse_torwic"
    ;;
  r2b)
    PLAY_BAG="$WAREHOUSE_DIR/r2b_storage"
    TRAJ="/tmp/cogninav_r2b_trajectory.txt"
    LAUNCH="r2b_storage.launch.py"
    DATASET="warehouse_r2b"
    SEQ="r2b_storage"
    ;;
  *)
    echo "Unknown source: $SOURCE (use torwic or r2b)"
    exit 1
    ;;
esac

if [[ ! -d "$PLAY_BAG" ]]; then
  echo "Missing bag $PLAY_BAG — run: ./scripts/download_warehouse.sh --source $SOURCE --seq $SEQ"
  exit 1
fi

set +u
cogninav_ros_setup
set -u
cd "$ROOT/ros2_ws"
colcon build --packages-select cogninav_vslam cogninav_bringup cogninav_viz
set +u
source install/setup.bash
set -u

rm -f "$TRAJ"
export LD_LIBRARY_PATH="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
set +e
timeout "$TIMEOUT_SEC" ros2 launch cogninav_bringup "$LAUNCH" \
  bag_path:="$PLAY_BAG" \
  rate:="$RATE" \
  use_viz:=false
SLAM_RC=$?
set -e

if [[ ! -f "$TRAJ" ]]; then
  echo "ERROR: trajectory not saved at $TRAJ"
  exit 1
fi

LINES=$(wc -l <"$TRAJ")
GIT_SHA="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
"$ROOT/benchmarks/run_benchmark.sh" \
  --dataset "$DATASET" --seq "$SEQ" --phase "$PHASE" \
  --git-sha "$GIT_SHA" \
  --docker-image "$(cogninav_docker_image)" \
  --smoke-status "ok" \
  --smoke-note "Warehouse ($SOURCE) trajectory saved ($LINES poses)."

echo "==> Warehouse SLAM test complete ($SOURCE / $SEQ)"
