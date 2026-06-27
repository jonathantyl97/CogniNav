#!/usr/bin/env bash
# Run CogniNav SLAM on a TUM-VI bag (Phase 2).
#
# Usage:
#   ./benchmarks/run_tumvi_slam.sh --seq dataset-room1_512_16

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_docker "benchmarks/run_tumvi_slam.sh" "$@"

# shellcheck source=benchmarks/bag_convert.sh
source "$ROOT/benchmarks/bag_convert.sh"

SEQ="dataset-room1_512_16"
RATE="1.0"
TIMEOUT_SEC="${SLAM_TIMEOUT_SEC:-900}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

TUMVI_DIR="${TUMVI_DIR:-$(cogninav_downloads_dir)/tumvi}"
BAG_RAW="$TUMVI_DIR/${SEQ}.bag"
BAG_ROS2="$TUMVI_DIR/${SEQ}_ros2_$(cogninav_ros_distro)"
GT_SRC="$TUMVI_DIR/groundtruth/${SEQ}.txt"
TRAJ="/tmp/cogninav_tumvi_trajectory.txt"
GT="/tmp/cogninav_tumvi_${SEQ}_gt.txt"
HAVE_GT=false

if PLAY_BAG="$(cogninav_resolve_ros2_bag "$SEQ" "$TUMVI_DIR" "$ROOT" 2>/dev/null)"; then
  :
elif [[ -f "$BAG_RAW" ]]; then
  PLAY_BAG="$(convert_ros1_bag_if_needed "$BAG_RAW" "$BAG_ROS2" "$(cogninav_ros_distro)")"
elif [[ -d "$BAG_RAW" ]]; then
  PLAY_BAG="$BAG_RAW"
else
  echo "Missing bag at $BAG_RAW or $BAG_ROS2"
  echo "Run: ./scripts/download_tumvi.sh ${SEQ%.bag}"
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

if [[ -f "$GT_SRC" ]]; then
  python3 "$ROOT/benchmarks/tumvi_gt_to_tum.py" "$GT_SRC" "$GT"
  HAVE_GT=true
else
  echo "WARN: no TUM-VI GT at $GT_SRC — smoke only"
fi

rm -f "$TRAJ"
export LD_LIBRARY_PATH="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
set +e
timeout "$TIMEOUT_SEC" ros2 launch cogninav_bringup tumvi.launch.py \
  seq:="$SEQ" \
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
if [[ "$HAVE_GT" == true ]]; then
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset tumvi --seq "$SEQ" --phase 2 \
    --git-sha "$GIT_SHA" \
    --docker-image "$(cogninav_docker_image)" \
    --traj "$TRAJ" --gt "$GT"
else
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset tumvi --seq "$SEQ" --phase 2 \
    --git-sha "$GIT_SHA" \
    --docker-image "$(cogninav_docker_image)" \
    --smoke-status "ok" \
    --smoke-note "TUM-VI trajectory saved ($LINES poses); ATE skipped (no GT file)."
fi

echo "==> Phase 2 TUM-VI test complete for $SEQ"
