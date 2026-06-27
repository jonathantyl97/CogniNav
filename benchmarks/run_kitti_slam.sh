#!/usr/bin/env bash
# Run CogniNav SLAM on a KITTI odometry sequence (Phase 2, stereo only).
#
# Usage:
#   ./benchmarks/run_kitti_slam.sh --seq 00

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_docker "benchmarks/run_kitti_slam.sh" "$@"

SEQ="00"
RATE="1.0"
TIMEOUT_SEC="${SLAM_TIMEOUT_SEC:-1200}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

SEQ="$(printf '%02d' "$((10#$SEQ))")"
KITTI_DIR="${KITTI_DIR:-$(cogninav_downloads_dir)/kitti}"
PLAY_BAG="$KITTI_DIR/${SEQ}_ros2"
SEQ_DIR="$KITTI_DIR/sequences/$SEQ"
POSES="$KITTI_DIR/poses/${SEQ}.txt"
TRAJ="/tmp/cogninav_kitti_trajectory.txt"
GT="/tmp/cogninav_kitti_${SEQ}_gt.txt"
HAVE_GT=false

if [[ ! -d "$PLAY_BAG" ]]; then
  if [[ -d "$SEQ_DIR/image_0" ]]; then
    "$ROOT/scripts/bag_from_kitti.sh" "$SEQ"
  else
    echo "Missing KITTI data — run: ./scripts/download_kitti.sh $SEQ"
    exit 1
  fi
fi

set +u
cogninav_ros_setup
set -u
cd "$ROOT/ros2_ws"
colcon build --packages-select cogninav_vslam cogninav_bringup cogninav_viz
set +u
source install/setup.bash
set -u

if [[ -f "$POSES" && -f "$SEQ_DIR/times.txt" ]]; then
  python3 "$ROOT/benchmarks/kitti_gt_to_tum.py" "$POSES" "$SEQ_DIR/times.txt" "$GT"
  HAVE_GT=true
else
  echo "WARN: no KITTI poses at $POSES — smoke only"
fi

rm -f "$TRAJ"
export LD_LIBRARY_PATH="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
set +e
timeout "$TIMEOUT_SEC" ros2 launch cogninav_bringup kitti.launch.py \
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
    --dataset kitti --seq "$SEQ" --phase 2 \
    --git-sha "$GIT_SHA" \
    --docker-image "$(cogninav_docker_image)" \
    --traj "$TRAJ" --gt "$GT"
else
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset kitti --seq "$SEQ" --phase 2 \
    --git-sha "$GIT_SHA" \
    --docker-image "$(cogninav_docker_image)" \
    --smoke-status "ok" \
    --smoke-note "KITTI trajectory saved ($LINES poses); ATE skipped (no poses file)."
fi

echo "==> Phase 2 KITTI test complete for sequence $SEQ"
