#!/usr/bin/env bash
# Run CogniNav SLAM on a EuRoC bag and evaluate ATE (Phase 1).
#
# Usage:
#   ./benchmarks/run_euroc_slam.sh --seq MH_01_easy
#
# Supports:
#   - ROS 2 bag directory: ${EUROC_DIR}/<seq>.bag/
#   - ROS 1 bag file (HuggingFace mirror): ${EUROC_DIR}/<seq>.bag

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
cogninav_reexec_in_docker "benchmarks/run_euroc_slam.sh" "$@"

SEQ="MH_01_easy"
RATE="1.0"
TIMEOUT_SEC="${SLAM_TIMEOUT_SEC:-600}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

EUROC_DIR="${EUROC_DIR:-$(cogninav_downloads_dir)/euroc}"
BAG_RAW="$EUROC_DIR/${SEQ}.bag"
BAG_ROS2="$EUROC_DIR/${SEQ}_ros2_$(cogninav_ros_distro)"
MAV0="$EUROC_DIR/$SEQ/mav0"
PLAY_BAG=""
TRAJ="/tmp/cogninav_euroc_trajectory.txt"
GT="/tmp/cogninav_euroc_${SEQ}_gt.txt"
HAVE_GT=false

if [[ -d "$BAG_RAW" ]]; then
  PLAY_BAG="$BAG_RAW"
elif [[ -f "$BAG_RAW" ]]; then
  if PLAY_BAG="$(cogninav_resolve_ros2_bag "$SEQ" "$EUROC_DIR" "$ROOT" 2>/dev/null)"; then
    echo "==> Using ROS 2 bag: $PLAY_BAG"
  else
    echo "==> Converting ROS 1 bag to ROS 2: $BAG_RAW"
    rosbags-convert \
      --src "$BAG_RAW" \
      --dst "$BAG_ROS2" \
      --src-typestore ros1_noetic \
      --dst-typestore "ros2_$(cogninav_ros_distro)"
    if [[ "$(cogninav_ros_distro)" == "humble" ]]; then
      python3 "$ROOT/scripts/sanitize_ros2_bag_for_humble.py" "$BAG_ROS2"
    fi
    PLAY_BAG="$BAG_ROS2"
  fi
else
  echo "Missing bag $BAG_RAW — run: ./scripts/download_euroc.sh $SEQ"
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

if [[ -f "$MAV0/cam0/data.csv" ]]; then
  python3 "$ROOT/benchmarks/euroc_gt_to_tum.py" "$MAV0" "$GT"
  HAVE_GT=true
else
  echo "WARN: no mav0 ground truth at $MAV0 — SLAM smoke only (no ATE)"
fi

rm -f "$TRAJ"

echo "==> Running SLAM on $SEQ (timeout ${TIMEOUT_SEC}s)..."
echo "    bag: $PLAY_BAG"
export LD_LIBRARY_PATH="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
set +e
timeout "$TIMEOUT_SEC" ros2 launch cogninav_bringup euroc.launch.py \
  seq:="$SEQ" \
  bag_path:="$PLAY_BAG" \
  rate:="$RATE" \
  use_viz:=false
SLAM_RC=$?
set -e
if [[ "$SLAM_RC" -ne 0 && "$SLAM_RC" -ne 124 ]]; then
  echo "WARN: launch exited with code $SLAM_RC"
fi

if [[ ! -f "$TRAJ" ]]; then
  echo "ERROR: trajectory not saved at $TRAJ"
  exit 1
fi

LINES=$(wc -l <"$TRAJ")
echo "==> Trajectory saved: $TRAJ ($LINES poses)"
if [[ "$LINES" -lt 10 ]]; then
  echo "ERROR: trajectory too short"
  exit 1
fi

GIT_SHA="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
PHASE="${COGNINAV_BENCHMARK_PHASE:-1}"
DOCKER_IMAGE="$(cogninav_docker_image)"
if [[ "$HAVE_GT" == true ]]; then
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset euroc \
    --seq "$SEQ" \
    --phase "$PHASE" \
    --git-sha "$GIT_SHA" \
    --docker-image "$DOCKER_IMAGE" \
    --traj "$TRAJ" \
    --gt "$GT"
else
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset euroc \
    --seq "$SEQ" \
    --phase "$PHASE" \
    --git-sha "$GIT_SHA" \
    --docker-image "$DOCKER_IMAGE" \
    --smoke-status "ok" \
    --smoke-note "SLAM trajectory saved ($LINES poses); ATE skipped (no mav0 GT)."
fi

echo "==> EuRoC SLAM test complete for $SEQ (phase $PHASE)"
