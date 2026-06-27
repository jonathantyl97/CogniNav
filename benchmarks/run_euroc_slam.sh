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

EUROC_DIR="${EUROC_DIR:-/root/Downloads/euroc}"
BAG_RAW="$EUROC_DIR/${SEQ}.bag"
BAG_ROS2="$EUROC_DIR/${SEQ}_ros2"
MAV0="$EUROC_DIR/$SEQ/mav0"
PLAY_BAG=""
TRAJ="/tmp/cogninav_euroc_trajectory.txt"
GT="/tmp/cogninav_euroc_${SEQ}_gt.txt"
HAVE_GT=false

if [[ -d "$BAG_RAW" ]]; then
  PLAY_BAG="$BAG_RAW"
elif [[ -f "$BAG_RAW" ]]; then
  if [[ -d "$BAG_ROS2" ]]; then
    echo "==> Using existing ROS 2 bag: $BAG_ROS2"
    PLAY_BAG="$BAG_ROS2"
  else
    echo "==> Converting ROS 1 bag to ROS 2: $BAG_RAW"
    rosbags-convert \
      --src "$BAG_RAW" \
      --dst "$BAG_ROS2" \
      --src-typestore ros1_noetic \
      --dst-typestore ros2_jazzy
    PLAY_BAG="$BAG_ROS2"
  fi
else
  echo "Missing bag $BAG_RAW — run: ./scripts/download_euroc.sh $SEQ"
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
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
if [[ "$HAVE_GT" == true ]]; then
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset euroc \
    --seq "$SEQ" \
    --phase 1 \
    --git-sha "$GIT_SHA" \
    --docker-image "${COGNINAV_JAZZY_IMAGE:-osrf/ros:jazzy-desktop-full}" \
    --traj "$TRAJ" \
    --gt "$GT"
else
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset euroc \
    --seq "$SEQ" \
    --phase 1 \
    --git-sha "$GIT_SHA" \
    --docker-image "${COGNINAV_JAZZY_IMAGE:-osrf/ros:jazzy-desktop-full}" \
    --smoke-status "ok" \
    --smoke-note "SLAM trajectory saved ($LINES poses); ATE skipped (no mav0 GT)."
fi

echo "==> Phase 1 test complete for $SEQ"
