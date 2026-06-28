#!/usr/bin/env bash
# KITTI road dataset — SLAM + lanes + dynamic-mask SLAM (Phase 6).
#
# Usage:
#   ./scripts/run_kitti_viz.sh --seq 00 --build
#   ./scripts/run_kitti_viz.sh --seq 00 --headless

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"

HEADLESS=false
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=true ;;
    --iris|--pangolin) ;;
    --seq) ;;
  esac
done

if [[ "$HEADLESS" != true ]]; then
  export COGNINAV_DOCKER_X11=1
fi
cogninav_reexec_in_docker "scripts/run_kitti_viz.sh" "$@"

SEQ="00"
RATE="1.0"
HEADLESS=false
VIEWER=""
FORCE_BUILD=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --headless) HEADLESS=true; shift ;;
    --iris) VIEWER="iris"; shift ;;
    --pangolin) VIEWER="pangolin"; shift ;;
    --build) FORCE_BUILD=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

SEQ="$(printf '%02d' "$((10#$SEQ))")"
KITTI_DIR="$(cogninav_downloads_dir)/kitti"
BAG_PATH="$KITTI_DIR/${SEQ}_ros2"

if [[ ! -d "$BAG_PATH" ]]; then
  echo "Missing bag $BAG_PATH — see README.md (Datasets)"
  exit 1
fi

if [[ "$HEADLESS" == true ]]; then
  VIEWER="none"
elif [[ -z "$VIEWER" ]]; then
  VIEWER="iris"
fi

set +u
cogninav_ros_setup
set -u

cd "$ROOT/ros2_ws"
if [[ "$FORCE_BUILD" == true ]] || [[ ! -f install/setup.bash ]]; then
  colcon build --packages-select \
    cogninav_vslam cogninav_depth cogninav_lanes cogninav_bringup cogninav_viz
fi
set +u
source install/setup.bash
set -u

export LD_LIBRARY_PATH="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"

USE_VIZ=false
USE_PANGOLIN=false
case "$VIEWER" in
  iris) USE_VIZ=true ;;
  pangolin) USE_PANGOLIN=true ;;
esac

echo "==> KITTI replay: seq=$SEQ bag=$BAG_PATH viewer=$VIEWER"

exec ros2 launch cogninav_bringup kitti.launch.py \
  bag_path:="$BAG_PATH" \
  rate:="$RATE" \
  use_viz:="$USE_VIZ" \
  use_pangolin_viewer:="$USE_PANGOLIN" \
  use_vslam:=true \
  use_depth:=true \
  use_lanes:=true
