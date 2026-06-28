#!/usr/bin/env bash
# Replay a recorded live-rig bag through CogniNav (Phase 4).
#
# Usage:
#   ./scripts/run_rig_bag_viz.sh --rig realsense_d455 --bag ~/Downloads/cogninav/my_run
#   ./scripts/run_rig_bag_viz.sh --rig zed2 --bag /root/Downloads/cogninav/zed2_test --full

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"

HEADLESS=false
FULL=false
FORCE_BUILD=false
VIEWER=""
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=true ;;
    --full) FULL=true ;;
    --build) FORCE_BUILD=true ;;
    --iris|--pangolin) VIEWER="${arg#--}" ;;
    --rig|--bag) ;;
  esac
done

if [[ "$HEADLESS" != true ]]; then
  export COGNINAV_DOCKER_X11=1
fi
cogninav_reexec_in_docker "scripts/run_rig_bag_viz.sh" "$@"

RIG="realsense_d455"
BAG_PATH=""
RATE="1.0"
HEADLESS=false
FULL=false
FORCE_BUILD=false
VIEWER=""
REALTIME=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rig) RIG="$2"; shift 2 ;;
    --bag) BAG_PATH="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --headless) HEADLESS=true; shift ;;
    --iris) VIEWER="iris"; shift ;;
    --pangolin) VIEWER="pangolin"; shift ;;
    --full) FULL=true; shift ;;
    --build) FORCE_BUILD=true; shift ;;
    --realtime) REALTIME=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

if [[ "$HEADLESS" == true ]]; then
  VIEWER="none"
elif [[ -z "$VIEWER" ]]; then
  VIEWER="iris"
fi

DOWNLOADS="$(cogninav_downloads_dir)/cogninav"
if [[ -z "$BAG_PATH" ]]; then
  BAG_PATH="$(ls -dt "$DOWNLOADS"/${RIG}_* 2>/dev/null | head -1 || true)"
fi
if [[ -z "$BAG_PATH" || ! -d "$BAG_PATH" ]]; then
  echo "Missing bag — record one: ./scripts/record_rig_bag.sh --rig $RIG --name warehouse_aisle1"
  exit 1
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

USE_DEPTH=false
USE_LANES=false
SHOW_STEREO=true
if [[ "$FULL" == true ]]; then
  USE_DEPTH=true
  USE_LANES=true
fi

echo "==> Rig replay: $RIG  bag=$BAG_PATH  viewer=$VIEWER"

exec ros2 launch cogninav_bringup rig_replay.launch.py \
  rig:="$RIG" \
  bag_path:="$BAG_PATH" \
  rate:="$RATE" \
  use_viz:="$USE_VIZ" \
  use_pangolin_viewer:="$USE_PANGOLIN" \
  use_depth:="$USE_DEPTH" \
  use_lanes:="$USE_LANES" \
  show_stereo_depth:="$SHOW_STEREO"
