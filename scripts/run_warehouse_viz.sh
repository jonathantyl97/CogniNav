#!/usr/bin/env bash
# CogniNav warehouse bag + 3D viewer.
#
# Default: SLAM + Iridescence (light stack).
# Use --full to add lane detection. Use --depth for dense stereo (works on r2b).
#
# Viewer (pick one; default --iris when not --headless):
#   --iris      Iridescence desktop viewer (map, trajectory, camera panel)
#   --pangolin  ORB-SLAM3 built-in Pangolin viewer (keyframes + map only)
#   --headless  no viewer window
#
# Usage:
#   ./scripts/run_warehouse_viz.sh --seq aisle_cw_run_1
#   ./scripts/run_warehouse_viz.sh --seq aisle_ccw_run_1 --pangolin
#   ./scripts/run_warehouse_viz.sh --source r2b --iris --full
#   ./scripts/run_warehouse_viz.sh --headless
#   ./scripts/run_warehouse_viz.sh --build

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"

HEADLESS=false
FULL=false
DEPTH=false
FORCE_BUILD=false
REALTIME=false
SOURCE="r2b"
VIEWER=""
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=true ;;
    --full) FULL=true ;;
    --depth) DEPTH=true ;;
    --build) FORCE_BUILD=true ;;
    --realtime) REALTIME=true ;;
    --iris|--pangolin) VIEWER="${arg#--}" ;;
    --source) ;;
  esac
done

if [[ "$HEADLESS" != true ]]; then
  export COGNINAV_DOCKER_X11=1
fi
cogninav_reexec_in_docker "scripts/run_warehouse_viz.sh" "$@"

SEQ="aisle_cw_run_1"
BAG_PATH=""
RATE=""
BAG_DELAY="10.0"
BAG_LOOP=false
SOURCE="r2b"
VIEWER=""
HEADLESS=false
FULL=false
DEPTH=false
FORCE_BUILD=false
REALTIME=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE="$2"; shift 2 ;;
    --seq) SEQ="$2"; shift 2 ;;
    --bag) BAG_PATH="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --headless) HEADLESS=true; shift ;;
    --iris) VIEWER="iris"; shift ;;
    --pangolin) VIEWER="pangolin"; shift ;;
    --full) FULL=true; shift ;;
    --depth) DEPTH=true; shift ;;
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

if [[ -z "$RATE" ]]; then
  if [[ "$REALTIME" == true ]]; then
    RATE="1.0"
  elif [[ "$SOURCE" == "torwic" ]]; then
    RATE="0.5"
  else
    RATE="1.0"
  fi
fi

if [[ "$VIEWER" != "iris" && "$VIEWER" != "pangolin" && "$VIEWER" != "none" ]]; then
  echo "Unknown viewer: $VIEWER (use --iris, --pangolin, or --headless)"
  exit 1
fi

case "$SOURCE" in
  torwic)
    LAUNCH="warehouse.launch.py"
    BAG_LOOP=true
    ;;
  r2b)
    LAUNCH="r2b_storage.launch.py"
    ;;
  *)
    echo "Unknown source: $SOURCE (use torwic or r2b)"
    exit 1
    ;;
esac

WAREHOUSE_DIR="${WAREHOUSE_DIR:-$(cogninav_downloads_dir)/warehouse}"
if [[ -z "$BAG_PATH" ]]; then
  if [[ "$SOURCE" == "r2b" ]]; then
    BAG_PATH="$WAREHOUSE_DIR/r2b_storage"
  else
    BAG_PATH="$WAREHOUSE_DIR/${SEQ}_ros2"
  fi
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "Missing bag $BAG_PATH — see README.md (Datasets)"
  exit 1
fi

set +u
cogninav_ros_setup
set -u

export RMW_FASTRTPS_USE_SHM=0
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/cogninav-runtime}"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true

cd "$ROOT/ros2_ws"
if [[ "$FORCE_BUILD" == true ]] || [[ ! -f install/setup.bash ]]; then
  colcon build --packages-select \
    cogninav_vslam cogninav_depth cogninav_lanes cogninav_bringup cogninav_viz
else
  echo "==> Skipping colcon build (use --build to force)"
fi
set +u
source install/setup.bash
set -u

export LD_LIBRARY_PATH="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"

USE_VIZ=false
USE_PANGOLIN_VIEWER=false
case "$VIEWER" in
  iris)
    USE_VIZ=true
  ;;
  pangolin)
    USE_PANGOLIN_VIEWER=true
  ;;
esac

USE_DEPTH=false
USE_LANES=false
SHOW_STEREO_DEPTH=false
if [[ "$FULL" == true ]]; then
  USE_LANES=true
  if [[ "$SOURCE" == "r2b" ]]; then
    USE_DEPTH=true
    SHOW_STEREO_DEPTH=true
    echo "==> Full stack: SLAM + depth + lanes + viz ($VIEWER)"
  else
    echo "==> Full stack: SLAM + lanes + viz ($VIEWER)"
  fi
  echo "==> Wait ~10s for bag play, then ~20s for first viz output"
else
  echo "==> Light stack: SLAM + viz ($VIEWER)"
  if [[ "$SOURCE" == "torwic" && "$REALTIME" != true && "$RATE" == "0.5" ]]; then
    echo "==> TorWIC bag at 0.5x (CPU-heavy 1280px stereo). Use --realtime or --rate 1.0 to push harder."
  fi
  echo "==> Wait ~10s for bag play, then ~20s for first viz output"
fi
if [[ "$DEPTH" == true ]]; then
  USE_DEPTH=true
  SHOW_STEREO_DEPTH=true
  if [[ "$SOURCE" == "torwic" ]]; then
    echo "==> Experimental raw stereo depth (TorWIC images are not rectified)"
  else
    echo "==> Dense stereo depth enabled (r2b D455 IR)"
  fi
fi

if [[ "$VIEWER" == "pangolin" ]]; then
  echo "==> Pangolin: ORB-SLAM3 keyframes/map only (no lanes/depth overlay)"
fi

exec ros2 launch cogninav_bringup "$LAUNCH" \
  bag_path:="$BAG_PATH" \
  rate:="$RATE" \
  bag_play_delay:="$BAG_DELAY" \
  bag_loop:="$BAG_LOOP" \
  use_viz:="$USE_VIZ" \
  use_pangolin_viewer:="$USE_PANGOLIN_VIEWER" \
  use_vslam:=true \
  use_depth:="$USE_DEPTH" \
  use_lanes:="$USE_LANES" \
  show_stereo_depth:="$SHOW_STEREO_DEPTH"
