#!/usr/bin/env bash
# Full CogniNav stack on TorWIC warehouse bag + Iridescence.
#
# Default: SLAM + viz only (lighter — avoids freezing the host).
# Use --full to add dense depth + lane detection.
#
# Usage:
#   ./scripts/run_warehouse_viz.sh --seq aisle_cw_run_1
#   ./scripts/run_warehouse_viz.sh --seq aisle_cw_run_1 --full
#   ./scripts/run_warehouse_viz.sh --headless
#   ./scripts/run_warehouse_viz.sh --build   # force colcon rebuild

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"

HEADLESS=false
FULL=false
FORCE_BUILD=false
for arg in "$@"; do
  case "$arg" in
    --headless) HEADLESS=true ;;
    --full) FULL=true ;;
    --build) FORCE_BUILD=true ;;
  esac
done

if [[ "$HEADLESS" != true ]]; then
  export COGNINAV_DOCKER_X11=1
fi
cogninav_reexec_in_docker "scripts/run_warehouse_viz.sh" "$@"

SEQ="aisle_cw_run_1"
BAG_PATH=""
RATE="1.0"
BAG_DELAY="10.0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --bag) BAG_PATH="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    --headless) HEADLESS=true; shift ;;
    --full) FULL=true; shift ;;
    --build) FORCE_BUILD=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

WAREHOUSE_DIR="${WAREHOUSE_DIR:-$(cogninav_downloads_dir)/warehouse}"
if [[ -z "$BAG_PATH" ]]; then
  BAG_PATH="$WAREHOUSE_DIR/${SEQ}_ros2"
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "Missing bag $BAG_PATH — run: ./scripts/download_warehouse.sh --source torwic --seq $SEQ"
  exit 1
fi

"$ROOT/scripts/download_models.sh"

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

USE_VIZ=true
USE_DEPTH=false
USE_LANES=false
if [[ "$HEADLESS" == true ]]; then
  USE_VIZ=false
fi
if [[ "$FULL" == true ]]; then
  USE_DEPTH=true
  USE_LANES=true
  echo "==> Full stack: SLAM + depth + lanes + viz (heavier CPU load)"
else
  echo "==> Light stack: SLAM + viz only (add --full for depth + lanes)"
fi

exec ros2 launch cogninav_bringup warehouse.launch.py \
  bag_path:="$BAG_PATH" \
  rate:="$RATE" \
  bag_play_delay:="$BAG_DELAY" \
  use_viz:="$USE_VIZ" \
  use_vslam:=true \
  use_depth:="$USE_DEPTH" \
  use_lanes:="$USE_LANES"
