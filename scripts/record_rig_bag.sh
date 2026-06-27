#!/usr/bin/env bash
# Record a live rig rosbag for CogniNav replay / warehouse validation (Phase 4).
#
# Usage:
#   ./scripts/record_rig_bag.sh --rig realsense_d455
#   ./scripts/record_rig_bag.sh --rig zed2 --name warehouse_aisle1

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RIG="realsense_d455"
NAME=""
OUT_DIR="${COGNINAV_BAG_DIR:-$HOME/Downloads/cogninav}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rig) RIG="$2"; shift 2 ;;
    --name) NAME="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

case "$RIG" in
  realsense_d455)
    TOPICS=(
      /camera/camera/infra1/image_rect_raw
      /camera/camera/infra2/image_rect_raw
      /camera/camera/imu
      /camera/camera/infra1/camera_info
      /camera/camera/infra2/camera_info
    )
    ;;
  zed2)
    TOPICS=(
      /zed/zed_node/left/image_rect_color
      /zed/zed_node/right/image_rect_color
      /zed/zed_node/imu/data
      /zed/zed_node/left/camera_info
      /zed/zed_node/right/camera_info
    )
    ;;
  *)
    echo "Unknown rig: $RIG (use realsense_d455 or zed2)"
    exit 1
    ;;
esac

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
if [[ -n "$NAME" ]]; then
  BAG_NAME="${RIG}_${NAME}_${STAMP}"
else
  BAG_NAME="${RIG}_${STAMP}"
fi

mkdir -p "$OUT_DIR"
BAG_PATH="$OUT_DIR/$BAG_NAME"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

echo "==> Recording rig '$RIG' to $BAG_PATH"
echo "    Topics: ${TOPICS[*]}"
exec ros2 bag record -o "$BAG_PATH" "${TOPICS[@]}"
