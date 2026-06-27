#!/usr/bin/env bash
# Download a TorWIC warehouse ROS bag and convert to ROS 2 for CogniNav testing.
#
# Source: Clearpath Robotics warehouse (TorWIC-SLAM dataset)
#   https://github.com/Viky397/TorWICDataset
#
# Usage:
#   ./scripts/download_warehouse.sh
#   ./scripts/download_warehouse.sh --seq aisle_ccw_run_1
#   ./scripts/download_warehouse.sh --keep-ros1   # retain 11GB ROS1 bag
#
# Output:
#   ${WAREHOUSE_DIR}/<seq>_ros2/   (ROS 2 bag directory, stereo + IMU subset)

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SEQ="aisle_cw_run_1"
WAREHOUSE_DIR="${WAREHOUSE_DIR:-${HOME}/Downloads/warehouse}"
KEEP_ROS1=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --keep-ros1) KEEP_ROS1=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

# TorWIC SLAM "Original Bags" (Jun 15, 2022) — Google Drive file IDs
declare -A TORWIC_BAG_IDS=(
  [aisle_cw_run_1]="1RSJCHnl6WFFPD0Wdb43MEsTgxueyfhwu"
  [aisle_cw_run_2]="1-wU3Ogexj6McrN9LUs2VwcDOhFkhJ4VL"
  [aisle_ccw_run_1]="1WahCGK7lUGYBvXwcb5M83UHeNwQZJ0-G"
  [aisle_ccw_run_2]="1BuFglb0w7U--BCt9Kve3-M0tDiWRtp0N"
  [hallway_full_cw_part_1]="169adLtoFNYS_drJdeNELTHbFVra3DSXM"
  [hallway_full_cw_part_2]="1zziS1YmJpRuZjK796_g01uvQgbcfaQxT"
  [hallway_full_ccw_part_1]="1v2u6Lc1ho3PlHtMKbWIAo8wqRqAFWFbD"
  [hallway_full_ccw_part_2]="1Zi43hI0x__zjqkWqgYjN-KWJHu7ptHFg"
  [hallway_straight_ccw_part_1]="144q6GVWyIx7aJVjoe2FN2P35Z9eATPv9"
  [hallway_straight_ccw_part_2]="1nkYMSyEH0Lk_p8eCH0WTBw8H_2U16E9i"
)

if [[ -z "${TORWIC_BAG_IDS[$SEQ]+x}" ]]; then
  echo "Unknown sequence: $SEQ"
  echo "Available: ${!TORWIC_BAG_IDS[*]}"
  exit 1
fi

mkdir -p "$WAREHOUSE_DIR"
ROS1_BAG="$WAREHOUSE_DIR/${SEQ}.bag"
ROS2_BAG="$WAREHOUSE_DIR/${SEQ}_ros2"

if [[ -d "$ROS2_BAG" && -f "$ROS2_BAG/metadata.yaml" ]]; then
  echo "ROS 2 warehouse bag ready: $ROS2_BAG"
  exit 0
fi

if ! python3 -c "import gdown" 2>/dev/null; then
  echo "Installing gdown..."
  pip3 install gdown --break-system-packages
fi

if [[ ! -f "$ROS1_BAG" ]]; then
  echo "==> Downloading TorWIC warehouse bag: $SEQ (~11 GB ROS1)..."
  python3 - "$ROS1_BAG" "${TORWIC_BAG_IDS[$SEQ]}" <<'PY'
import sys
import gdown
gdown.download(id=sys.argv[2], output=sys.argv[1], quiet=False)
PY
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  ROS_DISTRO="jazzy"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  ROS_DISTRO="humble"
else
  ROS_DISTRO="jazzy"
fi

echo "==> Converting to ROS 2 ($ROS_DISTRO): Azure stereo (compressed RGB) + left IMU..."
rosbags-convert \
  --src "$ROS1_BAG" \
  --dst "$ROS2_BAG" \
  --src-typestore ros1_noetic \
  --dst-typestore "ros2_${ROS_DISTRO}" \
  --include-topic /left_azure/rgb/image_raw/compressed \
  --include-topic /right_azure/rgb/image_raw/compressed \
  --include-topic /left_azure/imu \
  --include-topic /tf_static

if [[ "$ROS_DISTRO" == "humble" ]]; then
  python3 "$ROOT/scripts/sanitize_ros2_bag_for_humble.py" "$ROS2_BAG"
fi

if [[ "$KEEP_ROS1" != true ]]; then
  echo "==> Removing ROS1 bag to save disk (${ROS1_BAG})"
  rm -f "$ROS1_BAG"
fi

echo ""
echo "Done."
echo "  ROS 2 bag: $ROS2_BAG"
echo "  Play:  ros2 bag play $ROS2_BAG --clock"
echo "  SLAM:  ros2 launch cogninav_bringup warehouse.launch.py bag_path:=$ROS2_BAG"
