#!/usr/bin/env bash
# Download warehouse ROS 2 bags for CogniNav testing.
#
# Sources:
#   torwic  — Clearpath Robotics warehouse (TorWIC-SLAM), ROS1 → ROS2 convert
#             https://github.com/Viky397/TorWICDataset
#   r2b     — NVIDIA r2b_storage (native ROS 2, shelving/pallets scene)
#             https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/resources/r2bdataset2023
#
# Usage:
#   ./scripts/download_warehouse.sh
#   ./scripts/download_warehouse.sh --source torwic --seq aisle_ccw_run_1
#   ./scripts/download_warehouse.sh --source r2b
#   ./scripts/download_warehouse.sh --source torwic --keep-ros1

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE="torwic"
SEQ="aisle_cw_run_1"
WAREHOUSE_DIR="${WAREHOUSE_DIR:-${HOME}/Downloads/warehouse}"
KEEP_ROS1=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --source) SOURCE="$2"; shift 2 ;;
    --seq) SEQ="$2"; shift 2 ;;
    --keep-ros1) KEEP_ROS1=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

mkdir -p "$WAREHOUSE_DIR"

download_r2b_storage() {
  local bag_dir="$WAREHOUSE_DIR/r2b_storage"
  local ngc_base="https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/r2bdataset2023/versions/3/files/r2b_storage"

  if [[ -d "$bag_dir" && -f "$bag_dir/metadata.yaml" && -f "$bag_dir/r2b_storage_0.db3" ]]; then
    echo "ROS 2 warehouse bag ready: $bag_dir"
    return 0
  fi

  mkdir -p "$bag_dir"
  echo "==> Downloading NVIDIA r2b_storage (~2.9 GB, native ROS 2)..."
  wget -q --show-progress -O "$bag_dir/metadata.yaml" "${ngc_base}/metadata.yaml"
  wget -q --show-progress -O "$bag_dir/r2b_storage_0.db3" "${ngc_base}/r2b_storage_0.db3"

  echo ""
  echo "Done."
  echo "  ROS 2 bag: $bag_dir"
  echo "  Play:  ros2 bag play $bag_dir --clock"
  echo "  SLAM:  ros2 launch cogninav_bringup r2b_storage.launch.py bag_path:=$bag_dir"
}

download_torwic() {
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
    echo "Unknown TorWIC sequence: $SEQ"
    echo "Available: ${!TORWIC_BAG_IDS[*]}"
    exit 1
  fi

  local ros1_bag="$WAREHOUSE_DIR/${SEQ}.bag"
  local ros2_bag="$WAREHOUSE_DIR/${SEQ}_ros2"

  if [[ -d "$ros2_bag" && -f "$ros2_bag/metadata.yaml" ]]; then
    echo "ROS 2 warehouse bag ready: $ros2_bag"
    return 0
  fi

  if ! python3 -c "import gdown" 2>/dev/null; then
    echo "Installing gdown..."
    pip3 install gdown --break-system-packages
  fi

  if [[ ! -f "$ros1_bag" ]]; then
    echo "==> Downloading TorWIC warehouse bag: $SEQ (~11 GB ROS1)..."
    python3 - "$ros1_bag" "${TORWIC_BAG_IDS[$SEQ]}" <<'PY'
import sys
import gdown
gdown.download(id=sys.argv[2], output=sys.argv[1], quiet=False)
PY
  fi

  local ros_distro="jazzy"
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ros_distro="jazzy"
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ros_distro="humble"
  fi

  echo "==> Converting to ROS 2 ($ros_distro): Azure stereo (compressed RGB) + left IMU..."
  rosbags-convert \
    --src "$ros1_bag" \
    --dst "$ros2_bag" \
    --src-typestore ros1_noetic \
    --dst-typestore "ros2_${ros_distro}" \
    --include-topic /left_azure/rgb/image_raw/compressed \
    --include-topic /right_azure/rgb/image_raw/compressed \
    --include-topic /left_azure/imu \
    --include-topic /tf_static

  if [[ "$ros_distro" == "humble" ]]; then
    python3 "$ROOT/scripts/sanitize_ros2_bag_for_humble.py" "$ros2_bag"
  fi

  if [[ "$KEEP_ROS1" != true ]]; then
    echo "==> Removing ROS1 bag to save disk ($ros1_bag)"
    rm -f "$ros1_bag"
  fi

  echo ""
  echo "Done."
  echo "  ROS 2 bag: $ros2_bag"
  echo "  Play:  ros2 bag play $ros2_bag --clock"
  echo "  SLAM:  ros2 launch cogninav_bringup warehouse.launch.py bag_path:=$ros2_bag"
}

case "$SOURCE" in
  torwic) download_torwic ;;
  r2b) download_r2b_storage ;;
  *)
    echo "Unknown source: $SOURCE (use torwic or r2b)"
    exit 1
    ;;
esac
