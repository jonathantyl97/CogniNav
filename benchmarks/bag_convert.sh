#!/usr/bin/env bash
# Convert ROS 1 bag to ROS 2 if needed (shared helper).

set -euo pipefail

convert_ros1_bag_if_needed() {
  local raw_bag="$1"
  local ros2_bag="$2"
  local distro="${3:-jazzy}"

  if [[ -d "$ros2_bag" ]]; then
    echo "$ros2_bag"
    return 0
  fi
  if [[ ! -f "$raw_bag" ]]; then
    return 1
  fi
  echo "==> Converting ROS 1 bag to ROS 2: $raw_bag" >&2
  rosbags-convert \
    --src "$raw_bag" \
    --dst "$ros2_bag" \
    --src-typestore ros1_noetic \
    --dst-typestore "ros2_${distro}"
  if [[ "$distro" == "humble" ]]; then
    local root
    root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    python3 "$root/scripts/sanitize_ros2_bag_for_humble.py" "$ros2_bag"
  fi
  echo "$ros2_bag"
}
