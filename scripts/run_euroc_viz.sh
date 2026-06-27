#!/usr/bin/env bash
# EuRoC SLAM with Iridescence viewer (host X11 + Docker).
#
# Usage:
#   ./scripts/run_euroc_viz.sh
#   ./scripts/run_euroc_viz.sh --seq MH_01_easy --rate 2.0

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${COGNINAV_CONTAINER:-ros2_jazzy_cogninav}"
SEQ="MH_01_easy"
RATE="2.0"
DOWNLOADS_DIR="${DOWNLOADS_DIR:-$HOME/Downloads}"
EUROC_HOST="$DOWNLOADS_DIR/euroc"
BAG_PATH="/root/Downloads/euroc"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --seq) SEQ="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

BAG_ROS2_HOST="$EUROC_HOST/${SEQ}_ros2"
BAG_RAW_HOST="$EUROC_HOST/${SEQ}.bag"
if [[ -d "$BAG_ROS2_HOST" ]]; then
  BAG_PATH="$BAG_PATH/${SEQ}_ros2"
elif [[ -d "$BAG_RAW_HOST" ]]; then
  BAG_PATH="$BAG_PATH/${SEQ}.bag"
elif [[ -f "$BAG_RAW_HOST" ]]; then
  echo "ROS 1 bag only — convert first: ./benchmarks/run_euroc_slam.sh --seq $SEQ"
  exit 1
else
  echo "Missing bag — run: ./scripts/download_euroc.sh $SEQ"
  exit 1
fi

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is not set on the host (log into a graphical session first)."
  exit 1
fi

echo "==> Allowing Docker to use host X11..."
xhost +local:docker >/dev/null 2>&1 || xhost +local:root >/dev/null 2>&1 || true

docker start "$CONTAINER" >/dev/null 2>&1 || true

XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
XAUTH_ENV=()
if [[ -f "$XAUTH" ]]; then
  docker cp "$XAUTH" "$CONTAINER:/tmp/.docker.xauth" >/dev/null
  XAUTH_ENV=(-e "XAUTHORITY=/tmp/.docker.xauth")
fi

exec docker exec -it \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/cogninav-runtime \
  "${XAUTH_ENV[@]}" \
  "$CONTAINER" bash -lc "
    mkdir -p /tmp/cogninav-runtime && chmod 700 /tmp/cogninav-runtime
    source /opt/ros/jazzy/setup.bash
    cd /root/cogninav/ros2_ws
    colcon build --packages-select cogninav_viz cogninav_bringup --symlink-install >/dev/null
    source install/setup.bash
    ros2 launch cogninav_bringup euroc.launch.py \
      seq:=$SEQ \
      bag_path:=$BAG_PATH \
      rate:=$RATE
  "
