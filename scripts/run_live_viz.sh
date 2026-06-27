#!/usr/bin/env bash
# Launch CogniNav on a live stereo rig with Iridescence (host X11 + Docker).
#
# Prerequisite: camera driver running on host or in container with USB passthrough.
#
# Usage:
#   ./scripts/run_live_viz.sh
#   ./scripts/run_live_viz.sh --rig zed2

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${COGNINAV_CONTAINER:-ros2_jazzy_cogninav}"
RIG="realsense_d455"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --rig) RIG="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

if [[ -z "${DISPLAY:-}" ]]; then
  echo "ERROR: DISPLAY is not set on the host."
  exit 1
fi

xhost +local:docker >/dev/null 2>&1 || xhost +local:root >/dev/null 2>&1 || true
docker start "$CONTAINER" >/dev/null 2>&1 || true

XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
if [[ -f "$XAUTH" ]]; then
  docker cp "$XAUTH" "$CONTAINER:/tmp/.docker.xauth" >/dev/null 2>&1 || true
fi

docker exec -it \
  -e DISPLAY="$DISPLAY" \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/cogninav-runtime \
  -e XAUTHORITY=/tmp/.docker.xauth \
  "$CONTAINER" bash -lc "
    mkdir -p /tmp/cogninav-runtime && chmod 700 /tmp/cogninav-runtime
    source /opt/ros/jazzy/setup.bash
    cd /root/cogninav/ros2_ws
    colcon build --packages-select cogninav_bringup cogninav_vslam cogninav_viz cogninav_depth cogninav_lanes
    source install/setup.bash
    export LD_LIBRARY_PATH=/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:\${LD_LIBRARY_PATH:-}
    ros2 launch cogninav_bringup live.launch.py rig:=$RIG
  "
