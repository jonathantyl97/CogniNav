#!/usr/bin/env bash
# Wait until ROS 2 topics publish at least one message (uses ros2 CLI).
#
# Usage:
#   ./benchmarks/wait_for_topics.sh /cogninav/odom /cogninav/stereo_points
#   ./benchmarks/wait_for_topics.sh --timeout 120 /cogninav/lane_markers

set -euo pipefail

TIMEOUT_SEC=120
TOPICS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --timeout) TIMEOUT_SEC="$2"; shift 2 ;;
    -*) echo "Unknown arg: $1"; exit 2 ;;
    *) TOPICS+=("$1"); shift ;;
  esac
done

if [[ ${#TOPICS[@]} -eq 0 ]]; then
  TOPICS=(
    /cogninav/odom
    /cogninav/map_points
    /cogninav/stereo_points
    /cogninav/lane_markers
  )
fi

wait_topic() {
  local topic="$1"
  echo "==> Waiting for ${topic} (timeout ${TIMEOUT_SEC}s)..."
  if timeout "$TIMEOUT_SEC" ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    echo "OK ${topic}"
    return 0
  fi
  echo "MISS ${topic}" >&2
  return 1
}

FAIL=0
for topic in "${TOPICS[@]}"; do
  wait_topic "$topic" || FAIL=1
done

exit "$FAIL"
