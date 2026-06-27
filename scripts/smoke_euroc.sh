#!/usr/bin/env bash
# Phase 0 smoke: verify ORB-SLAM3 + workspace on EuRoC MH_01_easy.
#
# Usage (inside CogniNav Docker container):
#   ./scripts/smoke_euroc.sh
#   ./scripts/smoke_euroc.sh --workspace-only   # skip EuRoC download / SLAM run

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"
SEQ="${EUROC_SEQ:-MH_01_easy}"
EUROC_DIR="${EUROC_DIR:-$(cogninav_downloads_dir)/euroc}"
ORB_DIR="$ROOT/third_party/ORB_SLAM3"
ORB_LIB="$ORB_DIR/lib/libORB_SLAM3.so"
WORKSPACE_ONLY=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace-only) WORKSPACE_ONLY=true; shift ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

echo "==> Phase 0 smoke: EuRoC $SEQ"

if [[ ! -f "$ORB_LIB" ]]; then
  echo "Missing $ORB_LIB — run ./scripts/setup_deps.sh first"
  exit 1
fi

if [[ -f "$ROOT/benchmarks/cogninav_docker.sh" ]]; then
  set +u
  cogninav_ros_setup
  set -u
  DOCKER_IMAGE="$(cogninav_docker_image)"
else
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  DOCKER_IMAGE="${COGNINAV_JAZZY_IMAGE:-osrf/ros:jazzy-desktop-full}"
fi

cd "$ROOT/ros2_ws"
colcon build
set +u
source install/setup.bash
set -u

GIT_SHA="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"

record_result() {
  local status="$1"
  local note="$2"
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset euroc \
    --seq "$SEQ" \
    --phase 0 \
    --git-sha "$GIT_SHA" \
    --docker-image "$DOCKER_IMAGE" \
    --smoke-status "$status" \
    --smoke-note "$note"
}

if [[ "$WORKSPACE_ONLY" == true ]]; then
  record_result "workspace_ok" "colcon build + libORB_SLAM3.so verified; EuRoC SLAM run skipped."
  echo "==> Phase 0 workspace smoke passed (download EuRoC for full SLAM smoke)."
  exit 0
fi

if ! "$ROOT/scripts/download_euroc.sh" "$SEQ"; then
  record_result "workspace_ok" "Workspace built; EuRoC download failed — run download_euroc.sh manually."
  echo "==> Phase 0 workspace smoke passed; EuRoC download failed (network or mirror)."
  exit 0
fi

EUROC_EXAMPLE="$ORB_DIR/Examples/Stereo-Inertial/stereo_inertial_euroc"
VOCAB="$ORB_DIR/Vocabulary/ORBvoc.txt"
SETTINGS="$ORB_DIR/Examples/Stereo-Inertial/EuRoC.yaml"
SEQ_ROOT="$EUROC_DIR/$SEQ"
TIMES_FILE="$SEQ_ROOT/mav0/cam0/data.csv"

if [[ ! -x "$EUROC_EXAMPLE" ]]; then
  cmake --build "$ORB_DIR/build" -j"$(nproc)" --target stereo_inertial_euroc
fi

FIRST_TS="$(awk -F, 'NF && $1 !~ /^#/ {print $1; exit}' "$TIMES_FILE")"
LAST_TS="$(awk -F, 'NF && $1 !~ /^#/ {ts=$1} END {print ts}' "$TIMES_FILE")"
MAX_SEC="${SMOKE_MAX_SEC:-30}"
END_TS=$((FIRST_TS + MAX_SEC * 1000000000))
if [[ "$END_TS" -gt "$LAST_TS" ]]; then
  END_TS="$LAST_TS"
fi

TIMES_OUT="$(mktemp)"
awk -F, -v start="$FIRST_TS" -v end="$END_TS" '
  NF && $1 !~ /^#/ && $1 >= start && $1 <= end { print $1 }
' "$TIMES_FILE" >"$TIMES_OUT"

echo "==> Running stereo_inertial_euroc on $(wc -l <"$TIMES_OUT") frames (max ${MAX_SEC}s)..."
cd "$ORB_DIR"
"$EUROC_EXAMPLE" "$VOCAB" "$SETTINGS" "$SEQ_ROOT" "$TIMES_OUT"
rm -f "$TIMES_OUT"

record_result "ok" "ORB-SLAM3 stereo-inertial EuRoC smoke completed."
echo "==> Phase 0 smoke passed."
