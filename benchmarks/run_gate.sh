#!/usr/bin/env bash
# CogniNav validation gates (workspace, SLAM trajectory, full stack on bag replay).
#
# Usage:
#   ./benchmarks/run_gate.sh --all [--skip-humble]
#   ./benchmarks/run_gate.sh --humble --workspace
#   ./benchmarks/run_gate.sh --workspace
#   ./benchmarks/run_gate.sh --slam --source r2b
#   ./benchmarks/run_gate.sh --stack --source r2b
#   ./benchmarks/run_gate.sh --stack --source kitti --seq 00

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=benchmarks/cogninav_docker.sh
source "$ROOT/benchmarks/cogninav_docker.sh"

if [[ "${1:-}" == "--humble" ]]; then
  shift
  cogninav_reexec_in_humble "benchmarks/run_gate.sh" "$@"
fi

cogninav_reexec_in_docker "benchmarks/run_gate.sh" "$@"

SOURCE="r2b"
SEQ="aisle_cw_run_1"
RATE="1.0"
MODE_WORKSPACE=false
MODE_SLAM=false
MODE_STACK=false
MODE_ALL=false
SKIP_HUMBLE=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace) MODE_WORKSPACE=true; shift ;;
    --slam) MODE_SLAM=true; shift ;;
    --stack) MODE_STACK=true; shift ;;
    --all) MODE_ALL=true; shift ;;
    --skip-humble) SKIP_HUMBLE=true; shift ;;
    --source) SOURCE="$2"; shift 2 ;;
    --seq) SEQ="$2"; shift 2 ;;
    --rate) RATE="$2"; shift 2 ;;
    *) echo "Unknown arg: $1"; exit 1 ;;
  esac
done

if [[ "$MODE_ALL" == true ]]; then
  MODE_WORKSPACE=true
  MODE_SLAM=true
  MODE_STACK=true
fi
if [[ "$MODE_WORKSPACE" != true && "$MODE_SLAM" != true && "$MODE_STACK" != true ]]; then
  MODE_ALL=true
  MODE_WORKSPACE=true
  MODE_SLAM=true
  MODE_STACK=true
fi

ORB_LIB="$ROOT/third_party/ORB_SLAM3/lib/libORB_SLAM3.so"
GIT_SHA="$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null || echo unknown)"
DOCKER_IMAGE="$(cogninav_docker_image)"

record_result() {
  local dataset="$1" seq="$2" phase="$3" status="$4" note="$5"
  "$ROOT/benchmarks/run_benchmark.sh" \
    --dataset "$dataset" --seq "$seq" --phase "$phase" \
    --git-sha "$GIT_SHA" --docker-image "$DOCKER_IMAGE" \
    --smoke-status "$status" --smoke-note "$note"
}

gate_workspace() {
  echo "==> Gate: workspace"
  if [[ ! -f "$ORB_LIB" ]]; then
    echo "Missing $ORB_LIB — run ./docker/setup_deps.sh first"
    exit 1
  fi
  set +u
  cogninav_ros_setup
  set -u
  cd "$ROOT/ros2_ws"
  colcon build
  set +u
  source install/setup.bash
  set -u
  record_result "workspace" "build" 0 "workspace_ok" \
    "colcon build + libORB_SLAM3.so verified."
  echo "==> Workspace gate passed."
}

resolve_bag() {
  case "$SOURCE" in
    r2b)
      PLAY_BAG="$(cogninav_downloads_dir)/warehouse/r2b_storage"
      LAUNCH="r2b_storage.launch.py"
      DATASET="warehouse_r2b"
      SEQ="r2b_storage"
      TRAJ="/tmp/cogninav_r2b_trajectory.txt"
      BAG_LOOP="true"
      SLAM_TIMEOUT="${SLAM_TIMEOUT_SEC:-120}"
      STACK_TOPICS=(
        /cogninav/odom
        /cogninav/map_points
        /cogninav/stereo_points
        /cogninav/lane_markers
        /cogninav/aisle_guidance
        /cogninav/dynamic_detections
        /cogninav/dynamic_mask
        /cogninav/slam_mask_stats
      )
      ;;
    torwic)
      PLAY_BAG="$(cogninav_downloads_dir)/warehouse/${SEQ}_ros2"
      LAUNCH="warehouse.launch.py"
      DATASET="warehouse_torwic"
      TRAJ="/tmp/cogninav_warehouse_trajectory.txt"
      BAG_LOOP="true"
      SLAM_TIMEOUT="${SLAM_TIMEOUT_SEC:-120}"
      STACK_TOPICS=(
        /cogninav/odom
        /cogninav/map_points
        /cogninav/lane_markers
        /cogninav/aisle_guidance
        /cogninav/dynamic_detections
        /cogninav/dynamic_mask
        /cogninav/slam_mask_stats
      )
      ;;
    kitti)
      SEQ="$(printf '%02d' "$((10#$SEQ))")"
      PLAY_BAG="$(cogninav_downloads_dir)/kitti/${SEQ}_ros2"
      LAUNCH="kitti.launch.py"
      DATASET="kitti_odometry"
      TRAJ="/tmp/cogninav_kitti_trajectory.txt"
      BAG_LOOP="true"
      SLAM_TIMEOUT="${SLAM_TIMEOUT_SEC:-120}"
      STACK_TOPICS=(
        /cogninav/odom
        /cogninav/map_points
        /cogninav/stereo_points
        /cogninav/lane_markers
        /cogninav/aisle_guidance
        /cogninav/dynamic_detections
        /cogninav/dynamic_mask
        /cogninav/slam_mask_stats
      )
      ;;
    *)
      echo "Unknown source: $SOURCE (use r2b, torwic, or kitti)"
      exit 1
      ;;
  esac

  if [[ ! -d "$PLAY_BAG" ]]; then
    echo "Missing bag $PLAY_BAG — see README.md (Datasets)"
    exit 1
  fi
}

gate_slam() {
  echo "==> Gate: SLAM trajectory ($SOURCE)"
  resolve_bag
  set +u
  cogninav_ros_setup
  set -u
  cd "$ROOT/ros2_ws"
  colcon build --packages-select cogninav_vslam cogninav_bringup cogninav_viz
  set +u
  source install/setup.bash
  set -u

  rm -f "$TRAJ"
  export LD_LIBRARY_PATH="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"
  set +e
  timeout --signal=INT "$SLAM_TIMEOUT" ros2 launch cogninav_bringup "$LAUNCH" \
    bag_path:="$PLAY_BAG" rate:="$RATE" bag_play_delay:=12.0 bag_loop:="$BAG_LOOP" \
    use_viz:=false use_vslam:=true use_depth:=false use_lanes:=false
  set -e
  sleep 2

  if [[ ! -f "$TRAJ" ]]; then
    record_result "$DATASET" "$SEQ" 1 "fail" "SLAM trajectory not saved."
    echo "ERROR: trajectory not saved at $TRAJ"
    exit 1
  fi
  local lines
  lines=$(wc -l <"$TRAJ")
  record_result "$DATASET" "$SEQ" 1 "ok" \
    "SLAM trajectory saved ($lines poses) on $SOURCE."
  echo "==> SLAM gate passed ($SOURCE / $SEQ, $lines poses)."
}

gate_stack() {
  echo "==> Gate: full stack ($SOURCE)"
  resolve_bag
  set +u
  cogninav_ros_setup
  set -u
  cd "$ROOT/ros2_ws"
  colcon build --packages-select \
    cogninav_vslam cogninav_depth cogninav_lanes cogninav_bringup cogninav_viz
  set +u
  source install/setup.bash
  set -u
  export LD_LIBRARY_PATH="/root/cogninav/third_party/ORB_SLAM3/lib:/usr/local/lib:${LD_LIBRARY_PATH:-}"

  local probe_timeout="${GATE_PROBE_TIMEOUT:-120}"
  local launch_pid=""
  cleanup() { [[ -n "$launch_pid" ]] && kill "$launch_pid" 2>/dev/null || true; }
  trap cleanup EXIT

  ros2 launch cogninav_bringup "$LAUNCH" \
    bag_path:="$PLAY_BAG" rate:="$RATE" bag_play_delay:=12.0 bag_loop:="$BAG_LOOP" \
    use_viz:=false use_vslam:=true use_depth:=true use_lanes:=true &
  launch_pid=$!

  sleep 20
  set +e
  "$ROOT/benchmarks/wait_for_topics.sh" --timeout "$probe_timeout" "${STACK_TOPICS[@]}"
  local probe_rc=$?
  set -e

  kill -INT "$launch_pid" 2>/dev/null || true
  sleep 2
  kill "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  launch_pid=""
  trap - EXIT

  if [[ "$probe_rc" -eq 0 ]]; then
    record_result "$DATASET" "$SEQ" 2 "ok" \
      "Full stack topics OK ($SOURCE): ${STACK_TOPICS[*]}"
    echo "==> Stack gate passed ($SOURCE / $SEQ)."
  else
    record_result "$DATASET" "$SEQ" 2 "fail" "Stack topic probe failed ($SOURCE)."
    echo "ERROR: stack gate failed"
    exit 1
  fi
}

gate_humble_workspace() {
  if [[ "$SKIP_HUMBLE" == true ]]; then
    return 0
  fi
  local container="${COGNINAV_HUMBLE_CONTAINER:-ros2_humble_cogninav}"
  if ! docker ps -a --format '{{.Names}}' | grep -qx "$container"; then
    return 0
  fi
  echo "==> Gate: Humble workspace (optional)"
  docker start "$container" >/dev/null 2>&1 || true
  docker exec "$container" bash -lc \
    "export COGNINAV_IN_HUMBLE=1; cd /root/cogninav && ./benchmarks/run_gate.sh --workspace" || true
}

[[ "$MODE_WORKSPACE" == true ]] && gate_workspace
[[ "$MODE_SLAM" == true ]] && gate_slam
[[ "$MODE_STACK" == true ]] && gate_stack
[[ "$MODE_ALL" == true ]] && gate_humble_workspace

echo "==> Gate(s) complete."
