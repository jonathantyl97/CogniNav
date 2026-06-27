#!/usr/bin/env bash
# Re-run a CogniNav benchmark script inside the Jazzy Docker container when
# ROS 2 is not available on the host (Downloads are mounted at /root/Downloads).

#!/usr/bin/env bash
# Re-run a CogniNav benchmark script inside the Jazzy Docker container when
# ROS 2 is not available on the host (Downloads are mounted at /root/Downloads).

cogninav_prepare_docker_x11() {
  local container="${1:-${COGNINAV_CONTAINER:-ros2_jazzy_cogninav}}"
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: DISPLAY is not set on the host (required for Iridescence)."
    echo "Start a local X session, then run: ./docker/cogninav_jazzy.sh"
    exit 1
  fi
  xhost +local:docker >/dev/null 2>&1 || xhost +local:root >/dev/null 2>&1 || true
  docker start "$container" >/dev/null 2>&1 || true
  local xauth="${XAUTHORITY:-$HOME/.Xauthority}"
  if [[ -f "$xauth" ]]; then
    docker cp "$xauth" "$container:/tmp/.docker.xauth" >/dev/null 2>&1 || true
  fi
}

cogninav_reexec_in_docker() {
  local script_rel="$1"
  shift

  if [[ -n "${COGNINAV_IN_HUMBLE:-}" || -f /opt/ros/humble/setup.bash ]]; then
    return 0
  fi
  if [[ -n "${COGNINAV_IN_DOCKER:-}" || -f /opt/ros/jazzy/setup.bash ]]; then
    return 0
  fi

  local container="${COGNINAV_CONTAINER:-ros2_jazzy_cogninav}"
  if ! docker ps -a --format '{{.Names}}' | grep -qx "$container"; then
    echo "ERROR: ROS 2 Jazzy not found and container '$container' does not exist."
    echo "Start it with: ./docker/cogninav_jazzy.sh"
    exit 1
  fi

  local quoted=""
  local arg
  for arg in "$@"; do
    quoted+=$(printf '%q' "$arg")" "
  done

  local inner_env="export COGNINAV_IN_DOCKER=1"
  inner_env+="; export RMW_FASTRTPS_USE_SHM=0"

  if [[ "${COGNINAV_DOCKER_X11:-0}" == "1" ]]; then
    cogninav_prepare_docker_x11 "$container"
    inner_env+="; export DISPLAY=${DISPLAY:-}"
    inner_env+="; export QT_X11_NO_MITSHM=1"
    inner_env+="; export XDG_RUNTIME_DIR=/tmp/cogninav-runtime"
    inner_env+="; export XAUTHORITY=/tmp/.docker.xauth"
    inner_env+="; mkdir -p /tmp/cogninav-runtime && chmod 700 /tmp/cogninav-runtime"
    exec docker exec -it \
      -e DISPLAY="${DISPLAY:-}" \
      -e QT_X11_NO_MITSHM=1 \
      -e XDG_RUNTIME_DIR=/tmp/cogninav-runtime \
      -e XAUTHORITY=/tmp/.docker.xauth \
      -e RMW_FASTRTPS_USE_SHM=0 \
      "$container" bash -lc \
      "${inner_env}; cd /root/cogninav && ./${script_rel} ${quoted}"
  fi

  docker start "$container" >/dev/null 2>&1 || true
  exec docker exec "$container" bash -lc \
    "${inner_env}; cd /root/cogninav && ./${script_rel} ${quoted}"
}

cogninav_downloads_dir() {
  echo "${DOWNLOADS_DIR:-${HOME}/Downloads}"
}

cogninav_reexec_in_humble() {
  local script_rel="$1"
  shift

  if [[ -n "${COGNINAV_IN_HUMBLE:-}" || -f /opt/ros/humble/setup.bash ]]; then
    return 0
  fi

  local container="${COGNINAV_HUMBLE_CONTAINER:-ros2_humble_cogninav}"
  if ! docker ps -a --format '{{.Names}}' | grep -qx "$container"; then
    echo "ERROR: ROS 2 Humble not found and container '$container' does not exist."
    echo "Create it with: ./docker/cogninav_humble.sh"
    exit 1
  fi

  docker start "$container" >/dev/null 2>&1 || true
  local quoted=""
  local arg
  for arg in "$@"; do
    quoted+=$(printf '%q' "$arg")" "
  done
  exec docker exec "$container" bash -lc \
    "export COGNINAV_IN_HUMBLE=1; cd /root/cogninav && ./${script_rel} ${quoted}"
}

cogninav_ros_setup() {
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
  else
    echo "ERROR: no ROS 2 setup.bash found"
    exit 1
  fi
}

cogninav_ros_distro() {
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    echo humble
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    echo jazzy
  else
    echo unknown
  fi
}

cogninav_docker_image() {
  case "$(cogninav_ros_distro)" in
    humble) echo "${COGNINAV_HUMBLE_IMAGE:-osrf/ros:humble-desktop}" ;;
    jazzy) echo "${COGNINAV_JAZZY_IMAGE:-osrf/ros:jazzy-desktop-full}" ;;
    *) echo "unknown" ;;
  esac
}

# Resolve a ROS 2 bag directory for the active distro. On Humble, Jazzy/rosbags
# metadata is converted to a *_ros2_humble copy when needed.
cogninav_resolve_ros2_bag() {
  local seq="$1"
  local parent_dir="$2"
  local root="${3:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

  local distro bag_distro bag_legacy play_bag
  distro="$(cogninav_ros_distro)"
  bag_distro="$parent_dir/${seq}_ros2_${distro}"
  bag_legacy="$parent_dir/${seq}_ros2"

  if [[ -d "$bag_distro" ]]; then
    play_bag="$bag_distro"
  elif [[ -d "$bag_legacy" ]]; then
    if [[ "$distro" == "humble" ]]; then
      echo "==> Preparing Humble-compatible ROS 2 bag: $bag_distro" >&2
      python3 "$root/scripts/sanitize_ros2_bag_for_humble.py" \
        "$bag_legacy" --out "$bag_distro" >&2
      play_bag="$bag_distro"
    else
      play_bag="$bag_legacy"
    fi
  else
    return 1
  fi

  printf '%s' "$play_bag"
}
