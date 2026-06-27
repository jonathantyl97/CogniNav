#!/usr/bin/env bash
# Re-run a CogniNav benchmark script inside the Jazzy Docker container when
# ROS 2 is not available on the host (Downloads are mounted at /root/Downloads).

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

  docker start "$container" >/dev/null 2>&1 || true
  local quoted=""
  local arg
  for arg in "$@"; do
    quoted+=$(printf '%q' "$arg")" "
  done
  exec docker exec "$container" bash -lc \
    "export COGNINAV_IN_DOCKER=1; cd /root/cogninav && ./${script_rel} ${quoted}"
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
