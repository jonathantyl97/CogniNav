#!/usr/bin/env bash
# Persistent ROS 2 Humble dev container for CogniNav (parity testing).

set -euo pipefail

CONTAINER_NAME="ros2_humble_cogninav"
COGNINAV_DIR="${COGNINAV_DIR:-$HOME/OSS/CogniNav}"
DOWNLOADS_DIR="${DOWNLOADS_DIR:-$HOME/Downloads}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-100}"
IMAGE="${COGNINAV_HUMBLE_IMAGE:-osrf/ros:humble-desktop}"

XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"
XAUTH_ENV=()
XAUTH_RUN=()
if [[ -f "$XAUTH" ]]; then
  XAUTH_ENV=(-e "XAUTHORITY=/tmp/.docker.xauth")
  XAUTH_RUN=(-v "$XAUTH:/tmp/.docker.xauth:ro" -e "XAUTHORITY=/tmp/.docker.xauth")
fi

echo "Configuring X11 forwarding..."
xhost +local:docker >/dev/null 2>&1 || xhost +local:root >/dev/null 2>&1 || true

mkdir -p "$COGNINAV_DIR" "$DOWNLOADS_DIR"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "Starting existing container '$CONTAINER_NAME'..."
  docker start "$CONTAINER_NAME" >/dev/null
  if [[ -f "$XAUTH" ]]; then
    docker cp "$XAUTH" "$CONTAINER_NAME:/tmp/.docker.xauth" >/dev/null 2>&1 || true
  fi
  docker exec -it \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/cogninav-runtime \
    "${XAUTH_ENV[@]}" \
    "$CONTAINER_NAME" bash -lc 'mkdir -p /tmp/cogninav-runtime && chmod 700 /tmp/cogninav-runtime; source /opt/ros/humble/setup.bash; cd /root/cogninav; exec bash'
else
  echo "Creating container '$CONTAINER_NAME' from $IMAGE ..."
  docker run -it \
    --name "$CONTAINER_NAME" \
    --net=host \
    --gpus all \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/cogninav-runtime \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID_VALUE" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${XAUTH_RUN[@]}" \
    -v "$COGNINAV_DIR:/root/cogninav" \
    -v "$DOWNLOADS_DIR:/root/Downloads" \
    "$IMAGE" \
    bash -lc 'grep -q cogninav /root/.bashrc || {
      echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
      echo "export ROS_DOMAIN_ID='"$ROS_DOMAIN_ID_VALUE"'" >> /root/.bashrc
      echo "cd /root/cogninav" >> /root/.bashrc
    }; source /opt/ros/humble/setup.bash; cd /root/cogninav; exec bash'
fi
