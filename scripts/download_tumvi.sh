#!/usr/bin/env bash
# Download a TUM-VI eval ROS bag (stereo + IMU).
#
# Usage:
#   ./scripts/download_tumvi.sh
#   ./scripts/download_tumvi.sh dataset-room1_512_16
#
# Files land in ${TUMVI_DIR:-$HOME/Downloads/tumvi}/

set -euo pipefail

SEQ="${1:-dataset-room1_512_16}"
TUMVI_DIR="${TUMVI_DIR:-${HOME}/Downloads/tumvi}"
mkdir -p "$TUMVI_DIR"
cd "$TUMVI_DIR"

BAG="${SEQ}.bag"
if [[ -f "$BAG" || -d "${SEQ}_ros2" ]]; then
  echo "TUM-VI data ready under $TUMVI_DIR"
  exit 0
fi

URLS=(
  "https://vision.in.tum.de/data/datasets/visual-inertial-dataset/eval/${SEQ}.bag"
  "https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset/eval/${SEQ}.bag"
)

echo "Downloading TUM-VI ${SEQ}..."
for url in "${URLS[@]}"; do
  echo "  trying $url"
  if wget -c --timeout=60 --tries=2 "$url" -O "$BAG"; then
    echo "Downloaded $TUMVI_DIR/$BAG"
    exit 0
  fi
  rm -f "$BAG"
done

echo "ERROR: could not download ${SEQ}.bag"
echo "Fetch manually from https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset"
exit 1
