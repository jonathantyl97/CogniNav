#!/usr/bin/env bash
# Download a TUM-VI calibrated ROS bag (stereo + IMU).
#
# Usage:
#   ./scripts/download_tumvi.sh
#   ./scripts/download_tumvi.sh dataset-room1_512_16
#   ./scripts/download_tumvi.sh room1          # shorthand -> dataset-room1_512_16
#
# Official layout (2024+):
#   https://cvg.cit.tum.de/tumvi/calibrated/<res>_<imu>/<seq>.bag
# Mirrors via cdn*.vision.in.tum.de (wget follows redirects).

set -euo pipefail

SEQ="${1:-dataset-room1_512_16}"
TUMVI_DIR="${TUMVI_DIR:-${HOME}/Downloads/tumvi}"
mkdir -p "$TUMVI_DIR"
cd "$TUMVI_DIR"

# Shorthand: room1 -> dataset-room1_512_16
if [[ "$SEQ" =~ ^room[0-9]+$ ]]; then
  SEQ="dataset-${SEQ}_512_16"
fi

BAG="${SEQ}.bag"
if [[ -f "$BAG" || -d "${SEQ}_ros2" ]]; then
  echo "TUM-VI data ready under $TUMVI_DIR"
  exit 0
fi

tumvi_bag_urls() {
  local seq="$1"
  local folder="512_16"
  if [[ "$seq" =~ _([0-9]+)_([0-9]+)$ ]]; then
    folder="${BASH_REMATCH[1]}_${BASH_REMATCH[2]}"
  fi
  echo "https://cvg.cit.tum.de/tumvi/calibrated/${folder}/${seq}.bag"
  echo "https://vision.in.tum.de/tumvi/calibrated/${folder}/${seq}.bag"
}

echo "Downloading TUM-VI ${SEQ}..."
while IFS= read -r url; do
  echo "  trying $url"
  if wget -c --timeout=120 --tries=2 --content-disposition "$url" -O "$BAG"; then
    echo "Downloaded $TUMVI_DIR/$BAG"
    exit 0
  fi
  rm -f "$BAG"
done < <(tumvi_bag_urls "$SEQ")

echo "ERROR: could not download ${SEQ}.bag"
echo "See https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset"
exit 1
