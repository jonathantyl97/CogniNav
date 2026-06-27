#!/usr/bin/env bash
# Download KITTI odometry poses + one stereo sequence for CogniNav (Phase 2).
#
# Usage:
#   ./scripts/download_kitti.sh        # sequence 00
#   ./scripts/download_kitti.sh 04
#
# Requires ~2.3 GB for sequence 00 gray stereo (single-sequence archive).
# Poses zip is small and downloaded first.

set -euo pipefail

SEQ="${1:-00}"
SEQ="$(printf '%02d' "$((10#$SEQ))")"
KITTI_DIR="${KITTI_DIR:-${HOME}/Downloads/kitti}"
SEQ_DIR="$KITTI_DIR/sequences/$SEQ"

mkdir -p "$KITTI_DIR"
cd "$KITTI_DIR"

if [[ -d "$SEQ_DIR/image_0" && -f "$SEQ_DIR/times.txt" ]]; then
  echo "KITTI sequence ready: $SEQ_DIR"
  exit 0
fi

POSES_ZIP="data_odometry_poses.zip"
if [[ ! -f "$POSES_ZIP" ]]; then
  echo "Downloading KITTI poses..."
  wget -c --timeout=60 "https://s3.eu-central-1.amazonaws.com/avg-kitti/${POSES_ZIP}" -O "$POSES_ZIP"
  unzip -o "$POSES_ZIP"
fi

SEQ_ARCHIVE="data_odometry_gray_velodyne_${SEQ}.zip"
SEQ_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/${SEQ_ARCHIVE}"

if [[ ! -f "$SEQ_ARCHIVE" ]]; then
  echo "Downloading KITTI gray stereo sequence ${SEQ} (~2.3 GB)..."
  if ! wget -c --timeout=120 --tries=2 "$SEQ_URL" -O "$SEQ_ARCHIVE"; then
    echo "WARN: per-sequence archive unavailable."
    echo "Download manually from https://www.cvlibs.net/datasets/kitti/eval_odometry.php"
    echo "  data_odometry_gray_velodyne.zip (all sequences, ~22 GB)"
    echo "Then extract sequences/${SEQ}/ into $KITTI_DIR/sequences/"
    exit 1
  fi
fi

mkdir -p sequences
unzip -o "$SEQ_ARCHIVE" -d sequences
if [[ -d "sequences/sequences/${SEQ}" ]]; then
  mv "sequences/sequences/${SEQ}" "sequences/${SEQ}"
  rmdir sequences/sequences 2>/dev/null || true
fi

if [[ ! -d "$SEQ_DIR/image_0" ]]; then
  echo "ERROR: expected $SEQ_DIR/image_0 after extract"
  exit 1
fi

echo "KITTI sequence ${SEQ} in $SEQ_DIR"
