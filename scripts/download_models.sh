#!/usr/bin/env bash
# Download lightweight perception models for cogninav_lanes (MobileNet-SSD).
#
# Usage: ./scripts/download_models.sh

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL_DIR="$ROOT/models"
mkdir -p "$MODEL_DIR"

PROTO="$MODEL_DIR/MobileNetSSD_deploy.prototxt"
WEIGHTS="$MODEL_DIR/MobileNetSSD_deploy.caffemodel"

if [[ -f "$PROTO" && -f "$WEIGHTS" ]]; then
  echo "MobileNet-SSD already present in $MODEL_DIR"
  exit 0
fi

echo "==> Downloading MobileNet-SSD (OpenCV DNN, VOC classes)..."
wget -q -O "$PROTO" \
  "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/MobileNetSSD_deploy.prototxt"
wget -q -O "$WEIGHTS" \
  "https://github.com/chuanqi305/MobileNet-SSD/raw/master/MobileNetSSD_deploy.caffemodel"

echo "Done: $MODEL_DIR"
