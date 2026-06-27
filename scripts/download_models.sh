#!/usr/bin/env bash
# Download lightweight perception models for cogninav_lanes (MobileNet-SSD).
#
# Prototxt is vendored in-repo; weights are optional (Google Drive may block gdown).
# Lane detection works without weights when enable_object_detection:=false.
#
# Usage: ./scripts/download_models.sh

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL_DIR="$ROOT/models"
mkdir -p "$MODEL_DIR"

PROTO="$MODEL_DIR/MobileNetSSD_deploy.prototxt"
WEIGHTS="$MODEL_DIR/MobileNetSSD_deploy.caffemodel"
PROTO_URL="https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/voc/MobileNetSSD_deploy.prototxt"
WEIGHTS_ID="0B3gersZ2cHIxRm5PMWRoTkdHdHc"

if [[ -f "$PROTO" && -s "$PROTO" && -f "$WEIGHTS" && -s "$WEIGHTS" ]]; then
  echo "MobileNet-SSD already present in $MODEL_DIR"
  exit 0
fi

echo "==> MobileNet-SSD models for cogninav_lanes..."

if [[ ! -f "$PROTO" || ! -s "$PROTO" ]]; then
  wget -q -O "$PROTO" "$PROTO_URL"
fi

if [[ ! -f "$WEIGHTS" || ! -s "$WEIGHTS" ]]; then
  if ! python3 -c "import gdown" 2>/dev/null; then
    PIP_EXTRA=()
    if pip3 install --help 2>/dev/null | grep -q break-system-packages; then
      PIP_EXTRA+=(--break-system-packages)
    fi
    pip3 install "${PIP_EXTRA[@]}" gdown >/dev/null 2>&1 || true
  fi
  if python3 -c "import gdown" 2>/dev/null; then
    python3 - "$WEIGHTS" "$WEIGHTS_ID" 2>/dev/null <<'PY' || true
import sys
import gdown
gdown.download(id=sys.argv[2], output=sys.argv[1], quiet=True)
PY
  fi
  if [[ ! -s "$WEIGHTS" ]]; then
    echo "NOTE: MobileNet-SSD weights skipped (optional; lane detection still runs)."
    rm -f "$WEIGHTS"
  fi
fi

echo "Done: $MODEL_DIR (prototxt ready; weights optional)"
