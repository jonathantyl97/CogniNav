#!/usr/bin/env bash
# Road demo: KITTI raw city drive (downloads ~640 MB on first run).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source .venv/bin/activate

KITTI_DIR="${KITTI_DIR:-$HOME/Downloads/kitti_raw}"
DRIVE="2011_09_26_drive_0005_sync"
FRAMES="$KITTI_DIR/2011_09_26/$DRIVE/image_02/data"

if [[ ! -d "$FRAMES" ]]; then
  mkdir -p "$KITTI_DIR"
  cd "$KITTI_DIR"
  wget -c "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0005/${DRIVE}.zip"
  unzip -q "${DRIVE}.zip"
  cd "$ROOT"
fi

python -m cogninav.pipeline \
  --image_folder "$FRAMES" \
  --mode road \
  --categories "car,person" \
  --det_every 3 \
  --output_dir outputs/road "$@"
