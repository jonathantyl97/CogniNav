#!/usr/bin/env bash
# LingBot-Map side-by-side demo: indoor drone flight + live point-cloud map.
# Downloads the indoor_travel demo video from the lingbot-map demo dataset on
# first run (~5.6 GB). The raw input is gitignored; only the output MP4/GIF
# are tracked in assets/.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source .venv/bin/activate

export HF_HUB_ENABLE_HF_TRANSFER=1
DEMO_DIR="${DEMO_DIR:-$HOME/Downloads/lingbot_demo}"
VIDEO="$DEMO_DIR/indoor_travel.MP4"

if [[ ! -f "$VIDEO" ]]; then
  echo "Downloading indoor drone demo video (~5.6 GB) ..."
  mkdir -p "$DEMO_DIR"
  python - <<'PY'
from huggingface_hub import hf_hub_download
import os
out = hf_hub_download(
    repo_id="robbyant/lingbot-map-demo",
    repo_type="dataset",
    filename="indoor_travel.MP4",
    local_dir=os.environ["DEMO_DIR"],
)
print("downloaded", out)
PY
fi

python -m cogninav.slam_sidebyside \
  --video "$VIDEO" \
  --output assets/slam_indoor_drone.mp4 \
  --fps 5

echo "Indoor drone side-by-side SLAM video: assets/slam_indoor_drone.mp4"
