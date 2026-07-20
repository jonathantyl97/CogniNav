#!/usr/bin/env bash
# Warehouse demo: NVIDIA r2b_storage ROS 2 bag (downloads ~2.9 GB on first run).
# No ROS installation required — the bag is decoded with pure-Python rosbags.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
source .venv/bin/activate

BAG="${BAG:-$HOME/Downloads/warehouse/r2b_storage}"

if [[ ! -f "$BAG/metadata.yaml" ]]; then
  mkdir -p "$BAG"
  BASE=https://api.ngc.nvidia.com/v2/resources/nvidia/isaac/r2bdataset2023/versions/3/files/r2b_storage
  wget -O "$BAG/metadata.yaml" "$BASE/metadata.yaml"
  wget -O "$BAG/r2b_storage_0.db3" "$BASE/r2b_storage_0.db3"
fi

python -m cogninav.pipeline \
  --bag "$BAG" \
  --bag_topic d455_1_rgb_image \
  --mode warehouse \
  --categories "person,forklift,cart" \
  --det_every 4 \
  --output_dir outputs/warehouse "$@"
