#!/usr/bin/env bash
# Download a EuRoC MAV sequence into ~/Downloads (mounted at /root/Downloads in Docker).
#
# Usage:
#   ./scripts/download_euroc.sh MH_01_easy
#
# Requires: wget, unzip. Files land in ${EUROC_DIR:-$HOME/Downloads/euroc}/<seq>/mav0.

set -euo pipefail

SEQ="${1:-MH_01_easy}"
EUROC_DIR="${EUROC_DIR:-${HOME}/Downloads/euroc}"
SEQ_DIR="$EUROC_DIR/$SEQ"
MAV0="$SEQ_DIR/mav0"

if [[ -d "$MAV0/cam0/data" ]]; then
  echo "EuRoC sequence ready: $MAV0"
  exit 0
fi

mkdir -p "$EUROC_DIR"
cd "$EUROC_DIR"

ZIP="${SEQ}.zip"
URLS=(
  "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/${SEQ}/${SEQ}.zip"
  "http://robotics.ethz.ch/~mavryang/datasets/ashlagonian/EuRoC/${SEQ}.zip"
)

if [[ ! -f "$ZIP" ]]; then
  echo "Downloading EuRoC ${SEQ}..."
  downloaded=false
  for url in "${URLS[@]}"; do
    echo "  trying $url"
    if wget -c --timeout=30 --tries=2 "$url" -O "$ZIP"; then
      downloaded=true
      break
    fi
    rm -f "$ZIP"
  done
  if [[ "$downloaded" != true ]]; then
    echo "ERROR: could not download ${SEQ}. Fetch manually into $SEQ_DIR (need mav0/cam0/data)."
    exit 1
  fi
fi

mkdir -p "$SEQ_DIR"
unzip -o "$ZIP" -d "$SEQ_DIR"

# ASL zip often extracts mav0 at archive root.
if [[ -d "$EUROC_DIR/mav0" && ! -d "$MAV0" ]]; then
  mv "$EUROC_DIR/mav0" "$SEQ_DIR/"
fi
if [[ -d "$SEQ_DIR/${SEQ}/mav0" ]]; then
  mv "$SEQ_DIR/${SEQ}/mav0" "$SEQ_DIR/"
  rmdir "$SEQ_DIR/${SEQ}" 2>/dev/null || true
fi

if [[ ! -d "$MAV0/cam0/data" ]]; then
  echo "ERROR: expected $MAV0/cam0/data after unzip"
  exit 1
fi

echo "EuRoC data in $SEQ_DIR"
