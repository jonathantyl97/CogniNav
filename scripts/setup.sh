#!/usr/bin/env bash
# One-shot setup: venv, PyTorch cu128, LingBot-Map, LocateAnything-3B weights.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if [[ ! -d .venv ]]; then
  python3.10 -m venv .venv 2>/dev/null || python3 -m venv .venv
fi
source .venv/bin/activate

pip install --upgrade pip
pip install torch==2.8.0 torchvision==0.23.0 --index-url https://download.pytorch.org/whl/cu128

if [[ ! -d third_party/lingbot-map ]]; then
  git clone --depth 1 https://github.com/Robbyant/lingbot-map third_party/lingbot-map
fi
pip install -e "third_party/lingbot-map[vis]"
pip install -r requirements.txt

export HF_HUB_ENABLE_HF_TRANSFER=1
mkdir -p models
if [[ ! -f models/lingbot-map/lingbot-map.pt ]]; then
  hf download robbyant/lingbot-map --local-dir models/lingbot-map
fi
if [[ ! -f models/LocateAnything-3B/config.json ]]; then
  hf download nvidia/LocateAnything-3B --local-dir models/LocateAnything-3B
fi

echo "Setup complete. Activate with: source .venv/bin/activate"
