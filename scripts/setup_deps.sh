#!/usr/bin/env bash
# Build ORB-SLAM3 inside the CogniNav Docker container.
# Visualization: Iridescence (cogninav_viz + pyridescence), NOT Pangolin GUI.
#
# Usage (inside container):
#   cd /root/cogninav && ./scripts/setup_deps.sh

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY="$ROOT/third_party"
ORB_DIR="$THIRD_PARTY/ORB_SLAM3"
ORB_REPO="${ORB_REPO:-https://github.com/UZ-SLAMLab/ORB_SLAM3.git}"
ORB_TAG="${ORB_TAG:-}"

echo "==> CogniNav setup (ORB-SLAM3, headless viewer)"
echo "    Root: $ROOT"

apt-get update
apt-get install -y --no-install-recommends \
  build-essential cmake git wget unzip pkg-config \
  libeigen3-dev libopencv-dev libglew-dev libssl-dev libboost-all-dev \
  libglfw3-dev libglm-dev libpng-dev libjpeg-dev libepoxy-dev \
  python3-pip python3-colcon-common-extensions

mkdir -p "$THIRD_PARTY"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  ORB_PATCH="$ROOT/scripts/patch_orb_humble.sh"
  ROS_DISTRO_DETECTED="humble"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  ORB_PATCH="$ROOT/scripts/patch_orb_jazzy.sh"
  ROS_DISTRO_DETECTED="jazzy"
else
  ORB_PATCH="$ROOT/scripts/patch_orb_jazzy.sh"
  ROS_DISTRO_DETECTED="unknown"
fi
echo "    ROS distro: $ROS_DISTRO_DETECTED"

if [[ ! -d "$ORB_DIR/.git" ]]; then
  echo "==> Cloning ORB-SLAM3..."
  git clone --recursive "$ORB_REPO" "$ORB_DIR"
  if [[ -n "$ORB_TAG" ]]; then
    git -C "$ORB_DIR" checkout "$ORB_TAG"
    git -C "$ORB_DIR" submodule update --init --recursive
  fi
else
  echo "==> ORB-SLAM3 already present at $ORB_DIR"
fi

"$ORB_PATCH" "$ORB_DIR"

# Clean partial builds after patch changes.
rm -rf "$ORB_DIR/build" "$ORB_DIR/Thirdparty/"{DBoW2,g2o,Sophus}/build

# Pangolin: required to link stock ORB-SLAM3; we disable the viewer in the ROS node.
if [[ ! -f /usr/local/lib/libpangolin.so ]] && [[ ! -f /usr/lib/libpangolin.so ]]; then
  echo "==> Building Pangolin (compile-only; Iridescence is the viewer)..."
  PANGOLIN_DIR="/tmp/Pangolin"
  rm -rf "$PANGOLIN_DIR"
  git clone --depth 1 --branch v0.9.2 https://github.com/stevenlovegrove/Pangolin.git "$PANGOLIN_DIR"
  cmake -S "$PANGOLIN_DIR" -B "$PANGOLIN_DIR/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_STANDARD=17
  cmake --build "$PANGOLIN_DIR/build" -j"$(nproc)"
  cmake --install "$PANGOLIN_DIR/build"
fi

echo "==> Building ORB-SLAM3..."
cd "$ORB_DIR"
chmod +x build.sh
set +e
./build.sh
BUILD_RC=$?
set -e

if [[ ! -f "$ORB_DIR/lib/libORB_SLAM3.so" ]]; then
  echo "ERROR: libORB_SLAM3.so not produced (build.sh exit $BUILD_RC)"
  exit 1
fi

if [[ "$BUILD_RC" -ne 0 ]]; then
  echo "==> ORB-SLAM3 examples had errors; building stereo-inertial example for smoke test..."
  SI_TARGET="$(grep -oP 'add_executable\(\Kstereo_inertial\w+' "$ORB_DIR/CMakeLists.txt" | head -1)"
  if [[ -z "$SI_TARGET" ]]; then
    echo "ERROR: could not find stereo-inertial example target in ORB-SLAM3 CMakeLists.txt"
    exit 1
  fi
  cmake --build "$ORB_DIR/build" -j"$(nproc)" --target "$SI_TARGET"
fi

echo "==> Installing Iridescence (pyridescence) + evo..."
PIP_EXTRA=()
if pip3 install --help 2>/dev/null | grep -q break-system-packages; then
  PIP_EXTRA+=(--break-system-packages)
fi
pip3 install "${PIP_EXTRA[@]}" \
  pyridescence \
  evo numpy scipy

echo ""
echo "Done."
echo "  ORB-SLAM3 lib: $ORB_DIR/lib/libORB_SLAM3.so"
echo "  Next: vendor ORB-SLAM3 ROS2 wrapper into ros2_ws/src/cogninav_vslam"
echo "        Set System(..., bUseViewer=false) — see cogninav_vslam/README"
echo "  Viz: ros2 launch cogninav_bringup cogninav.launch.py  (needs DISPLAY / X11)"
