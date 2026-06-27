#!/usr/bin/env bash
# Ubuntu 22.04 / ROS Humble patches for stock ORB-SLAM3 (GCC 11, OpenCV 4.5, Eigen 3.4).
set -euo pipefail

ORB_DIR="${1:?ORB_SLAM3 root required}"

echo "==> Patching ORB-SLAM3 for Humble / Ubuntu 22.04: $ORB_DIR"

SOPHUS_CMAKE="$ORB_DIR/Thirdparty/Sophus/CMakeLists.txt"
if [[ -f "$SOPHUS_CMAKE" ]] && ! grep -q 'Wno-error=array-bounds' "$SOPHUS_CMAKE"; then
  sed -i 's/-ftemplate-backtrace-limit=0/-ftemplate-backtrace-limit=0 -Wno-error=array-bounds/' "$SOPHUS_CMAKE"
fi
sed -i 's/find_package(Eigen3 3.3.0 REQUIRED)/find_package(Eigen3 REQUIRED)/' "$SOPHUS_CMAKE" 2>/dev/null || true

ORB_CMAKE="$ORB_DIR/CMakeLists.txt"
sed -i 's/find_package(OpenCV [0-9.]*)/find_package(OpenCV 4.5)/' "$ORB_CMAKE"
sed -i 's/find_package(Eigen3 3.1.0 REQUIRED)/find_package(Eigen3 REQUIRED)/' "$ORB_CMAKE"
sed -i 's/-std=c++11/-std=c++17/g' "$ORB_CMAKE"
sed -i 's/-std=c++14/-std=c++17/g' "$ORB_CMAKE"
sed -i 's/COMPILEDWITHC11/COMPILEDWITHC17/g' "$ORB_CMAKE"
sed -i 's/COMPILEDWITHC14/COMPILEDWITHC17/g' "$ORB_CMAKE"

LOOP_H="$ORB_DIR/include/LoopClosing.h"
if [[ -f "$LOOP_H" ]] && grep -q 'bool mnFullBAIdx' "$LOOP_H"; then
  sed -i 's/bool mnFullBAIdx;/int mnFullBAIdx;/' "$LOOP_H"
fi

DBOW2_CMAKE="$ORB_DIR/Thirdparty/DBoW2/CMakeLists.txt"
if [[ -f "$DBOW2_CMAKE" ]]; then
  sed -i 's/find_package(OpenCV [0-9.]* QUIET)/find_package(OpenCV 4.5 QUIET)/' "$DBOW2_CMAKE" 2>/dev/null || true
fi

BUILD_SH="$ORB_DIR/build.sh"
if [[ -f "$BUILD_SH" ]] && ! grep -q 'BUILD_TESTS=OFF' "$BUILD_SH"; then
  sed -i 's|cmake .. -DCMAKE_BUILD_TYPE=Release|cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF|g' "$BUILD_SH"
fi

find "$ORB_DIR/Examples" -name '*.cc' -exec sed -i \
  's/std::chrono::monotonic_clock/std::chrono::steady_clock/g' {} + 2>/dev/null || true

echo "==> ORB-SLAM3 Humble patches applied."
