#!/usr/bin/env bash
# Convert KITTI odometry sequence to ROS 2 bag (stereo gray, no IMU).
#
# Usage:
#   ./scripts/bag_from_kitti.sh 00
#
# Input:  ${KITTI_DIR}/sequences/<seq>/{image_0,image_1,times.txt}
# Output: ${KITTI_DIR}/<seq>_ros2/

set -euo pipefail

SEQ="${1:-00}"
SEQ="$(printf '%02d' "$((10#$SEQ))")"
KITTI_DIR="${KITTI_DIR:-${HOME}/Downloads/kitti}"
SEQ_DIR="$KITTI_DIR/sequences/$SEQ"
OUT_BAG="$KITTI_DIR/${SEQ}_ros2"

if [[ ! -d "$SEQ_DIR/image_0" ]]; then
  echo "Missing KITTI sequence at $SEQ_DIR"
  echo "Run: ./scripts/download_kitti.sh $SEQ"
  exit 1
fi

if [[ -d "$OUT_BAG" ]]; then
  echo "Bag already exists: $OUT_BAG"
  exit 0
fi

if ! command -v ros2 >/dev/null; then
  echo "ros2 not found — source /opt/ros/jazzy/setup.bash"
  exit 1
fi

python3 - "$SEQ_DIR" "$OUT_BAG" <<'PY'
import argparse
import sys
from pathlib import Path

import cv2
import rclpy
import rosbag2_py
from builtin_interfaces.msg import Time
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def ns_to_stamp(sec_float: float) -> Time:
    t = Time()
    t.sec = int(sec_float)
    t.nanosec = int(round((sec_float - t.sec) * 1e9))
    return t


def main(seq_dir: Path, out_bag: Path) -> None:
    times = [float(x) for x in (seq_dir / "times.txt").read_text().split()]
    left_dir = seq_dir / "image_0"
    right_dir = seq_dir / "image_1"

    storage = StorageOptions(uri=str(out_bag), storage_id="sqlite3")
    writer = SequentialWriter()
    writer.open(storage, rosbag2_py.ConverterOptions("", ""))
    for name, typ in (
        ("/cam0/image_raw", "sensor_msgs/msg/Image"),
        ("/cam1/image_raw", "sensor_msgs/msg/Image"),
    ):
        writer.create_topic(TopicMetadata(name=name, type=typ, serialization_format="cdr"))

    for idx, t_sec in enumerate(times):
        ts_ns = int(round(t_sec * 1e9))
        for topic, folder, frame in (
            ("/cam0/image_raw", left_dir, "cam0"),
            ("/cam1/image_raw", right_dir, "cam1"),
        ):
            path = folder / f"{idx:06d}.png"
            img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if img is None:
                continue
            msg = Image()
            msg.header = Header(stamp=ns_to_stamp(t_sec), frame_id=frame)
            msg.height, msg.width = img.shape[:2]
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = msg.width
            msg.data = img.tobytes()
            writer.write(topic, serialize_message(msg), ts_ns)

    writer.close()


if __name__ == "__main__":
    rclpy.init()
    try:
        main(Path(sys.argv[1]), Path(sys.argv[2]))
    finally:
        rclpy.shutdown()
PY

echo "Wrote $OUT_BAG"
