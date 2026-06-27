#!/usr/bin/env bash
# Convert a EuRoC MAV sequence folder to ROS 2 bag (stereo + IMU).
#
# Usage:
#   ./scripts/bag_from_euroc.sh MH_01_easy
#
# Input:  ${EUROC_DIR:-$HOME/Downloads/euroc}/<seq>/mav0/
# Output: ${EUROC_DIR}/<seq>.bag/

set -euo pipefail

SEQ="${1:-MH_01_easy}"
EUROC_DIR="${EUROC_DIR:-${HOME}/Downloads/euroc}"
SEQ_DIR="$EUROC_DIR/$SEQ"
MAV0="$SEQ_DIR/mav0"
OUT_BAG="$EUROC_DIR/${SEQ}.bag"

if [[ ! -d "$MAV0/cam0/data" ]]; then
  echo "Missing EuRoC sequence at $MAV0"
  echo "Run: ./scripts/download_euroc.sh $SEQ"
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

PYTHON_SCRIPT="$(mktemp)"
trap 'rm -f "$PYTHON_SCRIPT"' EXIT

cat >"$PYTHON_SCRIPT" <<'PY'
import argparse
import csv
from pathlib import Path

import cv2
import rclpy
import rosbag2_py
from rclpy.serialization import serialize_message
from rosbag2_py import SequentialWriter, StorageOptions, TopicMetadata
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header


def ns_to_stamp(ns: int):
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec = ns // 1_000_000_000
    t.nanosec = ns % 1_000_000_000
    return t


def read_cam_csv(path: Path):
    rows = []
    with path.open() as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            rows.append((int(row[0]), row[1]))
    return rows


def read_imu_csv(path: Path):
    rows = []
    with path.open() as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            ts = int(row[0])
            gx, gy, gz = map(float, row[1:4])
            ax, ay, az = map(float, row[4:7])
            rows.append((ts, gx, gy, gz, ax, ay, az))
    return rows


def write_image(writer, topic, frame_id, ts_ns, image_path):
    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        return
    msg = Image()
    msg.header = Header(stamp=ns_to_stamp(ts_ns), frame_id=frame_id)
    msg.height, msg.width = img.shape[:2]
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = msg.width
    msg.data = img.tobytes()
    writer.write(topic, serialize_message(msg), ts_ns)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mav0")
    parser.add_argument("out_bag")
    args = parser.parse_args()

    mav0 = Path(args.mav0)
    out_bag = Path(args.out_bag)

    storage = StorageOptions(uri=str(out_bag), storage_id="sqlite3")
    writer = SequentialWriter()
    writer.open(storage, rosbag2_py.ConverterOptions("", ""))

    topics = [
        ("/cam0/image_raw", "sensor_msgs/msg/Image", Image),
        ("/cam1/image_raw", "sensor_msgs/msg/Image", Image),
        ("/imu0", "sensor_msgs/msg/Imu", Imu),
    ]
    for name, typ, _ in topics:
        writer.create_topic(TopicMetadata(name=name, type=typ, serialization_format="cdr"))

    cam0 = read_cam_csv(mav0 / "cam0" / "data.csv")
    cam1 = read_cam_csv(mav0 / "cam1" / "data.csv")
    imu = read_imu_csv(mav0 / "imu0" / "data.csv")

    for ts, rel in cam0:
        write_image(writer, "/cam0/image_raw", "cam0", ts, mav0 / "cam0" / "data" / rel)
    for ts, rel in cam1:
        write_image(writer, "/cam1/image_raw", "cam1", ts, mav0 / "cam1" / "data" / rel)

    for ts, gx, gy, gz, ax, ay, az in imu:
        msg = Imu()
        msg.header = Header(stamp=ns_to_stamp(ts), frame_id="imu0")
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax, ay, az
        writer.write("/imu0", serialize_message(msg), ts)

    writer.close()


if __name__ == "__main__":
    rclpy.init()
    try:
        main()
    finally:
        rclpy.shutdown()
PY

chmod +x "$PYTHON_SCRIPT"
python3 "$PYTHON_SCRIPT" "$MAV0" "$OUT_BAG"
echo "Wrote $OUT_BAG"
