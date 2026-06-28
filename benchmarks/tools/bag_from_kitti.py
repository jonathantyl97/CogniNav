#!/usr/bin/env python3
"""Convert KITTI odometry sequence to ROS 2 bag (stereo gray, no IMU)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import rclpy
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import ConverterOptions, SequentialWriter, StorageOptions, TopicMetadata


def main() -> int:
    parser = argparse.ArgumentParser(description="KITTI odometry → ROS 2 bag")
    parser.add_argument("seq", nargs="?", default="00", help="Sequence id, e.g. 00")
    parser.add_argument(
        "--kitti-dir",
        type=Path,
        default=Path.home() / "Downloads" / "kitti",
    )
    args = parser.parse_args()

    seq = f"{int(args.seq):02d}"
    seq_dir = args.kitti_dir / "sequences" / seq
    out_bag = args.kitti_dir / f"{seq}_ros2"

    if not (seq_dir / "image_0").is_dir():
        print(f"Missing {seq_dir}/image_0 — see README.md (Datasets)", file=sys.stderr)
        return 1
    if out_bag.is_dir():
        print(f"Bag already exists: {out_bag}")
        return 0

    left_files = sorted((seq_dir / "image_0").glob("*.png"))
    right_files = sorted((seq_dir / "image_1").glob("*.png"))
    times_path = seq_dir / "times.txt"
    times = times_path.read_text().splitlines() if times_path.is_file() else []

    if not left_files or len(left_files) != len(right_files):
        print(f"Image count mismatch in {seq_dir}", file=sys.stderr)
        return 1

    rclpy.init()
    Image = get_message("sensor_msgs/msg/Image")
    writer = SequentialWriter()
    writer.open(
        StorageOptions(uri=str(out_bag), storage_id="sqlite3"),
        converter_options=ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    for topic, typ in (
        ("/kitti/cam0/image_raw", "sensor_msgs/msg/Image"),
        ("/kitti/cam1/image_raw", "sensor_msgs/msg/Image"),
    ):
        writer.create_topic(
            TopicMetadata(name=topic, type=typ, serialization_format="cdr")
        )

    for idx, (lf, rf) in enumerate(zip(left_files, right_files)):
        t_sec = float(times[idx]) if idx < len(times) else idx * 0.1
        stamp_nsec_total = int(t_sec * 1e9)
        stamp_sec = stamp_nsec_total // 1_000_000_000
        stamp_nsec = stamp_nsec_total % 1_000_000_000

        for path, topic in ((lf, "/kitti/cam0/image_raw"), (rf, "/kitti/cam1/image_raw")):
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                print(f"Failed to read {path}", file=sys.stderr)
                return 1
            h, w = gray.shape
            msg = Image()
            msg.header.stamp.sec = int(stamp_sec)
            msg.header.stamp.nanosec = int(stamp_nsec)
            msg.header.frame_id = "kitti_cam0" if "cam0" in topic else "kitti_cam1"
            msg.height = h
            msg.width = w
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = w
            msg.data = gray.tobytes()
            writer.write(topic, serialize_message(msg), stamp_nsec_total)

    writer.close()
    rclpy.shutdown()
    print(f"Wrote {out_bag} ({len(left_files)} stereo pairs)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
