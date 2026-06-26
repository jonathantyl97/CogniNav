"""Publish subsampled stereo depth point cloud in map frame."""

from __future__ import annotations

from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from cogninav_depth.stereo_depth import StereoDepthEstimator


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def _camera_to_map(points_c: np.ndarray, pose) -> np.ndarray:
    r = _quat_to_rot(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    t = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
    return (points_c @ r.T + t).astype(np.float32)


class StereoDepthNode(Node):
    def __init__(self) -> None:
        super().__init__("cogninav_depth")

        self.declare_parameter("left_image_topic", "/cam0/image_raw")
        self.declare_parameter("right_image_topic", "/cam1/image_raw")
        self.declare_parameter("odom_topic", "/cogninav/odom")
        self.declare_parameter("depth_points_topic", "/cogninav/stereo_points")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("fx", 458.654)
        self.declare_parameter("fy", 457.296)
        self.declare_parameter("cx", 367.215)
        self.declare_parameter("cy", 248.375)
        self.declare_parameter("baseline_m", 0.11)
        self.declare_parameter("max_points", 30000)
        self.declare_parameter("sync_slop_sec", 0.05)

        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._depth_topic = self.get_parameter("depth_points_topic").get_parameter_value().string_value
        self._estimator = StereoDepthEstimator(
            fx=float(self.get_parameter("fx").value),
            fy=float(self.get_parameter("fy").value),
            cx=float(self.get_parameter("cx").value),
            cy=float(self.get_parameter("cy").value),
            baseline_m=float(self.get_parameter("baseline_m").value),
            max_points=int(self.get_parameter("max_points").value),
        )
        self._bridge = CvBridge()
        self._latest_odom: Optional[Odometry] = None
        self._pub = self.create_publisher(PointCloud2, self._depth_topic, 10)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)

        left_topic = self.get_parameter("left_image_topic").get_parameter_value().string_value
        right_topic = self.get_parameter("right_image_topic").get_parameter_value().string_value
        slop = float(self.get_parameter("sync_slop_sec").value)

        left_sub = Subscriber(self, Image, left_topic)
        right_sub = Subscriber(self, Image, right_topic)
        self._sync = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=10, slop=slop)
        self._sync.registerCallback(self._on_stereo_pair)

        self.get_logger().info(f"Stereo depth {left_topic} + {right_topic} -> {self._depth_topic}")

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_stereo_pair(self, left_msg: Image, right_msg: Image) -> None:
        if self._latest_odom is None:
            return
        try:
            left = self._bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            right = self._bridge.imgmsg_to_cv2(right_msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge failed: {exc}")
            return

        pts_c = self._estimator.compute_points(left, right)
        if pts_c is None:
            return

        pts_w = _camera_to_map(pts_c, self._latest_odom.pose.pose)
        header = Header()
        header.stamp = left_msg.header.stamp
        header.frame_id = self._map_frame
        cloud = pc2.create_cloud_xyz32(header, pts_w.tolist())
        self._pub.publish(cloud)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
