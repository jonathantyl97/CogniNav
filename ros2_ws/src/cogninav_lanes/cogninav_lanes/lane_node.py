"""Lightweight lane detection node: OpenCV on images, polylines in map frame."""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray

from cogninav_lanes.ground_projector import pixels_to_map_ground
from cogninav_lanes.opencv_lane_detector import LaneLines, OpenCvLaneDetector


class LaneNode(Node):
    def __init__(self) -> None:
        super().__init__("cogninav_lanes")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("odom_topic", "/cogninav/odom")
        self.declare_parameter("lane_markers_topic", "/cogninav/lane_markers")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("fx", 458.654)
        self.declare_parameter("fy", 457.296)
        self.declare_parameter("cx", 367.215)
        self.declare_parameter("cy", 248.375)
        self.declare_parameter("camera_height", 1.0)
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("process_width", 640)
        self.declare_parameter("sample_points", 20)
        self.declare_parameter("history_frames", 40)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        markers_topic = self.get_parameter("lane_markers_topic").get_parameter_value().string_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._intrinsics = (
            float(self.get_parameter("fx").value),
            float(self.get_parameter("fy").value),
            float(self.get_parameter("cx").value),
            float(self.get_parameter("cy").value),
        )
        self._camera_height = float(self.get_parameter("camera_height").value)
        self._ground_z = float(self.get_parameter("ground_z").value)
        self._sample_points = int(self.get_parameter("sample_points").value)
        self._history_max = int(self.get_parameter("history_frames").value)

        self._detector = OpenCvLaneDetector(
            process_width=int(self.get_parameter("process_width").value),
        )
        self._bridge = CvBridge()
        self._latest_odom: Optional[Odometry] = None
        self._left_history: Deque[np.ndarray] = deque(maxlen=self._history_max)
        self._right_history: Deque[np.ndarray] = deque(maxlen=self._history_max)

        self._pub = self.create_publisher(MarkerArray, markers_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(
            f"Lane detector (OpenCV) image={image_topic} odom={odom_topic} -> {markers_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _on_image(self, msg: Image) -> None:
        if self._latest_odom is None:
            return

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge failed: {exc}")
            return

        lanes = self._detector.detect(bgr)
        pose = self._latest_odom.pose.pose

        left_map = self._line_to_map(lanes.left, pose)
        right_map = self._line_to_map(lanes.right, pose)

        if left_map is not None:
            self._left_history.append(left_map)
        if right_map is not None:
            self._right_history.append(right_map)

        markers = MarkerArray()
        markers.markers.append(self._make_marker(0, self._merge_history(self._left_history), (1.0, 0.85, 0.1)))
        markers.markers.append(self._make_marker(1, self._merge_history(self._right_history), (0.9, 0.9, 0.95)))
        self._pub.publish(markers)

    def _line_to_map(
        self,
        line: Optional[tuple[float, float, float, float]],
        pose,
    ) -> Optional[np.ndarray]:
        if line is None:
            return None
        pixels = OpenCvLaneDetector.sample_line(line, self._sample_points)
        xyz = pixels_to_map_ground(
            pixels,
            self._intrinsics,
            pose,
            self._ground_z,
            self._camera_height,
        )
        if len(xyz) < 2:
            return None
        return xyz

    def _merge_history(self, history: Deque[np.ndarray]) -> np.ndarray:
        if not history:
            return np.zeros((0, 3), dtype=np.float32)
        return np.vstack(list(history))

    def _make_marker(self, marker_id: int, points_xyz: np.ndarray, color: tuple[float, float, float]) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "cogninav_lanes"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.12
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 0.95
        marker.pose.orientation.w = 1.0

        for p in points_xyz:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            marker.points.append(pt)
        return marker


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = LaneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
