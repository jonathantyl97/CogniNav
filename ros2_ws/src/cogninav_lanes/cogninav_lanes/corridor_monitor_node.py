"""Lane corridor monitor: lanes + in-corridor human/car marking."""

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
from cogninav_lanes.lane_corridor import point_in_lane_corridor
from cogninav_lanes.opencv_lane_detector import OpenCvLaneDetector
from cogninav_lanes.opencv_object_detector import MobileNetSsdDetector, ObjectDetection, ObjectKind


class CorridorMonitorNode(Node):
    """Detects lanes, finds humans/cars inside the lane corridor, publishes map markers."""

    def __init__(self) -> None:
        super().__init__("cogninav_corridor_monitor")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("odom_topic", "/cogninav/odom")
        self.declare_parameter("lane_markers_topic", "/cogninav/lane_markers")
        self.declare_parameter("in_lane_markers_topic", "/cogninav/in_lane_markers")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("fx", 458.654)
        self.declare_parameter("fy", 457.296)
        self.declare_parameter("cx", 367.215)
        self.declare_parameter("cy", 248.375)
        self.declare_parameter("camera_height", 0.0)
        self.declare_parameter("ground_z", 0.0)
        self.declare_parameter("process_width", 640)
        self.declare_parameter("sample_points", 20)
        self.declare_parameter("history_frames", 40)
        self.declare_parameter("corridor_margin_px", 8.0)
        self.declare_parameter("detection_confidence", 0.45)
        self.declare_parameter(
            "mobilenet_prototxt",
            "/root/cogninav/models/MobileNetSSD_deploy.prototxt",
        )
        self.declare_parameter(
            "mobilenet_weights",
            "/root/cogninav/models/MobileNetSSD_deploy.caffemodel",
        )
        self.declare_parameter("enable_object_detection", True)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        lane_topic = self.get_parameter("lane_markers_topic").get_parameter_value().string_value
        in_lane_topic = self.get_parameter("in_lane_markers_topic").get_parameter_value().string_value
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
        self._corridor_margin = float(self.get_parameter("corridor_margin_px").value)

        self._lane_detector = OpenCvLaneDetector(
            process_width=int(self.get_parameter("process_width").value),
        )
        self._object_detector: Optional[MobileNetSsdDetector] = None
        if bool(self.get_parameter("enable_object_detection").value):
            try:
                self._object_detector = MobileNetSsdDetector(
                    self.get_parameter("mobilenet_prototxt").get_parameter_value().string_value,
                    self.get_parameter("mobilenet_weights").get_parameter_value().string_value,
                    confidence_threshold=float(self.get_parameter("detection_confidence").value),
                )
            except FileNotFoundError as exc:
                self.get_logger().error(str(exc))

        self._bridge = CvBridge()
        self._latest_odom: Optional[Odometry] = None
        self._left_history: Deque[np.ndarray] = deque(maxlen=self._history_max)
        self._right_history: Deque[np.ndarray] = deque(maxlen=self._history_max)

        self._lane_pub = self.create_publisher(MarkerArray, lane_topic, 10)
        self._in_lane_pub = self.create_publisher(MarkerArray, in_lane_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(
            f"Corridor monitor image={image_topic} lanes->{lane_topic} in_lane->{in_lane_topic}"
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

        lanes = self._lane_detector.detect(bgr)
        pose = self._latest_odom.pose.pose

        if lanes.left is not None:
            left_map = self._line_to_map(lanes.left, pose)
            if left_map is not None:
                self._left_history.append(left_map)
        if lanes.right is not None:
            right_map = self._line_to_map(lanes.right, pose)
            if right_map is not None:
                self._right_history.append(right_map)

        lane_markers = MarkerArray()
        lane_markers.markers.append(
            self._lane_strip_marker(0, self._merge_history(self._left_history), (1.0, 0.85, 0.1))
        )
        lane_markers.markers.append(
            self._lane_strip_marker(1, self._merge_history(self._right_history), (0.9, 0.9, 0.95))
        )
        self._lane_pub.publish(lane_markers)

        in_lane_markers = MarkerArray()
        if self._object_detector is not None:
            detections = self._object_detector.detect(bgr)
            in_corridor = self._filter_in_corridor(detections, lanes.left, lanes.right)
            in_lane_markers = self._make_in_lane_markers(in_corridor, pose)
        self._in_lane_pub.publish(in_lane_markers)

    def _filter_in_corridor(
        self,
        detections: list[ObjectDetection],
        left,
        right,
    ) -> list[ObjectDetection]:
        inside: list[ObjectDetection] = []
        for det in detections:
            if point_in_lane_corridor(
                det.foot_u,
                det.foot_v,
                left,
                right,
                margin_px=self._corridor_margin,
            ):
                inside.append(det)
        return inside

    def _line_to_map(self, line, pose) -> Optional[np.ndarray]:
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

    def _make_in_lane_markers(
        self,
        detections: list[ObjectDetection],
        pose,
    ) -> MarkerArray:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for idx, det in enumerate(detections):
            foot = np.array([[det.foot_u, det.foot_v]], dtype=np.float32)
            xyz = pixels_to_map_ground(
                foot,
                self._intrinsics,
                pose,
                self._ground_z,
                self._camera_height,
            )
            if len(xyz) == 0:
                continue

            marker = Marker()
            marker.header.frame_id = self._map_frame
            marker.header.stamp = stamp
            marker.ns = "in_lane_human" if det.kind == ObjectKind.HUMAN else "in_lane_car"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(xyz[0, 0])
            marker.pose.position.y = float(xyz[0, 1])
            marker.pose.position.z = float(xyz[0, 2])
            marker.pose.orientation.w = 1.0

            if det.kind == ObjectKind.HUMAN:
                marker.scale.x = marker.scale.y = marker.scale.z = 0.45
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.35, 0.1
            else:
                marker.scale.x = marker.scale.y = marker.scale.z = 0.75
                marker.color.r, marker.color.g, marker.color.b = 0.15, 0.55, 1.0
            marker.color.a = 0.9
            markers.markers.append(marker)

        return markers

    def _lane_strip_marker(
        self,
        marker_id: int,
        points_xyz: np.ndarray,
        color: tuple[float, float, float],
    ) -> Marker:
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
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
            marker.points.append(pt)
        return marker


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = CorridorMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
