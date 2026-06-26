"""Visualize CogniNav SLAM output with Iridescence (desktop OpenGL viewer)."""

from __future__ import annotations

import threading
from collections import deque
from typing import Deque, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray

try:
    from pyridescence import guik

    _IRIDESCENCE_AVAILABLE = True
except ImportError:
    _IRIDESCENCE_AVAILABLE = False


def _pointcloud2_to_xyz(msg: PointCloud2, max_points: int) -> Optional[np.ndarray]:
    points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not points:
        return None

    xyz = np.asarray(points, dtype=np.float32)
    if len(xyz) > max_points:
        idx = np.linspace(0, len(xyz) - 1, max_points, dtype=np.int64)
        xyz = xyz[idx]
    return xyz


def _pose_xyz(pose: Pose) -> tuple[float, float, float]:
    return (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z),
    )


class IridescenceViewerNode(Node):
    """Subscribes to SLAM topics and renders map points + trajectory in Iridescence."""

    def __init__(self) -> None:
        super().__init__("cogninav_viz")

        self.declare_parameter("enable_viewer", True)
        self.declare_parameter("map_points_topic", "/cogninav/map_points")
        self.declare_parameter("stereo_points_topic", "/cogninav/stereo_points")
        self.declare_parameter("stereo_point_scale", 1.0)
        self.declare_parameter("odom_topic", "/cogninav/odom")
        self.declare_parameter("max_points", 50000)
        self.declare_parameter("trajectory_max_waypoints", 2000)
        self.declare_parameter("point_scale", 1.5)
        self.declare_parameter("trajectory_point_scale", 3.0)
        self.declare_parameter("window_title", "CogniNav SLAM")
        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("lane_markers_topic", "/cogninav/lane_markers")
        self.declare_parameter("in_lane_markers_topic", "/cogninav/in_lane_markers")
        self.declare_parameter("lane_line_width", 3.0)
        self.declare_parameter("in_lane_human_scale", 0.45)
        self.declare_parameter("in_lane_car_scale", 0.75)

        self._enabled = bool(self.get_parameter("enable_viewer").value)
        self._max_points = int(self.get_parameter("max_points").value)
        self._traj_max = int(self.get_parameter("trajectory_max_waypoints").value)
        self._point_scale = float(self.get_parameter("point_scale").value)
        self._stereo_scale = float(self.get_parameter("stereo_point_scale").value)
        self._traj_scale = float(self.get_parameter("trajectory_point_scale").value)
        self._rate_hz = float(self.get_parameter("update_rate_hz").value)
        self._lane_line_width = float(self.get_parameter("lane_line_width").value)
        self._human_scale = float(self.get_parameter("in_lane_human_scale").value)
        self._car_scale = float(self.get_parameter("in_lane_car_scale").value)

        map_topic = self.get_parameter("map_points_topic").get_parameter_value().string_value
        stereo_topic = self.get_parameter("stereo_points_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        lane_topic = self.get_parameter("lane_markers_topic").get_parameter_value().string_value
        in_lane_topic = self.get_parameter("in_lane_markers_topic").get_parameter_value().string_value
        title = self.get_parameter("window_title").get_parameter_value().string_value

        self._viewer = None
        self._lock = threading.Lock()
        self._pending_map: Optional[np.ndarray] = None
        self._pending_stereo: Optional[np.ndarray] = None
        self._trajectory: Deque[np.ndarray] = deque(maxlen=self._traj_max)
        self._camera_xyz: Optional[tuple[float, float, float]] = None
        self._lane_left: Optional[np.ndarray] = None
        self._lane_right: Optional[np.ndarray] = None
        self._in_lane_humans: list[np.ndarray] = []
        self._in_lane_cars: list[np.ndarray] = []
        self._dirty = False

        if self._enabled:
            if not _IRIDESCENCE_AVAILABLE:
                self.get_logger().error(
                    "pyridescence not installed. Run scripts/setup_deps.sh in the CogniNav container."
                )
                self._enabled = False
            else:
                self._viewer = guik.async_viewer(title=title)
                self._viewer.enable_xy_grid()
                self.get_logger().info(f"Iridescence viewer started ({title})")

        if self._enabled:
            self.create_timer(1.0 / max(self._rate_hz, 1.0), self._flush_to_viewer)

        self.create_subscription(PointCloud2, map_topic, self._on_map_points, 10)
        self.create_subscription(PointCloud2, stereo_topic, self._on_stereo_points, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        self.create_subscription(MarkerArray, lane_topic, self._on_lane_markers, 10)
        self.create_subscription(MarkerArray, in_lane_topic, self._on_in_lane_markers, 10)

        self.get_logger().info(
            f"Listening map={map_topic}, stereo={stereo_topic}, odom={odom_topic}, lanes={lane_topic}, in_lane={in_lane_topic}"
        )

    def _on_stereo_points(self, msg: PointCloud2) -> None:
        xyz = _pointcloud2_to_xyz(msg, self._max_points)
        if xyz is None:
            return
        with self._lock:
            self._pending_stereo = xyz
            self._dirty = True

    def _on_map_points(self, msg: PointCloud2) -> None:
        xyz = _pointcloud2_to_xyz(msg, self._max_points)
        if xyz is None:
            return
        with self._lock:
            self._pending_map = xyz
            self._dirty = True

    def _on_odom(self, msg: Odometry) -> None:
        xyz = np.asarray(_pose_xyz(msg.pose.pose), dtype=np.float32)
        with self._lock:
            self._trajectory.append(xyz)
            self._camera_xyz = tuple(xyz.tolist())
            self._dirty = True

    def _on_lane_markers(self, msg: MarkerArray) -> None:
        left: Optional[np.ndarray] = None
        right: Optional[np.ndarray] = None
        for marker in msg.markers:
            if len(marker.points) < 2:
                continue
            pts = np.array([[p.x, p.y, p.z] for p in marker.points], dtype=np.float32)
            if marker.id == 0:
                left = pts
            elif marker.id == 1:
                right = pts
        with self._lock:
            if left is not None:
                self._lane_left = left
            if right is not None:
                self._lane_right = right
            self._dirty = True

    def _on_in_lane_markers(self, msg: MarkerArray) -> None:
        humans: list[np.ndarray] = []
        cars: list[np.ndarray] = []
        for marker in msg.markers:
            p = np.array(
                [[marker.pose.position.x, marker.pose.position.y, marker.pose.position.z]],
                dtype=np.float32,
            )
            if marker.ns == "in_lane_human":
                humans.append(p)
            elif marker.ns == "in_lane_car":
                cars.append(p)
        with self._lock:
            self._in_lane_humans = humans
            self._in_lane_cars = cars
            self._dirty = True

    def _flush_to_viewer(self) -> None:
        if not self._enabled or self._viewer is None:
            return

        with self._lock:
            if not self._dirty:
                return
            map_xyz = self._pending_map
            stereo_xyz = self._pending_stereo
            traj = np.asarray(self._trajectory, dtype=np.float32) if self._trajectory else None
            camera = self._camera_xyz
            lane_left = self._lane_left
            lane_right = self._lane_right
            in_lane_humans = list(self._in_lane_humans)
            in_lane_cars = list(self._in_lane_cars)
            self._dirty = False

        if map_xyz is not None and len(map_xyz) > 0:
            self._viewer.update_points(
                "map_points",
                map_xyz,
                guik.Rainbow().add("point_scale", self._point_scale),
            )

        if stereo_xyz is not None and len(stereo_xyz) > 0:
            self._viewer.update_points(
                "stereo_depth",
                stereo_xyz,
                guik.FlatColor(0.2, 0.75, 0.95, 0.85).add("point_scale", self._stereo_scale),
            )

        if traj is not None and len(traj) > 0:
            self._viewer.update_points(
                "trajectory",
                traj,
                guik.FlatGreen().add("point_scale", self._traj_scale),
            )

        if camera is not None:
            self._viewer.update_coord(
                "camera",
                guik.FlatRed(scale=0.25, trans=camera),
            )

        self._draw_lane_strip("lane_left", lane_left, guik.FlatColor(1.0, 0.85, 0.1, 1.0))
        self._draw_lane_strip("lane_right", lane_right, guik.FlatColor(0.9, 0.9, 0.95, 1.0))

        for i, pt in enumerate(in_lane_humans):
            self._viewer.update_sphere(
                f"in_lane_human_{i}",
                guik.FlatOrange(scale=self._human_scale, trans=tuple(pt[0].tolist())),
            )
        for i, pt in enumerate(in_lane_cars):
            self._viewer.update_sphere(
                f"in_lane_car_{i}",
                guik.FlatBlue(scale=self._car_scale, trans=tuple(pt[0].tolist())),
            )

    def _draw_lane_strip(
        self,
        name: str,
        vertices: Optional[np.ndarray],
        shader,
    ) -> None:
        if vertices is None or len(vertices) < 2:
            return
        self._viewer.update_thin_lines(
            name,
            vertices.astype(np.float32),
            [],
            [],
            True,
            self._lane_line_width,
            shader,
        )

    def destroy_node(self) -> bool:
        if self._viewer is not None and _IRIDESCENCE_AVAILABLE:
            guik.async_destroy()
            self._viewer = None
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = IridescenceViewerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
