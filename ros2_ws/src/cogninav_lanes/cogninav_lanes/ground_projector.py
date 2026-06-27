"""Project image pixels to the ground plane in the map frame."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Pose


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def pixels_to_map_ground(
    pixels_uv: np.ndarray,
    intrinsics: tuple[float, float, float, float],
    pose_map_body: Pose,
    ground_z: float,
    camera_height: float,
) -> np.ndarray:
    """
    Lift 2D pixels to 3D on z=ground_z in map frame.

    Assumes camera optical frame is aligned with body (forward-facing stereo rig).
    and mounted ``camera_height`` meters above the ground plane.
    """
    fx, fy, cx, cy = intrinsics
    r_wb = _quat_to_rot(
        pose_map_body.orientation.x,
        pose_map_body.orientation.y,
        pose_map_body.orientation.z,
        pose_map_body.orientation.w,
    )
    t_wb = np.array(
        [
            pose_map_body.position.x,
            pose_map_body.position.y,
            pose_map_body.position.z,
        ],
        dtype=np.float64,
    )
    t_wc = t_wb + r_wb @ np.array([0.0, -camera_height, 0.0], dtype=np.float64)

    points: list[np.ndarray] = []
    for u, v in pixels_uv:
        d_c = np.array([(u - cx) / fx, (v - cy) / fy, 1.0], dtype=np.float64)
        d_c /= np.linalg.norm(d_c)
        d_w = r_wb @ d_c
        if abs(d_w[2]) < 1e-6:
            continue
        t_hit = (ground_z - t_wc[2]) / d_w[2]
        if t_hit <= 0.0:
            continue
        p_w = t_wc + t_hit * d_w
        points.append(p_w.astype(np.float32))

    if not points:
        return np.zeros((0, 3), dtype=np.float32)
    return np.vstack(points)
