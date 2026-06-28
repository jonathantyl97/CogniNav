"""Dense stereo depth via OpenCV StereoSGBM (CPU, lightweight)."""

from __future__ import annotations

from typing import Optional

import cv2
import numpy as np


class StereoDepthEstimator:
    def __init__(
        self,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        baseline_m: float,
        num_disparities: int = 128,
        block_size: int = 5,
        max_points: int = 30000,
    ) -> None:
        self._fx = fx
        self._fy = fy
        self._cx = cx
        self._cy = cy
        self._baseline = baseline_m
        self._max_points = max_points
        self._matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=num_disparities,
            blockSize=block_size,
            P1=8 * 3 * block_size**2,
            P2=32 * 3 * block_size**2,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )

    def compute_points(
        self,
        left_bgr: np.ndarray,
        right_bgr: np.ndarray,
        fx: Optional[float] = None,
        fy: Optional[float] = None,
        cx: Optional[float] = None,
        cy: Optional[float] = None,
    ) -> Optional[np.ndarray]:
        fx = self._fx if fx is None else fx
        fy = self._fy if fy is None else fy
        cx = self._cx if cx is None else cx
        cy = self._cy if cy is None else cy

        left = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
        right = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
        if left.shape != right.shape:
            right = cv2.resize(right, (left.shape[1], left.shape[0]))

        disp = self._matcher.compute(left, right).astype(np.float32) / 16.0
        h, w = disp.shape
        u_coords, v_coords = np.meshgrid(np.arange(w), np.arange(h))
        valid = disp > 1.0
        if not np.any(valid):
            return None

        z = (fx * self._baseline) / disp[valid]
        x = (u_coords[valid] - cx) * z / fx
        y = (v_coords[valid] - cy) * z / fy
        pts = np.stack([x, y, z], axis=1).astype(np.float32)

        # Keep close, reliable depth for AD-style preview (0.5–40 m).
        mask = (pts[:, 2] > 0.5) & (pts[:, 2] < 40.0)
        pts = pts[mask]
        if len(pts) == 0:
            return None
        if len(pts) > self._max_points:
            idx = np.linspace(0, len(pts) - 1, self._max_points, dtype=np.int64)
            pts = pts[idx]
        return pts
