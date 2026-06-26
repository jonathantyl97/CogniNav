"""CPU-only lane line detection (HSV + Canny + Hough). No deep learning."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class LaneLines:
    """Image-space lane segments as (x1, y1, x2, y2) in full-resolution coordinates."""

    left: Optional[Tuple[float, float, float, float]] = None
    right: Optional[Tuple[float, float, float, float]] = None


class OpenCvLaneDetector:
    """Lightweight lane detector tuned for forward-facing road cameras."""

    def __init__(
        self,
        process_width: int = 640,
        roi_top_ratio: float = 0.55,
        min_line_length: int = 40,
        max_line_gap: int = 80,
        hough_threshold: int = 50,
        smooth_alpha: float = 0.25,
    ) -> None:
        self._process_width = process_width
        self._roi_top_ratio = roi_top_ratio
        self._min_line_length = min_line_length
        self._max_line_gap = max_line_gap
        self._hough_threshold = hough_threshold
        self._smooth_alpha = smooth_alpha
        self._left_fit: Optional[np.ndarray] = None  # [slope, intercept] in scaled ROI coords
        self._right_fit: Optional[np.ndarray] = None

    def detect(self, bgr: np.ndarray) -> LaneLines:
        h, w = bgr.shape[:2]
        scale = self._process_width / float(w)
        small = cv2.resize(bgr, (self._process_width, int(h * scale)), interpolation=cv2.INTER_AREA)
        sh, sw = small.shape[:2]

        roi_top = int(sh * self._roi_top_ratio)
        roi = small[roi_top:sh, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, (0, 0, 180), (180, 60, 255))
        yellow = cv2.inRange(hsv, (15, 60, 80), (45, 255, 255))
        mask = cv2.bitwise_or(white, yellow)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(cv2.bitwise_and(gray, mask), 50, 150)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=self._hough_threshold,
            minLineLength=self._min_line_length,
            maxLineGap=self._max_line_gap,
        )

        left_pts: list[tuple[float, float]] = []
        right_pts: list[tuple[float, float]] = []
        mid_x = sw * 0.5

        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0]:
                if x2 == x1:
                    continue
                slope = (y2 - y1) / (x2 - x1)
                if abs(slope) < 0.3:
                    continue
                x_bottom = x1 + (roi.shape[0] - 1 - y1) / slope
                pts = [(x1, y1), (x2, y2)]
                if slope < 0 and x_bottom < mid_x:
                    left_pts.extend(pts)
                elif slope > 0 and x_bottom > mid_x:
                    right_pts.extend(pts)

        left_line = self._fit_side(left_pts, self._left_fit, side="left")
        right_line = self._fit_side(right_pts, self._right_fit, side="right")
        self._left_fit = left_line
        self._right_fit = right_line

        inv_scale = 1.0 / scale
        return LaneLines(
            left=self._segment_to_full(left_line, roi_top, sh, sw, inv_scale) if left_line is not None else None,
            right=self._segment_to_full(right_line, roi_top, sh, sw, inv_scale) if right_line is not None else None,
        )

    def _fit_side(
        self,
        points: list[tuple[float, float]],
        prev: Optional[np.ndarray],
        side: str,
    ) -> Optional[np.ndarray]:
        if len(points) < 2:
            return prev
        xs = np.array([p[0] for p in points], dtype=np.float64)
        ys = np.array([p[1] for p in points], dtype=np.float64)
        try:
            slope, intercept = np.polyfit(ys, xs, 1)
        except np.linalg.LinAlgError:
            return prev
        if side == "left" and slope <= 0:
            return prev
        if side == "right" and slope >= 0:
            return prev
        fit = np.array([slope, intercept], dtype=np.float64)
        if prev is None:
            return fit
        return self._smooth_alpha * fit + (1.0 - self._smooth_alpha) * prev

    @staticmethod
    def _segment_to_full(
        fit: np.ndarray,
        roi_top: int,
        sh: int,
        sw: int,
        inv_scale: float,
    ) -> Tuple[float, float, float, float]:
        y1 = 0.0
        y2 = float(sh - roi_top - 1)
        x1 = fit[0] * y1 + fit[1]
        x2 = fit[0] * y2 + fit[1]
        return (
            x1 * inv_scale,
            (y1 + roi_top) * inv_scale,
            x2 * inv_scale,
            (y2 + roi_top) * inv_scale,
        )

    @staticmethod
    def sample_line(
        line: Tuple[float, float, float, float],
        num_points: int = 16,
    ) -> np.ndarray:
        x1, y1, x2, y2 = line
        ts = np.linspace(0.0, 1.0, num_points, dtype=np.float32)
        xs = x1 + (x2 - x1) * ts
        ys = y1 + (y2 - y1) * ts
        return np.stack([xs, ys], axis=1)
