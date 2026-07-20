"""Lane / drivable-path detection for road and warehouse-aisle scenes.

Classical-CV pipeline (gradient + color thresholds, ROI, Hough fitting with
temporal smoothing) that overlays a drivable corridor polygon on each frame.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class LaneState:
    """Temporally smoothed left/right boundary fits (slope/intercept in pixels)."""

    left: np.ndarray | None = None
    right: np.ndarray | None = None
    alpha: float = 0.25
    misses: int = 0

    def update(self, side: str, fit: np.ndarray | None) -> None:
        cur = getattr(self, side)
        if fit is None:
            return
        setattr(self, side, fit if cur is None else self.alpha * fit + (1 - self.alpha) * cur)


@dataclass
class LaneDetector:
    """mode='road' expects painted lane lines; mode='warehouse' tracks aisle edges."""

    mode: str = "road"
    state: LaneState = field(default_factory=LaneState)

    def detect(self, frame_bgr: np.ndarray) -> dict:
        h, w = frame_bgr.shape[:2]
        edges = self._edge_map(frame_bgr)
        roi = self._roi_mask(h, w)
        edges = cv2.bitwise_and(edges, roi)

        lines = cv2.HoughLinesP(
            edges, rho=2, theta=np.pi / 180, threshold=40,
            minLineLength=max(20, h // 12), maxLineGap=h // 8,
        )
        left_fit, right_fit = self._split_and_fit(lines, w)
        self.state.update("left", left_fit)
        self.state.update("right", right_fit)
        if left_fit is None and right_fit is None:
            self.state.misses += 1
        else:
            self.state.misses = 0

        return {
            "left": self.state.left,
            "right": self.state.right,
            "valid": self.state.misses < 15 and (self.state.left is not None or self.state.right is not None),
        }

    def overlay(self, frame_bgr: np.ndarray, det: dict) -> np.ndarray:
        h, w = frame_bgr.shape[:2]
        out = frame_bgr.copy()
        if not det["valid"]:
            return out

        y_bot = h - 1
        y_top = int(h * (0.62 if self.mode == "road" else 0.55))

        # Never extend boundaries past their intersection (vanishing point),
        # otherwise the corridor lines cross into an X.
        lf, rf = det["left"], det["right"]
        if lf is not None and rf is not None and abs(lf[0] - rf[0]) > 1e-4:
            y_vanish = (lf[0] * rf[1] - rf[0] * lf[1]) / (lf[0] - rf[0])
            y_top = int(max(y_top, y_vanish + 0.04 * h))
        y_top = min(y_top, y_bot - 10)

        def x_at(fit: np.ndarray | None, y: int) -> int | None:
            if fit is None:
                return None
            slope, intercept = fit
            if abs(slope) < 1e-3:
                return None
            return int((y - intercept) / slope)

        lx_b, lx_t = x_at(det["left"], y_bot), x_at(det["left"], y_top)
        rx_b, rx_t = x_at(det["right"], y_bot), x_at(det["right"], y_top)

        # If one boundary is missing, mirror the other around a nominal lane width.
        lane_w = int(w * 0.42)
        if lx_b is None and rx_b is not None:
            lx_b, lx_t = rx_b - lane_w, rx_t - lane_w
        if rx_b is None and lx_b is not None:
            rx_b, rx_t = lx_b + lane_w, lx_t + lane_w
        if lx_b is None or rx_b is None:
            return out

        corridor = np.array([[lx_b, y_bot], [lx_t, y_top], [rx_t, y_top], [rx_b, y_bot]], dtype=np.int32)
        fill = out.copy()
        cv2.fillPoly(fill, [corridor], (80, 200, 80))
        out = cv2.addWeighted(fill, 0.30, out, 0.70, 0)

        cv2.line(out, (lx_b, y_bot), (lx_t, y_top), (0, 255, 255), 4)
        cv2.line(out, (rx_b, y_bot), (rx_t, y_top), (0, 255, 255), 4)

        # Center guidance line and lateral offset readout.
        cx_b, cx_t = (lx_b + rx_b) // 2, (lx_t + rx_t) // 2
        cv2.line(out, (cx_b, y_bot), (cx_t, y_top), (255, 160, 40), 2, cv2.LINE_AA)
        offset_frac = (cx_b - w / 2) / (rx_b - lx_b + 1e-6)
        cv2.putText(out, f"path offset {offset_frac:+.2f}", (12, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        return out

    # ------------------------------------------------------------------

    def _edge_map(self, frame_bgr: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 50, 150)

        if self.mode == "road":
            # Boost painted white/yellow markings.
            hls = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HLS)
            white = cv2.inRange(hls, (0, 190, 0), (255, 255, 255))
            yellow = cv2.inRange(hls, (10, 60, 90), (40, 210, 255))
            paint = cv2.bitwise_or(white, yellow)
            paint = cv2.Canny(paint, 50, 150)
            edges = cv2.bitwise_or(edges, paint)
        return edges

    def _roi_mask(self, h: int, w: int) -> np.ndarray:
        mask = np.zeros((h, w), dtype=np.uint8)
        if self.mode == "road":
            poly = np.array([
                [int(0.05 * w), h - 1], [int(0.42 * w), int(0.60 * h)],
                [int(0.58 * w), int(0.60 * h)], [int(0.98 * w), h - 1],
            ], dtype=np.int32)
        else:  # warehouse aisle: corridor edges converge higher in the image
            poly = np.array([
                [0, h - 1], [int(0.30 * w), int(0.40 * h)],
                [int(0.70 * w), int(0.40 * h)], [w - 1, h - 1],
            ], dtype=np.int32)
        cv2.fillPoly(mask, [poly], 255)
        return mask

    def _split_and_fit(self, lines, w: int):
        if lines is None:
            return None, None
        min_slope = 0.35 if self.mode == "road" else 0.25
        left_pts, right_pts = [], []
        for x1, y1, x2, y2 in lines[:, 0]:
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            if abs(slope) < min_slope or abs(slope) > 6.0:
                continue
            mid_x = (x1 + x2) / 2
            if slope < 0 and mid_x < w * 0.55:
                left_pts.append((x1, y1, x2, y2))
            elif slope > 0 and mid_x > w * 0.45:
                right_pts.append((x1, y1, x2, y2))

        def fit(pts):
            if not pts:
                return None
            xs = np.array([[p[0], p[2]] for p in pts]).ravel()
            ys = np.array([[p[1], p[3]] for p in pts]).ravel()
            return np.polyfit(xs, ys, 1)  # slope, intercept (y = m x + b)

        return fit(left_pts), fit(right_pts)
