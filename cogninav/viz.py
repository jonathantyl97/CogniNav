"""Rendering: SLAM point-cloud views and the composite navigation dashboard."""

from __future__ import annotations

import cv2
import numpy as np

from .slam import SlamResult


# ---------------------------------------------------------------------------
# Point-cloud renderer (pure NumPy z-buffer splatting)
# ---------------------------------------------------------------------------

class MapRenderer:
    """Accumulates the SLAM map and renders it from a virtual chase camera."""

    def __init__(self, slam: SlamResult, conf_threshold: float = 1.5,
                 subsample: int = 6, size: tuple[int, int] = (640, 480)):
        self.slam = slam
        self.size = size
        self.conf_threshold = conf_threshold
        self.subsample = subsample
        self.pts: list[np.ndarray] = []
        self.cols: list[np.ndarray] = []
        self._scene_scale = None

    def accumulate(self, i: int) -> None:
        wp = self.slam.world_points[i][:: self.subsample, :: self.subsample].reshape(-1, 3)
        conf = self.slam.conf[i][:: self.subsample, :: self.subsample].reshape(-1)
        rgb = self.slam.images[i][:, :: self.subsample, :: self.subsample]
        rgb = rgb.transpose(1, 2, 0).reshape(-1, 3)
        keep = conf > self.conf_threshold
        self.pts.append(wp[keep])
        self.cols.append(rgb[keep])

    def render(self, i: int, trail: int = 10_000) -> np.ndarray:
        """Render accumulated map + trajectory from a chase cam behind pose i."""
        w, h = self.size
        canvas = np.full((h, w, 3), 15, dtype=np.uint8)
        if not self.pts:
            return canvas
        pts = np.concatenate(self.pts)
        cols = np.concatenate(self.cols)

        traj = self.slam.trajectory[: i + 1]
        cam = self._chase_camera(i)

        # World -> camera
        pc = (pts - cam["eye"]) @ cam["R"].T
        valid = pc[:, 2] > 1e-3
        pc, cc = pc[valid], cols[valid]

        f = 0.9 * w
        u = (f * pc[:, 0] / pc[:, 2] + w / 2).astype(np.int32)
        v = (f * pc[:, 1] / pc[:, 2] + h / 2).astype(np.int32)
        inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u, v, z, cc = u[inside], v[inside], pc[inside, 2], cc[inside]

        # z-buffer: sort far-to-near so near points overwrite
        order = np.argsort(-z)
        u, v, cc = u[order], v[order], cc[order]
        colors = (cc * 255).astype(np.uint8)
        # 2x2 splats for visibility
        for du, dv in ((0, 0), (1, 0), (0, 1), (1, 1)):
            uu = np.clip(u + du, 0, w - 1)
            vv = np.clip(v + dv, 0, h - 1)
            canvas[vv, uu] = colors

        # Trajectory polyline
        tc = (traj - cam["eye"]) @ cam["R"].T
        tv = tc[:, 2] > 1e-3
        if tv.sum() >= 2:
            tu = (f * tc[tv, 0] / tc[tv, 2] + w / 2).astype(np.int32)
            tw = (f * tc[tv, 1] / tc[tv, 2] + h / 2).astype(np.int32)
            pts2d = np.stack([tu, tw], axis=1).reshape(-1, 1, 2)
            cv2.polylines(canvas, [pts2d], False, (60, 220, 255), 2, cv2.LINE_AA)
            cv2.circle(canvas, (int(tu[-1]), int(tw[-1])), 6, (0, 90, 255), -1, cv2.LINE_AA)

        return canvas

    def _scale(self) -> float:
        if self._scene_scale is None:
            traj = self.slam.trajectory
            self._scene_scale = max(float(np.linalg.norm(traj.max(0) - traj.min(0))), 1e-3)
        return self._scene_scale

    def _chase_camera(self, i: int) -> dict:
        """Chase cam: behind and above the current pose, looking ahead."""
        ext = self.slam.extrinsic[i]
        pos, fwd = ext[:3, 3], ext[:3, 2]
        up_world = -self.slam.extrinsic[max(0, i)][:3, 1]
        s = self._scale()
        eye = pos - fwd * 0.35 * s + up_world * 0.18 * s
        target = pos + fwd * 0.25 * s

        z = target - eye
        z = z / (np.linalg.norm(z) + 1e-9)
        x = np.cross(z, up_world)
        x = x / (np.linalg.norm(x) + 1e-9)
        y = np.cross(z, x)
        R = np.stack([x, y, z])  # rows: cam axes in world
        return {"eye": eye, "R": R}


# ---------------------------------------------------------------------------
# Dashboard compositing
# ---------------------------------------------------------------------------

PANEL_LABEL_STYLE = dict(fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.52, thickness=1)


def _panel(img: np.ndarray, size: tuple[int, int], title: str) -> np.ndarray:
    out = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
    cv2.rectangle(out, (0, 0), (size[0] - 1, 24), (25, 25, 25), -1)
    cv2.putText(out, title, (8, 17), color=(240, 240, 240), lineType=cv2.LINE_AA, **PANEL_LABEL_STYLE)
    cv2.rectangle(out, (0, 0), (size[0] - 1, size[1] - 1), (70, 70, 70), 1)
    return out


def compose_dashboard(
    lane_frame: np.ndarray,
    det_frame: np.ndarray,
    slam_frame: np.ndarray,
    hud: dict | None = None,
) -> np.ndarray:
    """2x2 dashboard: big lane/path view left, detection + SLAM stacked right."""
    H = 720
    left_w, right_w = 850, 430
    left = _panel(lane_frame, (left_w, H), "PATH / LANE GUIDANCE")
    det = _panel(det_frame, (right_w, H // 2), "OBJECT DETECTION  (LocateAnything-3B)")
    slam = _panel(slam_frame, (right_w, H - H // 2), "STREAMING SLAM MAP  (LingBot-Map)")
    right = np.vstack([det, slam])
    board = np.hstack([left, right])

    if hud:
        y = H - 12
        text = "  |  ".join(f"{k}: {v}" for k, v in hud.items())
        cv2.putText(board, text, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 180), 1, cv2.LINE_AA)
    return board


def tensor_frame_to_bgr(images: np.ndarray, i: int) -> np.ndarray:
    """SlamResult.images[i] ([3,H,W] float in [0,1]) -> BGR uint8."""
    rgb = (images[i].transpose(1, 2, 0) * 255).clip(0, 255).astype(np.uint8)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
