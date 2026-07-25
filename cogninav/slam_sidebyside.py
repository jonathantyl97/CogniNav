"""Side-by-side SLAM demo: original frame + LingBot-Map point-cloud map.

Usage:
    python -m cogninav.slam_sidebyside --video drive_frames.mp4 --output assets/slam_sidebyside.mp4

Extracts frames, runs LingBot-Map streaming SLAM, then renders the original
input frame on the left and the 3D point-cloud map on the right.
"""

from __future__ import annotations

import argparse
import os
import time

import cv2
import numpy as np

from .data import extract_video_frames, list_frames
from .slam import SlamResult, run_slam
from .viz import MapRenderer, tensor_frame_to_bgr


def sidebyside_video(
    video_path: str,
    output_path: str,
    slam_model: str = "models/lingbot-map/lingbot-map.pt",
    fps: int = 5,
    size: tuple[int, int] = (1280, 720),
    first_k: int | None = None,
    cache_dir: str | None = None,
) -> str:
    """Create a side-by-side video of input frames and LingBot-Map SLAM map."""
    if cache_dir is None:
        cache_dir = os.path.join(
            "outputs", os.path.splitext(os.path.basename(output_path))[0]
        )
    os.makedirs(cache_dir, exist_ok=True)

    frame_dir = os.path.join(cache_dir, "video_frames")
    if os.path.isdir(frame_dir) and os.listdir(frame_dir):
        paths = list_frames(frame_dir)
    else:
        paths = extract_video_frames(video_path, frame_dir, fps=fps)
    if first_k:
        paths = paths[:first_k]
    print(f"[sidebyside] {len(paths)} frames at {fps} fps")

    slam_cache = os.path.join(cache_dir, "slam.npz")
    if os.path.exists(slam_cache):
        print(f"[sidebyside] loading cached SLAM {slam_cache}")
        slam = SlamResult.load(slam_cache)
    else:
        slam = run_slam(paths, slam_model)
        slam.save(slam_cache)
    n = slam.images.shape[0]

    out_h, out_w = size[1], size[0]
    panel_w = out_w // 2
    panel_h = out_h
    map_size = (panel_w, panel_h)

    renderer = MapRenderer(slam, conf_threshold=1.2, subsample=3, size=map_size)

    raw = output_path + ".raw.mp4"
    w = cv2.VideoWriter(raw, cv2.VideoWriter_fourcc(*"mp4v"), fps, (out_w, out_h))
    print(f"[sidebyside] rendering {n} frames -> {output_path}")
    for i in range(n):
        orig_bgr = tensor_frame_to_bgr(slam.images, i)
        orig_panel = cv2.resize(orig_bgr, (panel_w, panel_h))
        cv2.rectangle(orig_panel, (0, 0), (panel_w - 1, 30), (18, 18, 18), -1)
        cv2.putText(orig_panel, "Input camera feed", (12, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 1, cv2.LINE_AA)

        renderer.accumulate(i)
        map_panel = renderer.render(i)
        cv2.rectangle(map_panel, (0, 0), (panel_w - 1, 30), (18, 18, 18), -1)
        cv2.putText(map_panel, f"LingBot-Map point-cloud map  frame {i + 1}/{n}", (12, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 1, cv2.LINE_AA)

        combined = np.hstack([orig_panel, map_panel])
        cv2.putText(combined, f"SLAM: {slam.fps:.1f} FPS model",
                    (12, out_h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 180), 1, cv2.LINE_AA)
        w.write(combined)
        if i % 50 == 0:
            print(f"  {i}/{n}")
    w.release()

    os.system(f'ffmpeg -y -i "{raw}" -c:v libx264 -crf 22 -preset medium -pix_fmt yuv420p "{output_path}" >/dev/null 2>&1')
    os.remove(raw)
    print(f"[sidebyside] done: {output_path} ({os.path.getsize(output_path) / 1e6:.1f} MB)")
    return output_path


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="LingBot-Map side-by-side SLAM demo")
    p.add_argument("--video", type=str, required=True)
    p.add_argument("--output", type=str, default="assets/slam_sidebyside.mp4")
    p.add_argument("--slam_model", type=str, default="models/lingbot-map/lingbot-map.pt")
    p.add_argument("--fps", type=int, default=5)
    p.add_argument("--first_k", type=int, default=None)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--cache_dir", type=str, default=None)
    return p


if __name__ == "__main__":
    args = build_parser().parse_args()
    sidebyside_video(
        video_path=args.video,
        output_path=args.output,
        slam_model=args.slam_model,
        fps=args.fps,
        size=(args.width, args.height),
        first_k=args.first_k,
        cache_dir=args.cache_dir,
    )
