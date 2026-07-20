"""CogniNav end-to-end pipeline.

Stage 1: LingBot-Map streaming SLAM over the full sequence (GPU).
Stage 2: LocateAnything-3B car/human detection on sampled frames (GPU).
Stage 3: lane/path detection + dashboard video render (CPU).

Usage:
    python -m cogninav.pipeline --image_folder data/kitti_frames --mode road
    python -m cogninav.pipeline --bag ~/Downloads/warehouse/r2b_storage \
        --bag_topic d455_1_rgb_image --mode warehouse
"""

from __future__ import annotations

import argparse
import os
import time

import cv2
import numpy as np
from PIL import Image

from .data import extract_bag_frames, extract_video_frames, list_frames
from .detect import Detection, ObjectDetector, draw_detections
from .lanes import LaneDetector
from .slam import SlamResult, run_slam
from .viz import MapRenderer, compose_dashboard, tensor_frame_to_bgr

DEFAULT_SLAM_MODEL = "models/lingbot-map/lingbot-map.pt"
DEFAULT_DET_MODEL = "models/LocateAnything-3B"


def run_pipeline(args: argparse.Namespace) -> str:
    os.makedirs(args.output_dir, exist_ok=True)

    # ── Frame source ─────────────────────────────────────────────────────
    if args.bag:
        cache = os.path.join(args.output_dir, "bag_frames")
        paths = sorted(
            os.path.join(cache, f) for f in os.listdir(cache)
        ) if os.path.isdir(cache) and os.listdir(cache) else extract_bag_frames(
            os.path.expanduser(args.bag), args.bag_topic, cache)
    elif args.video:
        cache = os.path.join(args.output_dir, "video_frames")
        paths = extract_video_frames(os.path.expanduser(args.video), cache, fps=args.fps)
    else:
        paths = list_frames(os.path.expanduser(args.image_folder))
    if args.first_k:
        paths = paths[: args.first_k]
    if args.stride > 1:
        paths = paths[:: args.stride]
    print(f"[pipeline] {len(paths)} frames")

    # ── Stage 1: SLAM ────────────────────────────────────────────────────
    slam_cache = os.path.join(args.output_dir, "slam.npz")
    if os.path.exists(slam_cache) and not args.no_cache:
        print(f"[pipeline] loading cached SLAM result {slam_cache}")
        slam = SlamResult.load(slam_cache)
    else:
        slam = run_slam(paths, args.slam_model, image_size=args.image_size)
        slam.save(slam_cache)
    n = slam.images.shape[0]

    # ── Stage 2: object detection (sampled frames, tracked between) ─────
    det_cache = os.path.join(args.output_dir, "detections.npz")
    if os.path.exists(det_cache) and not args.no_cache:
        z = np.load(det_cache, allow_pickle=True)
        all_dets = list(z["dets"])
        det_fps = float(z["fps"])
    else:
        detector = ObjectDetector(args.det_model)
        categories = [c.strip() for c in args.categories.split(",")]
        all_dets, det_times = [], []
        for i in range(n):
            if i % args.det_every == 0:
                frame = tensor_frame_to_bgr(slam.images, i)
                pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                t0 = time.time()
                dets = detector.detect(pil, categories)
                det_times.append(time.time() - t0)
                print(f"[detect] frame {i}/{n}: {len(dets)} objects ({det_times[-1]:.2f}s)")
            all_dets.append([(d.label, d.x1, d.y1, d.x2, d.y2) for d in dets])
        det_fps = 1.0 / (np.mean(det_times) + 1e-9)
        np.savez_compressed(det_cache, dets=np.array(all_dets, dtype=object), fps=det_fps)
        del detector
        import torch
        torch.cuda.empty_cache()

    # ── Stage 3: lanes + dashboard render ────────────────────────────────
    lane = LaneDetector(mode=args.mode)
    renderer = MapRenderer(slam)
    video_path = os.path.join(args.output_dir, f"cogninav_{args.mode}.mp4")
    writer = None

    for i in range(n):
        frame = tensor_frame_to_bgr(slam.images, i)
        det_list = [Detection(*d) for d in all_dets[i]]

        lane_frame = lane.overlay(frame, lane.detect(frame))
        det_frame = draw_detections(frame, det_list)
        renderer.accumulate(i)
        slam_frame = renderer.render(i)

        board = compose_dashboard(
            lane_frame, det_frame, slam_frame,
            hud={
                "frame": f"{i + 1}/{n}",
                "SLAM": f"{slam.fps:.1f} FPS",
                "detector": f"{det_fps:.2f} FPS",
                "objects": len(det_list),
                "mode": args.mode,
            },
        )
        if writer is None:
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(video_path, fourcc, args.out_fps,
                                     (board.shape[1], board.shape[0]))
        writer.write(board)
        if i % 50 == 0:
            print(f"[render] {i}/{n}")

    writer.release()
    print(f"[pipeline] dashboard video: {video_path}")
    return video_path


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="CogniNav GPU navigation pipeline")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--image_folder", type=str)
    src.add_argument("--video", type=str)
    src.add_argument("--bag", type=str, help="ROS 2 bag directory (no ROS install needed)")
    p.add_argument("--bag_topic", type=str, default="d455_1_rgb_image")
    p.add_argument("--mode", choices=["road", "warehouse"], default="road")
    p.add_argument("--slam_model", type=str, default=DEFAULT_SLAM_MODEL)
    p.add_argument("--det_model", type=str, default=DEFAULT_DET_MODEL)
    p.add_argument("--categories", type=str, default="car,person")
    p.add_argument("--det_every", type=int, default=5,
                   help="Run the VLM detector every N frames (detections held between)")
    p.add_argument("--image_size", type=int, default=518)
    p.add_argument("--first_k", type=int, default=None)
    p.add_argument("--stride", type=int, default=1)
    p.add_argument("--fps", type=int, default=10, help="Video frame extraction FPS")
    p.add_argument("--out_fps", type=int, default=10)
    p.add_argument("--output_dir", type=str, default="outputs/run")
    p.add_argument("--no_cache", action="store_true")
    return p


if __name__ == "__main__":
    run_pipeline(build_parser().parse_args())
