"""Streaming SLAM via LingBot-Map (Geometric Context Transformer).

Wraps the lingbot-map feed-forward 3D foundation model to produce per-frame
camera poses (c2w), dense depth, and world points from a monocular stream.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass

import numpy as np
import torch

from lingbot_map.utils.geometry import closed_form_inverse_se3_general
from lingbot_map.utils.load_fn import load_and_preprocess_images
from lingbot_map.utils.pose_enc import pose_encoding_to_extri_intri


@dataclass
class SlamResult:
    """Per-frame SLAM outputs for a sequence of S frames at model resolution HxW."""

    images: np.ndarray        # [S, 3, H, W] float32 in [0, 1] (model input)
    extrinsic: np.ndarray     # [S, 3, 4] camera-to-world
    intrinsic: np.ndarray     # [S, 3, 3]
    depth: np.ndarray         # [S, H, W, 1]
    world_points: np.ndarray  # [S, H, W, 3]
    conf: np.ndarray          # [S, H, W] world point confidence
    fps: float                # end-to-end model throughput

    @property
    def trajectory(self) -> np.ndarray:
        """Camera centers [S, 3] in world frame."""
        return self.extrinsic[:, :3, 3]

    def save(self, path: str) -> None:
        np.savez_compressed(
            path,
            images=self.images.astype(np.float16),
            extrinsic=self.extrinsic,
            intrinsic=self.intrinsic,
            depth=self.depth.astype(np.float16),
            world_points=self.world_points.astype(np.float16),
            conf=self.conf.astype(np.float16),
            fps=self.fps,
        )

    @staticmethod
    def load(path: str) -> "SlamResult":
        z = np.load(path)
        return SlamResult(
            images=z["images"].astype(np.float32),
            extrinsic=z["extrinsic"],
            intrinsic=z["intrinsic"],
            depth=z["depth"].astype(np.float32),
            world_points=z["world_points"].astype(np.float32),
            conf=z["conf"].astype(np.float32),
            fps=float(z["fps"]),
        )


def _unproject_depth(depth: np.ndarray, extrinsic_c2w: np.ndarray, intrinsic: np.ndarray) -> np.ndarray:
    """Lift per-frame depth maps to world coordinates. depth: [S,H,W,1] -> [S,H,W,3]."""
    S, H, W, _ = depth.shape
    ys, xs = np.mgrid[0:H, 0:W].astype(np.float32)
    pix = np.stack([xs, ys, np.ones_like(xs)], axis=-1)  # [H,W,3]

    world = np.empty((S, H, W, 3), dtype=np.float32)
    for i in range(S):
        K_inv = np.linalg.inv(intrinsic[i])
        rays = pix @ K_inv.T                     # [H,W,3] camera rays at z=1
        cam_pts = rays * depth[i]                # [H,W,3]
        R, t = extrinsic_c2w[i, :3, :3], extrinsic_c2w[i, :3, 3]
        world[i] = cam_pts @ R.T + t
    return world


def run_slam(
    image_paths: list[str],
    model_path: str,
    image_size: int = 518,
    keyframe_interval: int | None = None,
    camera_num_iterations: int = 4,
    kv_sliding_window: int = 32,
    device: str = "cuda",
) -> SlamResult:
    """Run LingBot-Map streaming inference over an ordered image sequence."""
    from lingbot_map.models.gct_stream import GCTStream

    images = load_and_preprocess_images(image_paths, mode="crop", image_size=image_size, patch_size=14)
    num_frames = images.shape[0]

    model = GCTStream(
        img_size=image_size,
        patch_size=14,
        enable_3d_rope=True,
        max_frame_num=1024,
        kv_cache_sliding_window=kv_sliding_window,
        kv_cache_scale_frames=8,
        kv_cache_cross_frame_special=True,
        kv_cache_include_scale_frames=True,
        use_sdpa=True,
        camera_num_iterations=camera_num_iterations,
    )
    ckpt = torch.load(model_path, map_location=device, weights_only=False)
    model.load_state_dict(ckpt.get("model", ckpt), strict=False)
    model = model.to(device).eval()

    dtype = torch.bfloat16 if torch.cuda.get_device_capability()[0] >= 8 else torch.float16
    model.aggregator = model.aggregator.to(dtype=dtype)

    if keyframe_interval is None:
        # Cap cached keyframes at ~160 (fits 16 GB GPUs at 518px), and never
        # exceed the 320-view RoPE training range.
        keyframe_interval = max(1, (num_frames + 159) // 160)

    images_gpu = images.to(device)
    print(f"[slam] {num_frames} frames @ {tuple(images.shape[-2:])}, keyframe_interval={keyframe_interval}")

    t0 = time.time()
    with torch.no_grad(), torch.amp.autocast("cuda", dtype=dtype):
        pred = model.inference_streaming(
            images_gpu,
            num_scale_frames=8,
            keyframe_interval=keyframe_interval,
            output_device=torch.device("cpu"),
        )
    elapsed = time.time() - t0
    fps = num_frames / elapsed
    print(f"[slam] inference {elapsed:.1f}s ({fps:.1f} FPS)")

    extrinsic_w2c, intrinsic = pose_encoding_to_extri_intri(pred["pose_enc"], images.shape[-2:])
    ext44 = torch.zeros((*extrinsic_w2c.shape[:-2], 4, 4), dtype=extrinsic_w2c.dtype)
    ext44[..., :3, :4] = extrinsic_w2c
    ext44[..., 3, 3] = 1.0
    extrinsic_c2w = closed_form_inverse_se3_general(ext44)[..., :3, :4]

    def _np(t: torch.Tensor) -> np.ndarray:
        a = t.detach().cpu().float().numpy()
        return a[0] if a.ndim > 0 and a.shape[0] == 1 else a

    ext_np = _np(extrinsic_c2w)
    intr_np = _np(intrinsic)
    depth_np = _np(pred["depth"])

    result = SlamResult(
        images=images.numpy(),
        extrinsic=ext_np,
        intrinsic=intr_np,
        depth=depth_np,
        world_points=_unproject_depth(depth_np, ext_np, intr_np),
        conf=_np(pred["depth_conf"]),
        fps=fps,
    )

    del model, pred, images_gpu
    torch.cuda.empty_cache()
    return result
