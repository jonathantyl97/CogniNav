"""Frame sources: image folders, videos, and ROS 2 bags (no ROS install needed)."""

from __future__ import annotations

import glob
import os
from pathlib import Path

import cv2
import numpy as np

IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".JPG", ".PNG")


def list_frames(image_folder: str, stride: int = 1, first_k: int | None = None) -> list[str]:
    paths: list[str] = []
    for ext in IMAGE_EXTS:
        paths.extend(glob.glob(os.path.join(image_folder, f"*{ext}")))
    paths = sorted(paths)
    if first_k:
        paths = paths[:first_k]
    if stride > 1:
        paths = paths[::stride]
    if not paths:
        raise FileNotFoundError(f"No images found in {image_folder}")
    return paths


def extract_video_frames(video_path: str, out_dir: str, fps: int = 10) -> list[str]:
    os.makedirs(out_dir, exist_ok=True)
    cap = cv2.VideoCapture(video_path)
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30
    interval = max(1, round(src_fps / fps))
    idx, saved = 0, []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if idx % interval == 0:
            path = os.path.join(out_dir, f"{len(saved):06d}.jpg")
            cv2.imwrite(path, frame)
            saved.append(path)
        idx += 1
    cap.release()
    return saved


def extract_bag_frames(bag_path: str, topic: str, out_dir: str) -> list[str]:
    """Extract an image topic from a ROS 2 bag using rosbags (pure Python)."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore

    os.makedirs(out_dir, exist_ok=True)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    saved: list[str] = []
    with AnyReader([Path(bag_path)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            topics = sorted({c.topic for c in reader.connections})
            raise ValueError(f"Topic {topic!r} not in bag. Available: {topics}")
        for conn, _ts, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            img = _decode_image_msg(msg)
            path = os.path.join(out_dir, f"{len(saved):06d}.png")
            cv2.imwrite(path, img)
            saved.append(path)
    return saved


def _decode_image_msg(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image into a BGR uint8 array."""
    h, w = msg.height, msg.width
    enc = msg.encoding
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("rgb8", "bgr8"):
        img = data.reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == "rgb8" else img
    if enc in ("mono8", "8UC1"):
        img = data.reshape(h, w)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc in ("mono16", "16UC1"):
        img16 = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        img = (img16 / max(img16.max(), 1) * 255).astype(np.uint8)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"Unsupported image encoding: {enc}")
