"""Dynamic object mask and ROS detection message helpers."""

from __future__ import annotations

from typing import List

import cv2
import numpy as np
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from cogninav_lanes.opencv_object_detector import ObjectDetection, ObjectKind


def build_dynamic_mask(
    height: int,
    width: int,
    detections: List[ObjectDetection],
    dilate_px: int = 6,
) -> np.ndarray:
    """Binary mask (uint8 0/255) covering dynamic detections."""
    mask = np.zeros((height, width), dtype=np.uint8)
    for det in detections:
        x1 = max(0, int(det.x1))
        y1 = max(0, int(det.y1))
        x2 = min(width, int(np.ceil(det.x2)))
        y2 = min(height, int(np.ceil(det.y2)))
        if x2 > x1 and y2 > y1:
            mask[y1:y2, x1:x2] = 255
    if dilate_px > 0:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (dilate_px, dilate_px))
        mask = cv2.dilate(mask, kernel, iterations=1)
    return mask


def detections_to_ros(
    detections: List[ObjectDetection],
    stamp,
    frame_id: str,
) -> Detection2DArray:
    """Convert OpenCV detections to vision_msgs/Detection2DArray."""
    msg = Detection2DArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id

    for det in detections:
        d = Detection2D()
        cx = 0.5 * (det.x1 + det.x2)
        cy = 0.5 * (det.y1 + det.y2)
        d.bbox.center.position.x = cx
        d.bbox.center.position.y = cy
        d.bbox.size_x = max(1.0, det.x2 - det.x1)
        d.bbox.size_y = max(1.0, det.y2 - det.y1)

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = (
            "person" if det.kind == ObjectKind.HUMAN else "vehicle"
        )
        hyp.hypothesis.score = float(det.confidence)
        d.results.append(hyp)
        msg.detections.append(d)

    return msg
