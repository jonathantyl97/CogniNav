"""Lightweight MobileNet-SSD object detector via OpenCV DNN (CPU)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np


class ObjectKind(Enum):
    HUMAN = "human"
    CAR = "car"


@dataclass
class ObjectDetection:
    kind: ObjectKind
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def foot_u(self) -> float:
        return 0.5 * (self.x1 + self.x2)

    @property
    def foot_v(self) -> float:
        return self.y2


# PASCAL VOC class ids used by MobileNet-SSD deploy model
_VOC_PERSON = 15
_VOC_CAR = 7
_VOC_BUS = 6
_VOC_MOTORBIKE = 14


class MobileNetSsdDetector:
    """MobileNet-SSD @ 300x300 — no PyTorch required."""

    def __init__(
        self,
        prototxt: str,
        caffemodel: str,
        confidence_threshold: float = 0.45,
        input_size: int = 300,
    ) -> None:
        proto = Path(prototxt)
        weights = Path(caffemodel)
        if not proto.is_file() or not weights.is_file():
            raise FileNotFoundError(
                f"MobileNet-SSD model not found.\n"
                f"  prototxt: {proto}\n"
                f"  caffemodel: {weights}\n"
                f"See README.md (Datasets) for MobileNet-SSD weights."
            )
        self._net = cv2.dnn.readNetFromCaffe(str(proto), str(weights))
        self._conf = confidence_threshold
        self._input_size = input_size

    def detect(self, bgr: np.ndarray) -> List[ObjectDetection]:
        h, w = bgr.shape[:2]
        blob = cv2.dnn.blobFromImage(
            cv2.resize(bgr, (self._input_size, self._input_size)),
            scalefactor=0.007843,
            size=(self._input_size, self._input_size),
            mean=127.5,
        )
        self._net.setInput(blob)
        raw = self._net.forward()

        detections: List[ObjectDetection] = []
        for i in range(raw.shape[2]):
            conf = float(raw[0, 0, i, 2])
            if conf < self._conf:
                continue
            class_id = int(raw[0, 0, i, 1])
            kind = self._class_to_kind(class_id)
            if kind is None:
                continue
            box = raw[0, 0, i, 3:7] * np.array([w, h, w, h], dtype=np.float32)
            x1, y1, x2, y2 = box.tolist()
            detections.append(
                ObjectDetection(
                    kind=kind,
                    confidence=conf,
                    x1=float(x1),
                    y1=float(y1),
                    x2=float(x2),
                    y2=float(y2),
                )
            )
        return detections

    @staticmethod
    def _class_to_kind(class_id: int) -> Optional[ObjectKind]:
        if class_id == _VOC_PERSON:
            return ObjectKind.HUMAN
        if class_id in (_VOC_CAR, _VOC_BUS, _VOC_MOTORBIKE):
            return ObjectKind.CAR
        return None
