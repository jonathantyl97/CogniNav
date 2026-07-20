"""Open-vocabulary object detection with NVIDIA LocateAnything-3B.

Detects cars and humans (or any natural-language categories) via the
Parallel Box Decoding VLM. Runs in bf16 on GPU; one call per frame.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

import numpy as np
import torch
from PIL import Image
from transformers import AutoModel, AutoProcessor, AutoTokenizer

BOX_RE = re.compile(r"<box><(\d+)><(\d+)><(\d+)><(\d+)></box>")
# Output format: <ref>label</ref><box>...</box><box>...</box><ref>next</ref>...
TOKEN_RE = re.compile(r"<ref>(.*?)</ref>|<box><(\d+)><(\d+)><(\d+)><(\d+)></box>")


@dataclass
class Detection:
    label: str
    x1: float
    y1: float
    x2: float
    y2: float


class ObjectDetector:
    """Loads LocateAnything-3B once and serves per-frame detection queries."""

    def __init__(self, model_path: str, device: str = "cuda", dtype=torch.bfloat16):
        self.device = device
        self.dtype = dtype
        self.tokenizer = AutoTokenizer.from_pretrained(model_path, trust_remote_code=True)
        self.processor = AutoProcessor.from_pretrained(model_path, trust_remote_code=True)
        self.model = AutoModel.from_pretrained(
            model_path, torch_dtype=dtype, trust_remote_code=True,
        ).to(device).eval()

    @torch.no_grad()
    def detect(
        self,
        image: Image.Image,
        categories: list[str],
        generation_mode: str = "hybrid",
        max_new_tokens: int = 2048,
    ) -> list[Detection]:
        cats = "</c>".join(categories)
        question = f"Locate all the instances that matches the following description: {cats}."
        messages = [{"role": "user", "content": [
            {"type": "image", "image": image},
            {"type": "text", "text": question},
        ]}]

        text = self.processor.py_apply_chat_template(messages, tokenize=False, add_generation_prompt=True)
        images, videos = self.processor.process_vision_info(messages)
        inputs = self.processor(text=[text], images=images, videos=videos, return_tensors="pt").to(self.device)

        response = self.model.generate(
            pixel_values=inputs["pixel_values"].to(self.dtype),
            input_ids=inputs["input_ids"],
            attention_mask=inputs["attention_mask"],
            image_grid_hws=inputs.get("image_grid_hws", None),
            tokenizer=self.tokenizer,
            max_new_tokens=max_new_tokens,
            use_cache=True,
            generation_mode=generation_mode,
            temperature=0.7,
            do_sample=True,
            top_p=0.9,
            repetition_penalty=1.1,
            verbose=False,
        )
        answer = response[0] if isinstance(response, tuple) else response
        if isinstance(answer, list):
            answer = answer[0]
        return self.parse(answer, image.width, image.height, categories)

    @staticmethod
    def parse(answer: str, w: int, h: int, categories: list[str]) -> list[Detection]:
        """A <ref>label</ref> applies to every following box until the next ref."""
        dets: list[Detection] = []
        label = categories[0] if categories else "object"
        for m in TOKEN_RE.finditer(answer):
            if m.group(1) is not None:
                label = m.group(1).strip().lower()
            else:
                x1, y1, x2, y2 = (int(g) for g in m.groups()[1:])
                dets.append(Detection(label, x1 / 1000 * w, y1 / 1000 * h,
                                      x2 / 1000 * w, y2 / 1000 * h))
        return dets


LABEL_COLORS = {
    "car": (60, 76, 231),      # red-ish (BGR)
    "vehicle": (60, 76, 231),
    "truck": (18, 156, 243),
    "person": (113, 204, 46),  # green
    "human": (113, 204, 46),
    "pedestrian": (113, 204, 46),
}


def draw_detections(frame_bgr: np.ndarray, dets: list[Detection]) -> np.ndarray:
    import cv2

    out = frame_bgr.copy()
    for d in dets:
        color = LABEL_COLORS.get(d.label, (200, 160, 60))
        p1, p2 = (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2))
        cv2.rectangle(out, p1, p2, color, 2)
        label = d.label
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(out, (p1[0], p1[1] - th - 8), (p1[0] + tw + 6, p1[1]), color, -1)
        cv2.putText(out, label, (p1[0] + 3, p1[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return out
