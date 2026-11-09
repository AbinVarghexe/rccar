"""Small TensorFlow Lite detector for Raspberry Pi deployments.

The detector intentionally accepts the common YOLO export layouts instead of
depending on PyTorch/Ultralytics at runtime. This keeps inference local and
allows an INT8 model exported on a faster development machine to run on the
Pi's CPU.
"""

import logging
import os
import threading
from typing import List, Optional, Tuple

import cv2
import numpy as np

from ai.classes import BoundingBox, DetectionResult

logger = logging.getLogger(__name__)

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter",
    "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
    "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase",
    "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
    "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
    "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet",
    "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]


def _load_interpreter(model_path: str, num_threads: int = 4):
    try:
        from ai_edge_litert.interpreter import Interpreter
    except ImportError:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            try:
                from tensorflow.lite import Interpreter
            except ImportError as exc:
                raise RuntimeError(
                    "No LiteRT runtime is installed. Install ai-edge-litert or tflite-runtime on Raspberry Pi."
                ) from exc
    interpreter = Interpreter(model_path=model_path, num_threads=num_threads)
    interpreter.allocate_tensors()
    return interpreter


class TFLiteYOLODetector:
    """Inference adapter for quantized YOLO TFLite exports on Raspberry Pi 4."""

    _lock: Optional[threading.Lock] = None

    def __init__(
        self,
        model_path: str,
        confidence: float = 0.35,
        iou: float = 0.45,
        target_classes: Optional[List[str]] = None,
        num_threads: int = 4,
    ) -> None:
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"TFLite model does not exist: {model_path}")
        self.model_path = model_path
        self.confidence = confidence
        self.iou = iou
        self.num_threads = num_threads
        self.target_classes = [c.lower() for c in target_classes] if target_classes else None
        self.interpreter = _load_interpreter(model_path, num_threads=num_threads)
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        shape = self.input_details[0]["shape"]
        self.input_height = int(shape[1])
        self.input_width = int(shape[2])
        self._lock = threading.Lock()
        self._letterbox_scale = 1.0
        self._letterbox_pad_x = 0.0
        self._letterbox_pad_y = 0.0

    @staticmethod
    def _dequantize(values: np.ndarray, detail: dict) -> np.ndarray:
        scale, zero_point = detail.get("quantization", (0.0, 0))
        if scale:
            return (values.astype(np.float32) - zero_point) * scale
        return values.astype(np.float32)

    def _prepare_input(self, frame: np.ndarray) -> np.ndarray:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # Preserve camera geometry. Directly stretching a 4:3 frame into the
        # square model input moves object boundaries and makes the decoded box
        # coordinates unsafe for corridor decisions.
        frame_h, frame_w = rgb.shape[:2]
        scale = min(self.input_width / float(frame_w), self.input_height / float(frame_h))
        resized_w = max(1, int(round(frame_w * scale)))
        resized_h = max(1, int(round(frame_h * scale)))
        resized = cv2.resize(rgb, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
        pad_x = (self.input_width - resized_w) / 2.0
        pad_y = (self.input_height - resized_h) / 2.0
        canvas = np.full((self.input_height, self.input_width, 3), 114, dtype=np.uint8)
        left = int(np.floor(pad_x))
        top = int(np.floor(pad_y))
        canvas[top:top + resized_h, left:left + resized_w] = resized
        resized = canvas
        self._letterbox_scale = scale
        self._letterbox_pad_x = pad_x
        self._letterbox_pad_y = pad_y
        tensor = resized.astype(np.float32) / 255.0
        detail = self.input_details[0]
        dtype = detail["dtype"]
        if dtype == np.uint8:
            scale, zero_point = detail.get("quantization", (1.0 / 255.0, 0))
            tensor = np.clip(tensor / scale + zero_point, 0, 255).astype(np.uint8)
        elif dtype == np.int8:
            scale, zero_point = detail.get("quantization", (1.0 / 255.0, -128))
            tensor = np.clip(tensor / scale + zero_point, -128, 127).astype(np.int8)
        else:
            tensor = tensor.astype(dtype)
        return np.ascontiguousarray(np.expand_dims(tensor, axis=0))

    def _rows(self) -> np.ndarray:
        raw = self.interpreter.get_tensor(self.output_details[0]["index"])
        output = self._dequantize(
            np.array(raw, copy=True),
            self.output_details[0],
        )
        output = np.squeeze(output)
        if output.ndim != 2:
            return np.empty((0, 6), dtype=np.float32)
        if output.shape[0] < output.shape[1] and output.shape[0] <= 128:
            output = output.T
        return output

    def detect(self, frame: np.ndarray) -> List[DetectionResult]:
        if frame is None or frame.size == 0:
            return []
        
        lock = getattr(self, "_lock", None)
        if lock is not None:
            with lock:
                self.interpreter.set_tensor(self.input_details[0]["index"], self._prepare_input(frame))
                self.interpreter.invoke()
                rows = self._rows()
        else:
            self.interpreter.set_tensor(self.input_details[0]["index"], self._prepare_input(frame))
            self.interpreter.invoke()
            rows = self._rows()

        height, width = frame.shape[:2]
        boxes, scores, class_ids = [], [], []

        if rows.ndim == 2 and rows.shape[1] == len(COCO_CLASSES) + 4:
            # Fast vectorized NumPy path for YOLOv8 (2100, 84)
            class_scores = rows[:, 4:]
            max_scores = np.max(class_scores, axis=1)
            mask = max_scores >= self.confidence

            if np.any(mask):
                filtered_rows = rows[mask]
                filtered_scores = max_scores[mask]
                filtered_class_ids = np.argmax(filtered_rows[:, 4:], axis=1)

                cx = filtered_rows[:, 0].copy()
                cy = filtered_rows[:, 1].copy()
                bw = filtered_rows[:, 2].copy()
                bh = filtered_rows[:, 3].copy()

                # Some exports emit normalized xywh; standard YOLOv8 LiteRT
                # exports emit model-input pixels. Normalize both cases to
                # model-input pixels before undoing letterbox padding.
                if np.max(np.abs(filtered_rows[:, :4])) <= 2.5:
                    cx *= self.input_width
                    cy *= self.input_height
                    bw *= self.input_width
                    bh *= self.input_height

                x1 = cx - bw / 2.0
                y1 = cy - bh / 2.0
                x2 = cx + bw / 2.0
                y2 = cy + bh / 2.0

                x1 = (x1 - self._letterbox_pad_x) / self._letterbox_scale
                y1 = (y1 - self._letterbox_pad_y) / self._letterbox_scale
                x2 = (x2 - self._letterbox_pad_x) / self._letterbox_scale
                y2 = (y2 - self._letterbox_pad_y) / self._letterbox_scale

                for i in range(len(filtered_scores)):
                    cid = int(filtered_class_ids[i])
                    if cid < 0 or cid >= len(COCO_CLASSES):
                        continue
                    name = COCO_CLASSES[cid]
                    if self.target_classes and not any(target in name for target in self.target_classes):
                        continue
                    bx1 = int(max(0, min(width, x1[i])))
                    by1 = int(max(0, min(height, y1[i])))
                    bx2 = int(min(width, max(0, x2[i])))
                    by2 = int(min(height, max(0, y2[i])))
                    boxes.append([bx1, by1, max(1, bx2 - bx1), max(1, by2 - by1)])
                    scores.append(float(filtered_scores[i]))
                    class_ids.append(cid)
        else:
            # Fallback path for non-standard output shapes
            for row in rows:
                if row.size == 6:
                    x1, y1, x2, y2, score, class_id = row
                elif row.size >= 7:
                    x, y, box_w, box_h, objectness = row[:5]
                    c_scores = row[5:]
                    class_id = int(np.argmax(c_scores))
                    score = float(objectness * c_scores[class_id])
                    x1, y1, x2, y2 = x - box_w / 2, y - box_h / 2, x + box_w / 2, y + box_h / 2
                else:
                    continue

                class_id = int(class_id)
                if score < self.confidence or class_id < 0 or class_id >= len(COCO_CLASSES):
                    continue
                name = COCO_CLASSES[class_id]
                if self.target_classes and not any(target in name for target in self.target_classes):
                    continue

                if max(abs(float(x1)), abs(float(y1)), abs(float(x2)), abs(float(y2))) <= 2.5:
                    x1 *= self.input_width
                    x2 *= self.input_width
                    y1 *= self.input_height
                    y2 *= self.input_height
                x1 = (x1 - self._letterbox_pad_x) / self._letterbox_scale
                y1 = (y1 - self._letterbox_pad_y) / self._letterbox_scale
                x2 = (x2 - self._letterbox_pad_x) / self._letterbox_scale
                y2 = (y2 - self._letterbox_pad_y) / self._letterbox_scale
                bx1 = int(max(0, min(width, x1)))
                by1 = int(max(0, min(height, y1)))
                bx2 = int(min(width, max(0, x2)))
                by2 = int(min(height, max(0, y2)))
                boxes.append([bx1, by1, max(1, bx2 - bx1), max(1, by2 - by1)])
                scores.append(float(score))
                class_ids.append(class_id)

        keep = cv2.dnn.NMSBoxes(boxes, scores, self.confidence, self.iou) if boxes else []
        detections = []
        for index in np.asarray(keep).reshape(-1).tolist():
            x, y, box_w, box_h = boxes[index]
            detections.append(DetectionResult(
                class_name=COCO_CLASSES[class_ids[index]],
                confidence=scores[index],
                bbox=BoundingBox(x1=x, y1=y, x2=x + box_w, y2=y + box_h),
            ))
        return detections
