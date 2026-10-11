import os
import sys
import logging
from typing import List, Optional
import numpy as np

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from autonomous_car.ai.classes import DetectionResult, BoundingBox

logger = logging.getLogger(__name__)

# Check Ultralytics availability
_ULTRALYTICS_AVAILABLE = False
try:
    from ultralytics import YOLO
    _ULTRALYTICS_AVAILABLE = True
except ImportError:
    pass


class YOLODetector:
    """YOLO Object Detector wrapping Ultralytics model inference."""

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence: float = 0.35,
        iou: float = 0.45,
        target_classes: Optional[List[str]] = None,
    ):
        self.model_path = model_path
        self.confidence = confidence
        self.iou = iou
        self.target_classes = [c.lower() for c in target_classes] if target_classes else None
        self.model = None
        self.is_initialized = False

        # Resolve model path relative to car directory
        candidate_paths = [
            model_path,
            os.path.join(parent_dir, model_path),
            os.path.join(parent_dir, "yolov8n.pt"),
            os.path.join(os.path.dirname(parent_dir), "self-driving-ai", "yolov8n.pt")
        ]
        resolved_path = model_path
        for p in candidate_paths:
            if os.path.exists(p):
                resolved_path = p
                break

        self.model_path = resolved_path
        self._load_model()

    def _load_model(self) -> None:
        if not _ULTRALYTICS_AVAILABLE:
            logger.warning("[AI DETECTOR] Ultralytics not installed. Detector will run in fallback CV mode.")
            return

        try:
            logger.info(f"[AI DETECTOR] Loading YOLO model from '{self.model_path}'...")
            self.model = YOLO(self.model_path)
            self.is_initialized = True
            logger.info(f"[AI DETECTOR] YOLO model loaded successfully from {self.model_path}")
        except Exception as e:
            logger.error(f"[AI DETECTOR] Failed to load YOLO model: {e}")
            self.model = None
            self.is_initialized = False

    def detect(self, bgr_frame: np.ndarray) -> List[DetectionResult]:
        """Runs object detection on a BGR numpy frame."""
        if not self.is_initialized or self.model is None or bgr_frame is None:
            return []

        try:
            results = self.model.predict(
                source=bgr_frame,
                conf=self.confidence,
                iou=self.iou,
                verbose=False
            )

            detections: List[DetectionResult] = []
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                for box in boxes:
                    xyxy = box.xyxy[0].cpu().numpy().tolist()
                    conf = float(box.conf[0].cpu().item())
                    cls_id = int(box.cls[0].cpu().item())
                    class_name = self.model.names[cls_id].lower()

                    if self.target_classes:
                        # Match target classes (e.g., bus, car, person, etc.)
                        if not any(t in class_name for t in self.target_classes):
                            continue

                    detections.append(DetectionResult(
                        class_name=class_name,
                        confidence=conf,
                        bbox=BoundingBox(x1=xyxy[0], y1=xyxy[1], x2=xyxy[2], y2=xyxy[3]),
                        track_id=None
                    ))

            return detections
        except Exception as e:
            logger.error(f"[AI DETECTOR] Error during inference: {e}")
            return []
