"""
YOLOv8 Object Detection module.
Detects target objects: person, bottle, chair, car, stop sign, traffic cone.
Gracefully handles missing Ultralytics/PyTorch packages without breaking system execution.
"""

import os
import sys
import time
import threading
import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from config import YOLOConfig
    from utils.logger import car_logger
except ImportError:
    from autonomous_car.config import YOLOConfig
    from autonomous_car.utils.logger import car_logger

@dataclass
class Detection:
    class_name: str
    confidence: float
    box: Tuple[int, int, int, int]

@dataclass
class ObjectDetectionResult:
    detections: List[Detection]
    stop_sign_detected: bool
    obstacle_detected: bool
    annotated_frame: np.ndarray

class ObjectDetector:
    def __init__(self, config: YOLOConfig) -> None:
        self.config: YOLOConfig = config
        self._model = None
        self._lock = threading.Lock()
        self._latest_result: Optional[ObjectDetectionResult] = None
        self._last_inference_time: float = 0.0

        if self.config.enable_yolo:
            self._init_yolo_model()
        else:
            car_logger.info("YOLO Object Detection disabled in config (Lightweight Mode).")

    def _init_yolo_model(self) -> None:
        try:
            from ultralytics import YOLO
            car_logger.info(f"Loading YOLO model '{self.config.model_name}'...")
            self._model = YOLO(self.config.model_name)
            car_logger.info("YOLOv8 Model loaded successfully.")
        except Exception as e:
            car_logger.warning(f"YOLO model init skipped ({e}). Object detection running in fallback mode.")
            self._model = None

    def detect(self, bgr_frame: np.ndarray, draw_boxes: bool = True) -> ObjectDetectionResult:
        if bgr_frame is None or bgr_frame.size == 0 or self._model is None:
            annotated = bgr_frame.copy() if bgr_frame is not None else np.zeros((480, 640, 3), dtype=np.uint8)
            return ObjectDetectionResult(
                detections=[],
                stop_sign_detected=False,
                obstacle_detected=False,
                annotated_frame=annotated
            )

        annotated_frame = bgr_frame.copy()
        detections: List[Detection] = []
        stop_sign_detected = False
        obstacle_detected = False

        try:
            results = self._model(
                bgr_frame,
                conf=self.config.confidence_threshold,
                verbose=False
            )

            if results and len(results) > 0:
                result = results[0]
                boxes = result.boxes

                for box in boxes:
                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())
                    cls_name = self._model.names.get(cls_id, str(cls_id)).lower()

                    if self.config.target_classes:
                        target_matched = any(t.lower() in cls_name for t in self.config.target_classes)
                        if not target_matched:
                            continue

                    xyxy = box.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                    
                    det = Detection(class_name=cls_name, confidence=conf, box=(x1, y1, x2, y2))
                    detections.append(det)

                    if "stop sign" in cls_name:
                        stop_sign_detected = True

                    if cls_name in ["person", "car", "chair", "bottle", "traffic cone"]:
                        obstacle_detected = True

                    if draw_boxes:
                        color = (0, 0, 255) if "stop" in cls_name else (255, 165, 0)
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                        label = f"{cls_name} {conf:.2f}"
                        cv2.putText(
                            annotated_frame,
                            label,
                            (x1, max(y1 - 10, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            color,
                            2
                        )

        except Exception as e:
            car_logger.error(f"YOLO inference error: {e}")

        res = ObjectDetectionResult(
            detections=detections,
            stop_sign_detected=stop_sign_detected,
            obstacle_detected=obstacle_detected,
            annotated_frame=annotated_frame
        )

        with self._lock:
            self._latest_result = res

        return res

    def detect_async(self, bgr_frame: np.ndarray) -> Optional[ObjectDetectionResult]:
        if not self.config.enable_yolo or self._model is None:
            return None

        now = time.perf_counter()
        if now - self._last_inference_time >= self.config.inference_interval_sec:
            self._last_inference_time = now
            thread = threading.Thread(target=self.detect, args=(bgr_frame, True), daemon=True)
            thread.start()

        with self._lock:
            return self._latest_result
