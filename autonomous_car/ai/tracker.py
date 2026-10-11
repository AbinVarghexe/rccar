import logging
from typing import List, Dict, Optional
import numpy as np
from autonomous_car.ai.classes import DetectionResult, BoundingBox
from autonomous_car.ai.detector import YOLODetector

logger = logging.getLogger(__name__)

class ObjectTracker:
    """Object Tracker wrapper using ByteTrack via Ultralytics, with centroid fallback."""
    def __init__(self, detector: YOLODetector):
        self.detector = detector
        self._next_id = 1
        self._tracked_objects: Dict[int, BoundingBox] = {}

    def track(self, bgr_frame: np.ndarray) -> List[DetectionResult]:
        if not self.detector.is_initialized or self.detector.model is None or bgr_frame is None:
            return []

        try:
            # 1. Try native Ultralytics ByteTrack
            results = self.detector.model.track(
                source=bgr_frame,
                persist=True,
                conf=self.detector.confidence,
                iou=self.detector.iou,
                tracker="bytetrack.yaml",
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
                    class_name = self.detector.model.names[cls_id].lower()

                    if self.detector.target_classes:
                        if not any(t in class_name for t in self.detector.target_classes):
                            continue

                    track_id = None
                    if box.id is not None:
                        track_id = int(box.id[0].cpu().item())

                    detections.append(DetectionResult(
                        class_name=class_name,
                        confidence=conf,
                        bbox=BoundingBox(x1=xyxy[0], y1=xyxy[1], x2=xyxy[2], y2=xyxy[3]),
                        track_id=track_id
                    ))

            # Assign fallback IDs if ByteTrack didn't assign one
            for d in detections:
                if d.track_id is None:
                    d.track_id = self._assign_fallback_id(d.bbox)

            return detections
        except Exception as e:
            # Fallback to detector.detect
            logger.debug(f"[TRACKER] ByteTrack fallback to detect: {e}")
            detections = self.detector.detect(bgr_frame)
            for d in detections:
                d.track_id = self._assign_fallback_id(d.bbox)
            return detections

    def _assign_fallback_id(self, bbox: BoundingBox) -> int:
        best_id = None
        best_dist = 60.0  # Pixel threshold
        for tid, prev_box in list(self._tracked_objects.items()):
            dist = ((bbox.center_x - prev_box.center_x)**2 + (bbox.center_y - prev_box.center_y)**2)**0.5
            if dist < best_dist:
                best_dist = dist
                best_id = tid

        if best_id is None:
            best_id = self._next_id
            self._next_id += 1

        self._tracked_objects[best_id] = bbox
        if len(self._tracked_objects) > 50:
            self._tracked_objects.pop(next(iter(self._tracked_objects)))
        return best_id
