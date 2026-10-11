import time
import logging
from typing import Dict, List, Tuple
from autonomous_car.ai.classes import DetectionResult, BoundingBox

logger = logging.getLogger(__name__)

class TrackHistory:
    """Stores history of bounding boxes and timestamps for a single tracked object to compute TTC."""
    def __init__(self, max_history: int = 15):
        self.boxes: List[Tuple[BoundingBox, float]] = []
        self.max_history = max_history

    def add(self, bbox: BoundingBox, timestamp: float) -> None:
        self.boxes.append((bbox, timestamp))
        if len(self.boxes) > self.max_history:
            self.boxes.pop(0)

    def get_ttc(self) -> float:
        """
        Estimate Time-To-Collision (TTC) in seconds using bounding box looming.
        TTC = width / (d_width / dt)
        """
        if len(self.boxes) < 4:
            return float('inf')

        first_box, first_time = self.boxes[0]
        last_box, last_time = self.boxes[-1]

        dt = last_time - first_time
        if dt <= 0.001:
            return float('inf')

        dw = last_box.width - first_box.width
        if dw > 2.0:  # Pixel expansion threshold
            growth_rate = dw / dt
            if growth_rate > 0:
                ttc = last_box.width / growth_rate
                return max(0.0, ttc)

        return float('inf')

    def get_lateral_velocity(self) -> float:
        """Calculate lateral velocity in pixels/second. Negative = moving left, Positive = moving right."""
        if len(self.boxes) < 2:
            return 0.0

        first_box, first_time = self.boxes[0]
        last_box, last_time = self.boxes[-1]

        dt = last_time - first_time
        if dt <= 0.001:
            return 0.0

        return (last_box.center_x - first_box.center_x) / dt


class RiskAnalyzer:
    """Analyzes driving hazards, approaching speeds, and assigns risk scores to tracked objects."""
    def __init__(self):
        self.histories: Dict[int, TrackHistory] = {}

    def update(self, detections: List[DetectionResult], timestamp: float, frame_width: int = 640, frame_height: int = 480) -> Dict[int, Dict[str, float]]:
        """
        Update tracking history and compute risk parameters for each object.
        Returns:
            Dict mapping track_id -> {"risk_score": float, "ttc": float, "lateral_velocity": float}
        """
        frame_area = max(1, frame_width * frame_height)
        current_track_ids = {d.track_id for d in detections if d.track_id is not None}
        self.histories = {tid: hist for tid, hist in self.histories.items() if tid in current_track_ids}

        risks: Dict[int, Dict[str, float]] = {}

        for d in detections:
            tid = d.track_id if d.track_id is not None else id(d)
            if tid not in self.histories:
                self.histories[tid] = TrackHistory()

            hist = self.histories[tid]
            hist.add(d.bbox, timestamp)

            ttc = hist.get_ttc()
            v_x = hist.get_lateral_velocity()

            class_lower = d.class_name.lower()

            # 1. Base class danger level (bus is maximum priority vehicle)
            if "bus" in class_lower:
                base_risk = 0.90
            elif any(c in class_lower for c in ["person", "pedestrian", "child", "dog", "cat"]):
                base_risk = 0.85
            elif any(c in class_lower for c in ["car", "truck", "motorcycle", "bicycle"]):
                base_risk = 0.70
            elif "stop" in class_lower:
                base_risk = 0.65
            elif any(c in class_lower for c in ["cone", "chair", "bottle", "box"]):
                base_risk = 0.50
            else:
                base_risk = 0.40

            # 2. Proximity factor (relative area in frame)
            norm_area = d.bbox.area / frame_area
            proximity_risk = min(1.0, norm_area * 5.0)

            # 3. Collision threat factor based on TTC
            ttc_risk = 0.0
            if ttc < 1.5:
                ttc_risk = 0.95
            elif ttc < 3.0:
                ttc_risk = 0.75
            elif ttc < 5.0:
                ttc_risk = 0.40

            risk_score = max(base_risk * 0.7 + proximity_risk * 0.3, proximity_risk, ttc_risk)
            risk_score = min(1.0, max(0.0, risk_score))

            risks[tid] = {
                "risk_score": risk_score,
                "ttc": ttc,
                "lateral_velocity": v_x,
                "norm_area": norm_area
            }

        return risks
