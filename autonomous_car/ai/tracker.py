import logging
import time
from typing import List, Dict, Optional, Tuple
import numpy as np
from ai.classes import DetectionResult, BoundingBox
from ai.detector import YOLODetector

logger = logging.getLogger(__name__)

class ObjectTracker:
    """Object Tracker with persistent IDs, IoU + Centroid matching, and trajectory trails."""
    def __init__(self, detector: YOLODetector, min_inference_interval_sec: float = 0.0):
        self.detector = detector
        self.min_inference_interval_sec = max(0.0, float(min_inference_interval_sec))
        self._last_inference_at = 0.0
        self._last_detections: List[DetectionResult] = []
        self._next_id = 1
        # Track state: id -> {"bbox": BoundingBox, "class_name": str, "missed": int, "trail": [(x, y)]}
        self.tracks: Dict[int, dict] = {}

    @staticmethod
    def _compute_iou(bb1: BoundingBox, bb2: BoundingBox) -> float:
        x_left = max(bb1.x1, bb2.x1)
        y_top = max(bb1.y1, bb2.y1)
        x_right = min(bb1.x2, bb2.x2)
        y_bottom = min(bb1.y2, bb2.y2)

        if x_right < x_left or y_bottom < y_top:
            return 0.0

        intersection_area = (x_right - x_left) * (y_bottom - y_top)
        bb1_area = bb1.area
        bb2_area = bb2.area
        union = bb1_area + bb2_area - intersection_area
        return intersection_area / union if union > 0 else 0.0

    def track(self, bgr_frame: np.ndarray) -> List[DetectionResult]:
        if not self.detector.is_initialized or bgr_frame is None:
            return []

        now = time.monotonic()
        if (
            self.min_inference_interval_sec > 0.0
            and self._last_inference_at > 0.0
            and now - self._last_inference_at < self.min_inference_interval_sec
        ):
            return list(self._last_detections)

        detections = self.detector.detect(bgr_frame)
        self._last_inference_at = now
        self._last_detections = list(detections)

        # Match detections to existing tracks
        matched_tracks = set()
        matched_dets = set()

        # Step 1: Match by IoU & class name
        for d_idx, d in enumerate(detections):
            best_iou = 0.25
            best_tid = None
            for tid, trk in self.tracks.items():
                if tid in matched_tracks:
                    continue
                if trk["class_name"] != d.class_name:
                    continue
                iou = self._compute_iou(d.bbox, trk["bbox"])
                if iou > best_iou:
                    best_iou = iou
                    best_tid = tid

            if best_tid is not None:
                d.track_id = best_tid
                matched_tracks.add(best_tid)
                matched_dets.add(d_idx)
                # Update track
                self.tracks[best_tid]["bbox"] = d.bbox
                self.tracks[best_tid]["missed"] = 0
                trail = self.tracks[best_tid]["trail"]
                trail.append((int(d.bbox.center_x), int(d.bbox.center_y)))
                if len(trail) > 20:
                    trail.pop(0)

        # Step 2: Match remaining by centroid proximity (< 120 pixels)
        for d_idx, d in enumerate(detections):
            if d_idx in matched_dets:
                continue
            best_dist = 120.0
            best_tid = None
            for tid, trk in self.tracks.items():
                if tid in matched_tracks:
                    continue
                dist = ((d.bbox.center_x - trk["bbox"].center_x)**2 + (d.bbox.center_y - trk["bbox"].center_y)**2)**0.5
                if dist < best_dist:
                    best_dist = dist
                    best_tid = tid

            if best_tid is not None:
                d.track_id = best_tid
                matched_tracks.add(best_tid)
                matched_dets.add(d_idx)
                self.tracks[best_tid]["bbox"] = d.bbox
                self.tracks[best_tid]["missed"] = 0
                trail = self.tracks[best_tid]["trail"]
                trail.append((int(d.bbox.center_x), int(d.bbox.center_y)))
                if len(trail) > 20:
                    trail.pop(0)

        # Step 3: Create new tracks for unmatched detections
        for d_idx, d in enumerate(detections):
            if d_idx not in matched_dets:
                new_id = self._next_id
                self._next_id += 1
                d.track_id = new_id
                matched_tracks.add(new_id)
                self.tracks[new_id] = {
                    "bbox": d.bbox,
                    "class_name": d.class_name,
                    "missed": 0,
                    "trail": [(int(d.bbox.center_x), int(d.bbox.center_y))]
                }

        # Step 4: Increment missed counter for unobserved tracks and prune stale ones
        stale_ids = []
        for tid in list(self.tracks.keys()):
            if tid not in matched_tracks:
                self.tracks[tid]["missed"] += 1
                if self.tracks[tid]["missed"] > 10:  # Prune after 10 missed frames (~0.7s)
                    stale_ids.append(tid)

        for tid in stale_ids:
            del self.tracks[tid]

        return detections

    def get_trails(self) -> Dict[int, List[Tuple[int, int]]]:
        """Returns the recent (x, y) centroid history for all active tracks."""
        return {tid: list(trk["trail"]) for tid, trk in self.tracks.items()}
