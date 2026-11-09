import cv2
import numpy as np
from typing import Dict, Tuple, Optional, List, Any

try:
    from ai.wall_detector import WallAndObstacleDetector
except ImportError:
    from autonomous_car.ai.wall_detector import WallAndObstacleDetector

class CorridorScanner:
    """
    Real-time Computer Vision spatial corridor analyzer.
    Evaluates driving path clearance, detects foreground obstructions,
    and calculates open directional paths (Left vs Right).
    """

    def __init__(self, lane_width_ratio: float = 0.42):
        self.lane_width_ratio = lane_width_ratio
        self.wall_detector = WallAndObstacleDetector(lane_width_ratio=lane_width_ratio)

    def analyze(self, bgr_frame: np.ndarray) -> Dict[str, any]:
        """
        Analyzes the frame to find obstacles and determine road sector clearance.
        Returns:
            {
                "center_blocked": bool,
                "obstacle_side": "left" | "right" | "center" | None,
                "left_clearance": float (0.0 to 1.0),
                "right_clearance": float (0.0 to 1.0),
                "center_clearance": float (0.0 to 1.0),
                "best_direction": "LEFT" | "RIGHT" | "FORWARD"
            }
        """
        if bgr_frame is None or bgr_frame.size == 0:
            return {
                "center_blocked": False,
                "obstacle_side": None,
                "left_clearance": 1.0,
                "right_clearance": 1.0,
                "center_clearance": 1.0,
                "best_direction": "FORWARD"
            }

        h, w = bgr_frame.shape[:2]

        # Analyze bottom 55% of frame (the road surface & immediate driving path)
        roi_top = int(h * 0.45)
        roi = bgr_frame[roi_top:h, :]
        roi_h = roi.shape[0]

        # Convert to grayscale and blur
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detect high-contrast edges and foreground obstacles
        edges = cv2.Canny(blurred, 65, 175)

        # Slight morphological dilation to connect thin obstacle edges without engulfing scene
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        dilated = cv2.dilate(edges, kernel, iterations=1)

        # Split width into sectors
        lane_margin = (1.0 - self.lane_width_ratio) / 2.0
        c_start = int(w * lane_margin)
        c_end = int(w * (1.0 - lane_margin))

        left_mask = dilated[:, :c_start]
        center_mask = dilated[:, c_start:c_end]
        right_mask = dilated[:, c_end:]

        # Calculate density / occupancy of obstacles in each sector
        # (Lower density = higher clearance)
        left_area = max(1, left_mask.size)
        center_area = max(1, center_mask.size)
        right_area = max(1, right_mask.size)

        left_density = float(np.count_nonzero(left_mask) / left_area)
        center_density = float(np.count_nonzero(center_mask) / center_area)
        right_density = float(np.count_nonzero(right_mask) / right_area)

        # Invert to compute clearance (1.0 = completely open, 0.0 = completely obstructed)
        left_clearance = max(0.0, min(1.0, 1.0 - (left_density * 4.5)))
        center_clearance = max(0.0, min(1.0, 1.0 - (center_density * 4.5)))
        right_clearance = max(0.0, min(1.0, 1.0 - (right_density * 4.5)))

        # Center path is considered blocked if center_density exceeds obstacle threshold
        center_blocked = center_density > 0.18 or center_clearance < 0.20

        # Check for large unclassified obstacles (boxes, barriers, walls) in corridor
        large_obs = self.detect_large_obstacles(bgr_frame)
        for obs in large_obs:
            cx = obs["center_x"]
            if c_start <= cx <= c_end:
                center_blocked = True
                center_clearance = min(center_clearance, 0.10)

        # Identify obstacle side if any
        obstacle_side = None
        if center_blocked:
            obstacle_side = "center"
        elif left_density > right_density and left_clearance < 0.40:
            obstacle_side = "left"
        elif right_density > left_density and right_clearance < 0.40:
            obstacle_side = "right"

        # Determine best direction
        if center_blocked:
            best_direction = "LEFT" if left_clearance >= right_clearance else "RIGHT"
        elif right_density > 0.08 and left_clearance > right_clearance + 0.15:
            best_direction = "LEFT"
        elif left_density > 0.08 and right_clearance > left_clearance + 0.15:
            best_direction = "RIGHT"
        elif center_clearance > 0.45:
            best_direction = "FORWARD"
        else:
            best_direction = "LEFT" if left_clearance >= right_clearance else "RIGHT"

        return {
            "center_blocked": center_blocked,
            "obstacle_side": obstacle_side,
            "left_clearance": round(float(left_clearance), 2),
            "right_clearance": round(float(right_clearance), 2),
            "center_clearance": round(float(center_clearance), 2),
            "best_direction": best_direction
        }

    def measure_pan_clearance(self, bgr_frame: np.ndarray) -> float:
        """
        Measures the overall open road clearance of a frame captured
        during camera pan survey (0.0 to 1.0).
        """
        if bgr_frame is None or bgr_frame.size == 0:
            return 0.5

        h, w = bgr_frame.shape[:2]
        roi = bgr_frame[int(h * 0.35):, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 60, 160)
        edge_density = np.count_nonzero(edges) / max(1, edges.size)

        # Clearance is inverse of obstruction edge density
        clearance = max(0.0, min(1.0, 1.0 - (edge_density * 7.5)))

        # Evaluate ground-plane wall horizon during camera pan
        try:
            wall_info = self.wall_detector.analyze_free_space(bgr_frame)
            if wall_info.get("wall_ahead", False):
                dist = wall_info.get("wall_distance_cm")
                if dist is not None:
                    wall_clr = max(0.05, min(1.0, (dist - 15.0) / 75.0))
                    clearance = min(clearance, wall_clr)
                else:
                    clearance = min(clearance, 0.15)
        except Exception:
            pass

        return round(float(clearance), 2)

    def detect_large_obstacles(self, bgr_frame: np.ndarray) -> list:
        """
        Detects unclassified large physical obstacles, barriers, boxes, and walls
        that may not be part of the standard 80 COCO neural network classes.
        Uses specialized ground-plane horizon segmentation + edge contour detection.
        Returns bounding boxes and area confidence of large obstacles.
        """
        if bgr_frame is None or bgr_frame.size == 0:
            return []

        h, w = bgr_frame.shape[:2]
        results = []

        # 1. Specialized Wall & Boundary Horizon Detection
        try:
            wall_dets = self.wall_detector.detect(bgr_frame)
            for wd in wall_dets:
                bx1, by1, bx2, by2 = int(wd.bbox.x1), int(wd.bbox.y1), int(wd.bbox.x2), int(wd.bbox.y2)
                bw = bx2 - bx1
                bh = by2 - by1
                area_ratio = min(0.99, (bw * bh) / float(w * h))
                results.append({
                    "class_name": wd.class_name,
                    "confidence": wd.confidence,
                    "bbox": [bx1, by1, bx2, by2],
                    "area_ratio": round(float(area_ratio), 3),
                    "center_x": (bx1 + bx2) / 2.0,
                })
        except Exception:
            pass

        # 2. Multi-cue foreground edge connected contours (only if wall_detector found nothing)
        if not results:
            roi_top = int(h * 0.35)
            roi = bgr_frame[roi_top:h, :]
            roi_h = roi.shape[0]

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            edges = cv2.Canny(blurred, 45, 140)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            dilated = cv2.dilate(edges, kernel, iterations=1)
            dilated = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            min_obstacle_area = (w * roi_h) * 0.06
            max_obstacle_area = (w * roi_h) * 0.50

            for c in contours:
                area = cv2.contourArea(c)
                if min_obstacle_area <= area <= max_obstacle_area:
                    bx, by, bw, bh = cv2.boundingRect(c)
                    if bw >= w * 0.75 or bh >= roi_h * 0.85:
                        continue
                    if (by + bh) < int(roi_h * 0.30):
                        continue
                    if bw >= 45 and bh >= 40:
                        full_y1 = roi_top + by
                        full_y2 = full_y1 + bh
                        full_x1 = bx
                        full_x2 = bx + bw
                        area_ratio = min(0.99, area / (w * roi_h))
                        results.append({
                            "class_name": "obstacle",
                            "confidence": round(float(0.50 + (area_ratio * 0.45)), 2),
                            "bbox": [full_x1, full_y1, full_x2, full_y2],
                            "area_ratio": round(float(area_ratio), 3),
                            "center_x": (full_x1 + full_x2) / 2.0,
                        })

        results.sort(key=lambda x: x["area_ratio"], reverse=True)
        return results[:4]
