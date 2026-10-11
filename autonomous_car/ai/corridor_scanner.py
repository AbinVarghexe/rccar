import cv2
import numpy as np
from typing import Dict, Tuple, Optional

class CorridorScanner:
    """
    Real-time Computer Vision spatial corridor analyzer.
    Evaluates driving path clearance, detects foreground obstructions,
    and calculates open directional paths (Left vs Right).
    """

    def __init__(self, lane_width_ratio: float = 0.42):
        self.lane_width_ratio = lane_width_ratio

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

        # Analyze bottom 60% of frame (the road surface & immediate driving path)
        roi_top = int(h * 0.40)
        roi = bgr_frame[roi_top:h, :]
        roi_h = roi.shape[0]

        # Convert to grayscale and blur
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detect high-contrast edges and foreground contours
        edges = cv2.Canny(blurred, 50, 150)

        # Morphological close to connect dense obstacle textures
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

        # Split width into sectors
        lane_margin = (1.0 - self.lane_width_ratio) / 2.0
        c_start = int(w * lane_margin)
        c_end = int(w * (1.0 - lane_margin))

        left_mask = closed[:, :c_start]
        center_mask = closed[:, c_start:c_end]
        right_mask = closed[:, c_end:]

        # Calculate density / occupancy of obstacles in each sector
        # (Lower density = higher clearance)
        left_area = max(1, left_mask.size)
        center_area = max(1, center_mask.size)
        right_area = max(1, right_mask.size)

        left_density = np.count_nonzero(left_mask) / left_area
        center_density = np.count_nonzero(center_mask) / center_area
        right_density = np.count_nonzero(right_mask) / right_area

        # Invert to compute clearance (1.0 = completely open, 0.0 = completely obstructed)
        left_clearance = max(0.0, min(1.0, 1.0 - (left_density * 8.0)))
        center_clearance = max(0.0, min(1.0, 1.0 - (center_density * 8.0)))
        right_clearance = max(0.0, min(1.0, 1.0 - (right_density * 8.0)))

        # Center path is considered blocked if center_density exceeds threshold
        center_blocked = center_density > 0.08

        # Identify obstacle side if any
        obstacle_side = None
        if center_blocked:
            obstacle_side = "center"
        elif left_density > right_density and left_density > 0.07:
            obstacle_side = "left"
        elif right_density > left_density and right_density > 0.07:
            obstacle_side = "right"

        # Determine best direction
        if not center_blocked and center_clearance > 0.5:
            best_direction = "FORWARD"
        elif left_clearance > right_clearance and left_clearance > 0.4:
            best_direction = "LEFT"
        elif right_clearance > left_clearance and right_clearance > 0.4:
            best_direction = "RIGHT"
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
        return round(float(clearance), 2)
