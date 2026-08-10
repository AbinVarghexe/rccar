"""
Junction detection module for detecting track intersections and road topologies.
Identifies Left Turn, Right Turn, T Junction, Cross Road, and Dead End conditions.
"""

import cv2
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Optional, List, Tuple

class JunctionType(str, Enum):
    NONE = "NONE"
    LEFT_TURN = "LEFT_TURN"
    RIGHT_TURN = "RIGHT_TURN"
    T_JUNCTION = "T_JUNCTION"
    CROSS_ROAD = "CROSS_ROAD"
    DEAD_END = "DEAD_END"

@dataclass
class JunctionResult:
    junction_type: JunctionType
    confidence: float
    branch_count: int
    has_left_branch: bool
    has_right_branch: bool
    has_forward_branch: bool

class JunctionDetector:
    """
    Analyzes binary image layout to classify road junctions:
    - LEFT_TURN: Path extends primarily to the left
    - RIGHT_TURN: Path extends primarily to the right
    - T_JUNCTION: Perpendicular horizontal line crossing without continuous forward path
    - CROSS_ROAD: 4-way intersection (forward + left + right)
    - DEAD_END: No track pixels detected in upper ROI
    """
    def __init__(self, min_pixel_threshold: int = 50) -> None:
        self.min_pixel_threshold: int = min_pixel_threshold

    def detect(self, binary_img: np.ndarray) -> JunctionResult:
        """
        Classifies junction type from binary ROI image.
        """
        if binary_img is None or binary_img.size == 0:
            return JunctionResult(JunctionType.NONE, 0.0, 0, False, False, False)

        height, width = binary_img.shape[:2]

        # Divide ROI into 3 vertical zones: Top (Forward), Left, Right
        top_zone = binary_img[0 : int(height * 0.4), int(width * 0.3) : int(width * 0.7)]
        left_zone = binary_img[int(height * 0.3) : int(height * 0.9), 0 : int(width * 0.3)]
        right_zone = binary_img[int(height * 0.3) : int(height * 0.9), int(width * 0.7) : width]
        bottom_zone = binary_img[int(height * 0.7) : height, int(width * 0.3) : int(width * 0.7)]

        # Count active white track pixels in each zone
        top_count = np.count_nonzero(top_zone)
        left_count = np.count_nonzero(left_zone)
        right_count = np.count_nonzero(right_zone)
        bottom_count = np.count_nonzero(bottom_zone)
        total_count = np.count_nonzero(binary_img)

        # Normalize pixel thresholds relative to zone size
        has_forward = top_count > (top_zone.size * 0.08)
        has_left = left_count > (left_zone.size * 0.12)
        has_right = right_count > (right_zone.size * 0.12)
        has_entry = bottom_count > (bottom_zone.size * 0.05)

        branch_count = sum([has_forward, has_left, has_right])

        # Classification Logic
        if total_count < self.min_pixel_threshold:
            junction_type = JunctionType.DEAD_END
            confidence = 1.0

        elif has_left and has_right and has_forward:
            junction_type = JunctionType.CROSS_ROAD
            confidence = 0.95

        elif has_left and has_right and not has_forward:
            junction_type = JunctionType.T_JUNCTION
            confidence = 0.90

        elif has_left and not has_right and not has_forward:
            junction_type = JunctionType.LEFT_TURN
            confidence = 0.85

        elif has_right and not has_left and not has_forward:
            junction_type = JunctionType.RIGHT_TURN
            confidence = 0.85

        else:
            junction_type = JunctionType.NONE
            confidence = 0.70

        return JunctionResult(
            junction_type=junction_type,
            confidence=confidence,
            branch_count=branch_count,
            has_left_branch=has_left,
            has_right_branch=has_right,
            has_forward_branch=has_forward
        )
