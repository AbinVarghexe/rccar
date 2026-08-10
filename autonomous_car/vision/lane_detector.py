"""
Lane detection module for autonomous vehicle navigation.
"""

import os
import sys
import cv2
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Tuple, Optional, List

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from utils.fps import FPSCounter
except ImportError:
    from autonomous_car.utils.fps import FPSCounter

class Direction(str, Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STRAIGHT = "STRAIGHT"
    NO_LANE = "NO LANE"

@dataclass
class LaneResult:
    lane_center_x: Optional[int]
    lane_center_y: Optional[int]
    image_center_x: int
    error: float
    direction: Direction
    contour_found: bool
    largest_contour: Optional[np.ndarray]
    annotated_frame: np.ndarray

class LaneDetector:
    def __init__(self, deadzone_pixels: int = 15) -> None:
        self.deadzone_pixels: int = deadzone_pixels
        self.fps_counter: FPSCounter = FPSCounter().start()

    def detect(self, binary_img: np.ndarray, original_bgr_frame: np.ndarray) -> LaneResult:
        self.fps_counter.update()
        fps = self.fps_counter.get_fps()

        height, width = binary_img.shape[:2]
        image_center_x = width // 2

        annotated_frame = original_bgr_frame.copy()

        contours, _ = cv2.findContours(
            binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        lane_center_x: Optional[int] = None
        lane_center_y: Optional[int] = None
        largest_contour: Optional[np.ndarray] = None
        contour_found: bool = False

        if contours:
            min_area = (width * height) * 0.002
            valid_contours = [c for c in contours if cv2.contourArea(c) >= min_area]

            if valid_contours:
                largest_contour = max(valid_contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    lane_center_x = int(M["m10"] / M["m00"])
                    lane_center_y = int(M["m01"] / M["m00"])
                    contour_found = True

        if contour_found and lane_center_x is not None and lane_center_y is not None:
            error = float(lane_center_x - image_center_x)

            if abs(error) <= self.deadzone_pixels:
                direction = Direction.STRAIGHT
            elif error < -self.deadzone_pixels:
                direction = Direction.LEFT
            else:
                direction = Direction.RIGHT
        else:
            error = 0.0
            direction = Direction.NO_LANE

        self._annotate_hud(
            annotated_frame=annotated_frame,
            image_center_x=image_center_x,
            lane_center_x=lane_center_x,
            lane_center_y=lane_center_y,
            error=error,
            direction=direction,
            contour=largest_contour,
            fps=fps
        )

        return LaneResult(
            lane_center_x=lane_center_x,
            lane_center_y=lane_center_y,
            image_center_x=image_center_x,
            error=error,
            direction=direction,
            contour_found=contour_found,
            largest_contour=largest_contour,
            annotated_frame=annotated_frame
        )

    def _annotate_hud(
        self,
        annotated_frame: np.ndarray,
        image_center_x: int,
        lane_center_x: Optional[int],
        lane_center_y: Optional[int],
        error: float,
        direction: Direction,
        contour: Optional[np.ndarray],
        fps: float
    ) -> None:
        height, width = annotated_frame.shape[:2]

        cv2.line(annotated_frame, (image_center_x, 0), (image_center_x, height), (255, 120, 0), 2)

        if contour is not None:
            cv2.drawContours(annotated_frame, [contour], -1, (0, 255, 0), 2)

        if lane_center_x is not None and lane_center_y is not None:
            cv2.circle(annotated_frame, (lane_center_x, lane_center_y), 7, (0, 0, 255), -1)
            cv2.line(
                annotated_frame,
                (image_center_x, lane_center_y),
                (lane_center_x, lane_center_y),
                (0, 255, 255),
                3
            )

        dir_color = (0, 255, 0)
        if direction == Direction.LEFT or direction == Direction.RIGHT:
            dir_color = (0, 215, 255)
        elif direction == Direction.NO_LANE:
            dir_color = (0, 0, 255)

        cv2.rectangle(annotated_frame, (10, 10), (300, 110), (0, 0, 0), -1)
        cv2.rectangle(annotated_frame, (10, 10), (300, 110), (255, 255, 255), 1)

        cv2.putText(annotated_frame, f"DIR: {direction.value}", (20, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.7, dir_color, 2)
        cv2.putText(annotated_frame, f"ERROR: {error:+.1f} px", (20, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(annotated_frame, f"FPS: {fps:.1f}", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
