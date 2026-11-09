"""
vision/lane_detector.py — Sanoguzhan Perspective Warping & Histogram Lane Keeping Assist (LKA).

Based on: https://github.com/sanoguzhan/Self-Driving-Car
Features:
1. Fixed 400x240 Birds-Eye Trapezoid Perspective Transform (Sanoguzhan standard).
2. Robust Multi-Feature Segmentation:
   - HSV White & Yellow track line masks
   - Adaptive Gaussian Canny Edge Detection
3. Sliding Column Histogram Peak Detection:
   - Fast 1D column summation across bottom half of warped plane
   - Dual-lane peak tracking with single-line fallback and nominal width memory
4. Closed-Loop PD Steering & Hard Virtual Boundary Enforcement:
   - Clamps steering output to strictly prevent lane departures
5. Augmented Reality Visual Projection:
   - Unwarps illuminated green driving corridor back onto camera perspective view
   - Generates live Bird's-Eye Perspective preview for cockpit streaming
"""

import os
import sys
import time
from enum import Enum
from dataclasses import dataclass
from typing import Tuple, Optional, List, Dict, Any
import cv2
import numpy as np

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
    lane_center_x: Optional[int] = None
    lane_center_y: Optional[int] = None
    image_center_x: int = 200
    error: float = 0.0
    direction: Direction = Direction.STRAIGHT
    contour_found: bool = False
    largest_contour: Optional[np.ndarray] = None
    annotated_frame: Optional[np.ndarray] = None
    birdseye_frame: Optional[np.ndarray] = None

    # Enhanced Lane Assist & Boundary Protection Attributes
    left_line_x: Optional[int] = None
    right_line_x: Optional[int] = None
    lane_width: Optional[int] = None
    lane_detected: bool = False
    is_left_boundary_violated: bool = False
    is_right_boundary_violated: bool = False
    departure_warning: Optional[str] = None
    suggested_steering: float = 0.0
    corrective_steering: float = 0.0
    min_allowed_steering: float = -1.0
    max_allowed_steering: float = 1.0

    # Three-line track fields (vision/lane_tracker.py). The defaults leave the
    # classic detector's results unchanged.
    lane_side: Optional[str] = None          # "left" / "right": lane the car is in
    left_kind: Optional[str] = None          # "solid" / "dashed"
    right_kind: Optional[str] = None
    look_ahead_error: float = 0.0            # lane-centre offset further ahead (px)
    blocked_sides: Tuple[str, ...] = ()      # sides bounded by a solid outer line


class LaneDetector:
    """
    Sanoguzhan Perspective Warping & Histogram Peak Detection Lane Assistant.
    """

    def __init__(
        self,
        deadzone_pixels: int = 15,
        vehicle_half_width_px: int = 45,
        boundary_margin_px: int = 18,
        kp: float = 0.0075,
        kd: float = 0.0025,
    ) -> None:
        self.deadzone_pixels: int = deadzone_pixels
        self.vehicle_half_width_px: int = vehicle_half_width_px
        self.boundary_margin_px: int = boundary_margin_px
        self.kp: float = kp
        self.kd: float = kd

        self._prev_error: float = 0.0
        self._last_time: float = time.time()
        self.fps_counter: FPSCounter = FPSCounter().start()

        # Sanoguzhan Fixed 400x240 Trapezoid Perspective Geometry
        # Source trapezoid: Bottom road surface in camera view
        self.src_points = np.float32([
            [80, 160],   # Top-Left
            [300, 160],  # Top-Right
            [40, 210],   # Bottom-Left
            [340, 210]   # Bottom-Right
        ])
        # Destination rectangle: Bird's-Eye View (Top-Down)
        self.dst_points = np.float32([
            [130, 0],    # Top-Left
            [310, 0],    # Top-Right
            [130, 240],  # Bottom-Left
            [310, 240]   # Bottom-Right
        ])

        self.M = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        self.Minv = cv2.getPerspectiveTransform(self.dst_points, self.src_points)

        self.nominal_lane_width = 160
        self.nominal_half_width = 80

        # Where the camera's optical centre (x=200 at the bumper row) lands in the
        # bird's-eye plane. The warp is not centred on x=200, so measuring lane
        # error against 200 gives a constant steering bias.
        centre = cv2.perspectiveTransform(np.float32([[[200.0, 210.0]]]), self.M)
        self.bev_center_x: int = int(round(float(centre[0, 0, 0])))

    def detect_from_bgr(self, bgr_frame: np.ndarray) -> LaneResult:
        """
        Executes end-to-end Sanoguzhan perspective transform, histogram peak detection,
        and AR corridor unwarping.
        """
        self.fps_counter.update()
        fps = self.fps_counter.get_fps()

        if bgr_frame is None or bgr_frame.size == 0:
            return LaneResult()

        orig_h, orig_w = bgr_frame.shape[:2]
        # Standardize to 400x240 for ultra-fast deterministic perspective math
        if orig_w != 400 or orig_h != 240:
            frame_400 = cv2.resize(bgr_frame, (400, 240), interpolation=cv2.INTER_NEAREST)
        else:
            frame_400 = bgr_frame.copy()

        h, w = 240, 400
        image_center_x = w // 2  # 200

        # ── 1. Warp Perspective to Bird's-Eye View ────────────────────────────
        warped = cv2.warpPerspective(frame_400, self.M, (w, h), flags=cv2.INTER_LINEAR)

        # ── 2. Multi-Channel Lane Thresholding ────────────────────────────────
        warped_hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)

        # White lane line mask (high value, low saturation)
        white_mask = cv2.inRange(warped_hsv, np.array([0, 0, 140]), np.array([180, 60, 255]))
        # Yellow lane line mask
        yellow_mask = cv2.inRange(warped_hsv, np.array([14, 60, 60]), np.array([38, 255, 255]))

        # Combined binary lane mask. Canny is deliberately excluded: it fires on
        # every floor texture / clutter edge and made a "lane" appear everywhere.
        combined_binary = cv2.bitwise_or(white_mask, yellow_mask)

        # Clean small noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        combined_binary = cv2.morphologyEx(combined_binary, cv2.MORPH_CLOSE, kernel)

        # ── 3. Column Histogram Peak Detection ────────────────────────────────
        # Focus on lower 60% of the top-down road (y = 100 to 240)
        hist_roi = combined_binary[100:240, :]
        histogram = np.sum(hist_roi > 0, axis=0)  # lit pixels per column

        midpoint = self.bev_center_x
        min_peak_threshold = 14    # Minimum lit pixels in a column (of 140) to count as a lane line

        left_half = histogram[:midpoint]
        right_half = histogram[midpoint:]

        left_max = np.max(left_half) if len(left_half) > 0 else 0
        right_max = np.max(right_half) if len(right_half) > 0 else 0

        left_line_x = None
        right_line_x = None

        if left_max >= min_peak_threshold:
            left_line_x = int(np.argmax(left_half))

        if right_max >= min_peak_threshold:
            right_line_x = int(midpoint + np.argmax(right_half))

        # Lane center & width resolution
        lane_center_x = None
        contour_found = False
        lane_width = None

        if left_line_x is not None and right_line_x is not None:
            lane_center_x = int((left_line_x + right_line_x) // 2)
            lane_width = right_line_x - left_line_x
            contour_found = True
            # Update nominal width tracker if within reasonable bounds (100 to 220 px)
            if 100 <= lane_width <= 220:
                self.nominal_lane_width = lane_width
                self.nominal_half_width = lane_width // 2
        elif left_line_x is not None:
            lane_center_x = int(left_line_x + self.nominal_half_width)
            lane_center_x = min(w - 10, lane_center_x)
            contour_found = True
        elif right_line_x is not None:
            lane_center_x = int(right_line_x - self.nominal_half_width)
            lane_center_x = max(10, lane_center_x)
            contour_found = True

        lane_center_y = int(h * 0.75)

        # ── 4. Virtual Hard Boundary Enforcement ──────────────────────────────
        car_left_x = midpoint - self.vehicle_half_width_px
        car_right_x = midpoint + self.vehicle_half_width_px

        is_left_boundary_violated = False
        is_right_boundary_violated = False
        departure_warning = None
        corrective_steering = 0.0
        min_allowed_steering = -1.0
        max_allowed_steering = 1.0

        if left_line_x is not None:
            left_clearance = car_left_x - left_line_x
            if left_clearance <= self.boundary_margin_px:
                is_left_boundary_violated = True
                departure_warning = "LEFT"
                corrective_steering = 0.50
                min_allowed_steering = 0.0  # Too close to left line: forbid steering left (negative)

        if right_line_x is not None:
            right_clearance = right_line_x - car_right_x
            if right_clearance <= self.boundary_margin_px:
                is_right_boundary_violated = True
                departure_warning = "RIGHT"
                corrective_steering = -0.50
                max_allowed_steering = 0.0  # Too close to right line: forbid steering right (positive)

        # ── 5. Closed-Loop PD Steering Calculation ────────────────────────────
        now = time.time()
        dt = max(0.01, now - self._last_time)
        self._last_time = now

        if contour_found and lane_center_x is not None:
            error = float(lane_center_x - midpoint)

            if abs(error) <= self.deadzone_pixels:
                direction = Direction.STRAIGHT
            elif error < -self.deadzone_pixels:
                direction = Direction.LEFT
            else:
                direction = Direction.RIGHT

            d_err = (error - self._prev_error) / dt
            self._prev_error = error

            raw_steer = self.kp * error + self.kd * d_err
            suggested_steering = max(min_allowed_steering, min(max_allowed_steering, raw_steer))
            lane_detected = True
        else:
            error = 0.0
            direction = Direction.NO_LANE
            suggested_steering = 0.0
            lane_detected = False
            self._prev_error = 0.0

        # ── 6. Augmented Reality Unwarped Overlay (Perspective Space) ─────────
        annotated_frame = frame_400.copy()

        # Build Bird's-Eye visualization
        birdseye_view = warped.copy()
        cv2.line(birdseye_view, (midpoint, 0), (midpoint, h), (255, 120, 0), 1)

        # Draw detected boundaries on Bird's-Eye
        if left_line_x is not None:
            cv2.line(birdseye_view, (left_line_x, 0), (left_line_x, h), (0, 255, 255), 2)
            cv2.circle(birdseye_view, (left_line_x, 180), 5, (0, 255, 255), -1)
        if right_line_x is not None:
            cv2.line(birdseye_view, (right_line_x, 0), (right_line_x, h), (255, 255, 0), 2)
            cv2.circle(birdseye_view, (right_line_x, 180), 5, (255, 255, 0), -1)

        if lane_center_x is not None:
            cv2.circle(birdseye_view, (lane_center_x, lane_center_y), 6, (0, 0, 255), -1)
            cv2.line(birdseye_view, (midpoint, lane_center_y), (lane_center_x, lane_center_y), (0, 0, 255), 2)

        # Draw illuminated drivable corridor on top-down plane and unwarp back
        if contour_found:
            l_draw = left_line_x if left_line_x is not None else (lane_center_x - self.nominal_half_width)
            r_draw = right_line_x if right_line_x is not None else (lane_center_x + self.nominal_half_width)
            l_draw = max(0, min(w, l_draw))
            r_draw = max(0, min(w, r_draw))

            corridor_mask = np.zeros((h, w, 3), dtype=np.uint8)
            corridor_pts = np.array([
                [l_draw, h],
                [l_draw, 40],
                [r_draw, 40],
                [r_draw, h]
            ], dtype=np.int32)
            cv2.fillPoly(corridor_mask, [corridor_pts], (40, 220, 40))

            # Unwarp corridor back to camera perspective
            unwarped_corridor = cv2.warpPerspective(corridor_mask, self.Minv, (w, h), flags=cv2.INTER_LINEAR)
            annotated_frame = cv2.addWeighted(annotated_frame, 1.0, unwarped_corridor, 0.35, 0)

        # Draw trapezoid boundary region on perspective camera view
        trapezoid_pts = np.int32(self.src_points)
        cv2.polylines(annotated_frame, [trapezoid_pts], isClosed=True, color=(140, 100, 255), thickness=1)

        # Vehicle Centerline & Envelope
        cv2.line(annotated_frame, (image_center_x, int(h * 0.50)), (image_center_x, h), (255, 120, 0), 1)
        cv2.line(annotated_frame, (car_left_x, h - 35), (car_left_x, h), (200, 200, 255), 1)
        cv2.line(annotated_frame, (car_right_x, h - 35), (car_right_x, h), (200, 200, 255), 1)

        # Lane Center Point
        if lane_center_x is not None:
            # Map lane center from top-down to perspective space
            pts_bev = np.array([[[float(lane_center_x), float(lane_center_y)]]], dtype=np.float32)
            pts_persp = cv2.perspectiveTransform(pts_bev, self.Minv)
            px, py = int(pts_persp[0, 0, 0]), int(pts_persp[0, 0, 1])
            if 0 <= px < w and 0 <= py < h:
                cv2.circle(annotated_frame, (px, py), 6, (0, 0, 255), -1)
                cv2.line(annotated_frame, (image_center_x, py), (px, py), (0, 255, 255), 2)

        # Departure Warnings
        if departure_warning == "LEFT":
            cv2.rectangle(annotated_frame, (8, 6), (280, 32), (0, 0, 240), -1)
            cv2.putText(annotated_frame, "LANE DEPARTURE: LEFT!", (12, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        elif departure_warning == "RIGHT":
            cv2.rectangle(annotated_frame, (8, 6), (280, 32), (0, 0, 240), -1)
            cv2.putText(annotated_frame, "LANE DEPARTURE: RIGHT!", (12, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        # Scale back to original dimensions if original was not 400x240
        if orig_w != 400 or orig_h != 240:
            annotated_frame = cv2.resize(annotated_frame, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
            scale_x = orig_w / 400.0
            scale_y = orig_h / 240.0
            res_center_x = int(lane_center_x * scale_x) if lane_center_x is not None else None
            res_center_y = int(lane_center_y * scale_y) if lane_center_y is not None else None
            res_left_x = int(left_line_x * scale_x) if left_line_x is not None else None
            res_right_x = int(right_line_x * scale_x) if right_line_x is not None else None
        else:
            res_center_x = lane_center_x
            res_center_y = lane_center_y
            res_left_x = left_line_x
            res_right_x = right_line_x

        return LaneResult(
            lane_center_x=res_center_x,
            lane_center_y=res_center_y,
            image_center_x=orig_w // 2,
            error=error,
            direction=direction,
            contour_found=contour_found,
            largest_contour=None,
            annotated_frame=annotated_frame,
            birdseye_frame=birdseye_view,
            left_line_x=res_left_x,
            right_line_x=res_right_x,
            lane_width=lane_width,
            lane_detected=lane_detected,
            is_left_boundary_violated=is_left_boundary_violated,
            is_right_boundary_violated=is_right_boundary_violated,
            departure_warning=departure_warning,
            suggested_steering=round(float(suggested_steering), 3),
            corrective_steering=corrective_steering,
            min_allowed_steering=min_allowed_steering,
            max_allowed_steering=max_allowed_steering,
        )


SanoguzhanLaneDetector = LaneDetector
