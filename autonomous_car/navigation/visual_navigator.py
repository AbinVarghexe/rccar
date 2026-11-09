"""
visual_navigator.py - Visual Destination Targeting & Lane Guidance Fusion.

Coordinates camera-based visual targeting and lane guidance:
1. Detects destination markers (colored visual beacon, e.g. Neon Green/Orange,
   or user-designated (x, y) target pixel in camera FOV).
2. Calculates bearing angle and distance to destination in the camera view.
3. Fuses lane centering error with destination steering bias to keep vehicle
   centered in lanes while progressing toward the visual target.
"""

import math
import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any


@dataclass
class VisualTarget:
    target_x: int
    target_y: int
    bearing_deg: float      # Negative = Left, Positive = Right
    distance_norm: float    # 0.0 (bottom of screen / close) to 1.0 (horizon / far)
    is_visible: bool
    label: str = "TARGET"


@dataclass
class NavigationVector:
    steering_angle: float   # -1.0 (full left) to +1.0 (full right)
    speed_throttle: float   # 0.0 to 1.0
    lane_error_norm: float  # -1.0 (left of lane) to +1.0 (right of lane)
    dest_bearing_deg: float # Bearing toward visual destination
    state: str              # "LANE_FOLLOWING", "TARGET_APPROACH", "SEARCHING_TARGET"


class VisualNavigator:
    """
    Fuses Lane Following with Visual Destination Targeting.
    """

    def __init__(
        self,
        camera_fov_h_deg: float = 62.2,  # OV5647 horizontal field of view
        dest_color_hsv_lower: Tuple[int, int, int] = (35, 80, 80),    # Default: Green marker
        dest_color_hsv_upper: Tuple[int, int, int] = (85, 255, 255),
    ):
        self.camera_fov_h_deg = camera_fov_h_deg
        self.dest_hsv_lower = np.array(dest_color_hsv_lower, dtype=np.uint8)
        self.dest_hsv_upper = np.array(dest_color_hsv_upper, dtype=np.uint8)
        self.manual_target: Optional[Tuple[int, int]] = None
        self.last_target: Optional[VisualTarget] = None

    def set_manual_destination(self, x: int, y: int) -> None:
        """Sets an explicit pixel coordinate in the camera view as destination."""
        self.manual_target = (int(x), int(y))

    def clear_manual_destination(self) -> None:
        self.manual_target = None

    def find_destination_marker(self, bgr_frame: np.ndarray) -> Optional[VisualTarget]:
        """
        Locates destination marker either via manual pixel target or color beacon.
        """
        if bgr_frame is None or bgr_frame.size == 0:
            return None

        h, w = bgr_frame.shape[:2]
        center_x = w / 2.0

        # Case 1: Manual destination set via UI / API
        if self.manual_target is not None:
            mx, my = self.manual_target
            bearing = ((mx - center_x) / center_x) * (self.camera_fov_h_deg / 2.0)
            dist_norm = max(0.0, min(1.0, (h - my) / float(h)))
            target = VisualTarget(
                target_x=mx,
                target_y=my,
                bearing_deg=bearing,
                distance_norm=dist_norm,
                is_visible=True,
                label="PINNED_DESTINATION"
            )
            self.last_target = target
            return target

        # Case 2: Autonomous color beacon detection (Green destination marker)
        hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.dest_hsv_lower, self.dest_hsv_upper)
        
        # Noise cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Filter by minimum area (at least 150 pixels)
        valid = [c for c in contours if cv2.contourArea(c) > 150]
        if not valid:
            return None

        largest = max(valid, key=cv2.contourArea)
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        tx = int(M["m10"] / M["m00"])
        ty = int(M["m01"] / M["m00"])

        bearing = ((tx - center_x) / center_x) * (self.camera_fov_h_deg / 2.0)
        dist_norm = max(0.0, min(1.0, (h - ty) / float(h)))

        target = VisualTarget(
            target_x=tx,
            target_y=ty,
            bearing_deg=bearing,
            distance_norm=dist_norm,
            is_visible=True,
            label="BEACON_DESTINATION"
        )
        self.last_target = target
        return target

    def compute_steering(
        self,
        bgr_frame: np.ndarray,
        lane_error_px: float = 0.0,
        lane_found: bool = True,
        lane_weight: float = 0.60,
        dest_weight: float = 0.40,
    ) -> NavigationVector:
        """
        Blends lane centering error with visual destination bearing.
        Returns combined steering angle [-1.0, +1.0] and suggested throttle.
        """
        h, w = bgr_frame.shape[:2]
        center_x = w / 2.0

        # Normalize lane error to [-1.0, 1.0]
        lane_error_norm = max(-1.0, min(1.0, lane_error_px / center_x))

        # Check visual destination
        target = self.find_destination_marker(bgr_frame)

        if target is not None and target.is_visible:
            # Normalize destination bearing to [-1.0, 1.0]
            max_bearing = self.camera_fov_h_deg / 2.0
            dest_steer_norm = max(-1.0, min(1.0, target.bearing_deg / max_bearing))

            if lane_found:
                # Fused navigation: follow lane while biasing toward destination
                combined_steer = (lane_error_norm * lane_weight) + (dest_steer_norm * dest_weight)
                state = "LANE_AND_DESTINATION"
            else:
                # Open area: head directly toward destination
                combined_steer = dest_steer_norm
                state = "TARGET_DIRECT_APPROACH"

            # Slow down throttle as we approach close to destination
            if target.distance_norm < 0.25:  # Target is near bottom of frame
                throttle = 0.35  # Slow approach / prepare to stop
            else:
                throttle = 0.55

            dest_bearing = target.bearing_deg
        else:
            # No visual destination in current FOV: pure lane keeping
            combined_steer = lane_error_norm if lane_found else 0.0
            throttle = 0.50 if lane_found else 0.0
            dest_bearing = 0.0
            state = "LANE_KEEPING" if lane_found else "SEARCHING_LANE_OR_DEST"

        combined_steer = max(-1.0, min(1.0, combined_steer))

        return NavigationVector(
            steering_angle=round(combined_steer, 3),
            speed_throttle=round(throttle, 2),
            lane_error_norm=round(lane_error_norm, 3),
            dest_bearing_deg=round(dest_bearing, 1),
            state=state
        )

    def draw_hud(
        self,
        frame: np.ndarray,
        nav_vector: NavigationVector,
        target: Optional[VisualTarget] = None
    ) -> np.ndarray:
        """Annotates frame with destination reticle, bearing arrow, and navigation HUD."""
        h, w = frame.shape[:2]
        cx = w // 2

        # 1. Draw Target Reticle if visible
        if target is not None and target.is_visible:
            tx, ty = target.target_x, target.target_y
            cv2.circle(frame, (tx, ty), 16, (0, 255, 0), 2)
            cv2.circle(frame, (tx, ty), 4, (0, 255, 0), -1)
            cv2.drawMarker(frame, (tx, ty), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
            
            # Destination label & bearing
            label = f"{target.label} [{target.bearing_deg:+.1f} deg]"
            cv2.putText(frame, label, (tx - 50, max(20, ty - 22)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

            # Draw trajectory guideline from car bottom-center to target
            cv2.line(frame, (cx, h - 10), (tx, ty), (0, 255, 200), 2, cv2.LINE_AA)

        # 2. Draw Heading / Steering Vector at bottom center
        steer_px = int(cx + (nav_vector.steering_angle * (cx * 0.7)))
        cv2.arrowedLine(frame, (cx, h - 15), (steer_px, h - 55), (0, 255, 255), 3, tipLength=0.35)

        # 3. Bottom Navigation Banner
        cv2.rectangle(frame, (0, h - 30), (w, h), (20, 20, 25), -1)
        nav_text = f"STATE: {nav_vector.state} | STEER: {nav_vector.steering_angle:+.2f} | DEST: {nav_vector.dest_bearing_deg:+.1f} deg"
        cv2.putText(frame, nav_text, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)

        return frame
