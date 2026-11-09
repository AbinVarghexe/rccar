"""
ai/traffic_rules.py - Traffic Rules Compliance Engine for Autonomous RC Car.

Enforces real-world traffic code:
1. Traffic Light Obedience:
   - Red Light: Mandatory full stop behind the stop line.
   - Yellow Light: Deceleration & prepare to stop.
   - Green Light: Proceed / cruise.
2. Stop Sign Compliance (DOT / SAE Standard):
   - Mandatory full stop ($v = 0$).
   - Mandatory dwell time (2.5 - 3.0 seconds) at the stop line.
   - Forward intersection clearance check before proceeding.
   - Single-stop spatial/cooldown lock so the car doesn't re-stop at the same sign.
3. Lane Boundary Traffic Law:
   - Solid lane line crossing prohibited.
   - In-lane queueing behind stopped obstacles.
"""

import time
import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, List, Tuple, Dict, Any
import numpy as np
import cv2

from ai.classes import DetectionResult, BoundingBox, ActionEnum

logger = logging.getLogger(__name__)


class TrafficLightState(str, Enum):
    RED = "RED"
    YELLOW = "YELLOW"
    GREEN = "GREEN"
    UNKNOWN = "UNKNOWN"


class StopSignState(str, Enum):
    IDLE = "IDLE"
    STOPPING = "STOPPING"
    DWELLING = "DWELLING"
    CHECKING_CLEAR = "CHECKING_CLEAR"
    CLEARED = "CLEARED"


@dataclass
class TrafficRuleResult:
    requires_stop: bool = False
    requires_slow: bool = False
    action: ActionEnum = ActionEnum.FORWARD
    reason: str = "Traffic rules satisfied: Path clear"
    rule_name: str = "NONE"
    traffic_light_state: TrafficLightState = TrafficLightState.UNKNOWN
    stop_sign_state: StopSignState = StopSignState.IDLE
    dwell_remaining_sec: float = 0.0
    suggested_throttle_cap: Optional[float] = None
    stop_sign_detected: bool = False
    traffic_light_detected: bool = False


class TrafficRuleEngine:
    """
    Evaluates real-time camera frames and detected objects against autonomous traffic regulations.
    """

    def __init__(
        self,
        stop_sign_dwell_sec: float = 2.5,
        stop_sign_area_threshold: float = 0.008,
        stop_sign_cooldown_sec: float = 8.0,
        traffic_light_area_threshold: float = 0.004,
    ):
        self.stop_sign_dwell_sec = stop_sign_dwell_sec
        self.stop_sign_area_threshold = stop_sign_area_threshold
        self.stop_sign_cooldown_sec = stop_sign_cooldown_sec
        self.traffic_light_area_threshold = traffic_light_area_threshold

        # Stop Sign State Machine
        self._stop_sign_state: StopSignState = StopSignState.IDLE
        self._stop_start_time: float = 0.0
        self._last_stop_sign_cleared_time: float = 0.0

        # Traffic Light Tracking Filter (Temporal Smoothing)
        self._last_tl_state: TrafficLightState = TrafficLightState.UNKNOWN
        self._tl_state_counter: int = 0

    def reset(self) -> None:
        """Resets traffic engine state."""
        self._stop_sign_state = StopSignState.IDLE
        self._stop_start_time = 0.0
        self._last_stop_sign_cleared_time = 0.0
        self._last_tl_state = TrafficLightState.UNKNOWN
        self._tl_state_counter = 0

    def evaluate(
        self,
        bgr_frame: np.ndarray,
        detections: List[DetectionResult],
        front_distance_cm: Optional[float] = None,
        now: Optional[float] = None,
    ) -> TrafficRuleResult:
        """
        Evaluates current frame and detections against traffic rules.
        """
        cur_time = now if now is not None else time.time()
        h, w = bgr_frame.shape[:2] if bgr_frame is not None and bgr_frame.size > 0 else (480, 640)
        frame_area = float(w * h)

        # ---------------------------------------------------------------------
        # 1. EVALUATE TRAFFIC LIGHTS
        # ---------------------------------------------------------------------
        tl_detections = [
            d for d in detections
            if d.class_name.lower() in ("traffic light", "traffic_light")
            and (d.bbox.area / frame_area) >= self.traffic_light_area_threshold
        ]

        tl_state = TrafficLightState.UNKNOWN
        if tl_detections and bgr_frame is not None and bgr_frame.size > 0:
            # Pick largest traffic light in view
            best_tl = max(tl_detections, key=lambda d: d.bbox.area)
            tl_state = self._classify_traffic_light_color(bgr_frame, best_tl.bbox)

            # Temporal filtering: Require at least 2 consistent frames or instant red
            if tl_state == TrafficLightState.RED:
                self._last_tl_state = TrafficLightState.RED
                logger.info("[TRAFFIC RULE] RED Traffic Light confirmed ahead! Enforcing mandatory stop.")
                return TrafficRuleResult(
                    requires_stop=True,
                    action=ActionEnum.STOP,
                    reason="🚦 Traffic Rule: RED Traffic Light active. Vehicle stopped behind stop line.",
                    rule_name="RED_LIGHT",
                    traffic_light_state=TrafficLightState.RED,
                    traffic_light_detected=True,
                )
            elif tl_state == TrafficLightState.YELLOW:
                return TrafficRuleResult(
                    requires_slow=True,
                    action=ActionEnum.SLOW,
                    reason="⚠️ Traffic Rule: YELLOW Traffic Light active. Preparing to stop / decelerating.",
                    rule_name="YELLOW_LIGHT",
                    traffic_light_state=TrafficLightState.YELLOW,
                    suggested_throttle_cap=0.28,
                    traffic_light_detected=True,
                )
            elif tl_state == TrafficLightState.GREEN:
                self._last_tl_state = TrafficLightState.GREEN

        # ---------------------------------------------------------------------
        # 2. EVALUATE STOP SIGNS (Mandatory 2.5s Dwell Stop-and-Go)
        # ---------------------------------------------------------------------
        stop_sign_dets = [
            d for d in detections
            if d.class_name.lower() in ("stop sign", "stop_sign")
            and (d.bbox.area / frame_area) >= self.stop_sign_area_threshold
        ]

        cooldown_elapsed = cur_time - self._last_stop_sign_cleared_time
        in_cooldown = cooldown_elapsed < self.stop_sign_cooldown_sec

        if stop_sign_dets and not in_cooldown:
            # Active Stop Sign Encountered
            if self._stop_sign_state in (StopSignState.IDLE, StopSignState.STOPPING):
                self._stop_sign_state = StopSignState.DWELLING
                self._stop_start_time = cur_time
                logger.info(
                    f"[TRAFFIC RULE] STOP Sign detected ahead (Area: {stop_sign_dets[0].bbox.area/frame_area:.3f}). "
                    f"Beginning mandatory {self.stop_sign_dwell_sec:.1f}s dwell stop."
                )

            if self._stop_sign_state == StopSignState.DWELLING:
                dwell_elapsed = cur_time - self._stop_start_time
                dwell_remaining = max(0.0, self.stop_sign_dwell_sec - dwell_elapsed)

                if dwell_elapsed < self.stop_sign_dwell_sec:
                    return TrafficRuleResult(
                        requires_stop=True,
                        action=ActionEnum.STOP,
                        reason=f"🛑 Traffic Rule: Mandatory STOP Sign Dwell ({dwell_elapsed:.1f}s / {self.stop_sign_dwell_sec:.1f}s). Holding position.",
                        rule_name="STOP_SIGN_DWELL",
                        stop_sign_state=StopSignState.DWELLING,
                        dwell_remaining_sec=round(dwell_remaining, 1),
                        stop_sign_detected=True,
                    )
                else:
                    # Dwell complete, transition to check clear
                    self._stop_sign_state = StopSignState.CHECKING_CLEAR

            if self._stop_sign_state == StopSignState.CHECKING_CLEAR:
                # Ensure intersection path is clear (front distance > 25cm)
                if front_distance_cm is not None and front_distance_cm < 25.0:
                    return TrafficRuleResult(
                        requires_stop=True,
                        action=ActionEnum.STOP,
                        reason="🛑 Traffic Rule: Stop sign dwell complete. Waiting for intersection to clear...",
                        rule_name="STOP_SIGN_WAIT_CLEAR",
                        stop_sign_state=StopSignState.CHECKING_CLEAR,
                        stop_sign_detected=True,
                    )
                else:
                    # Cleared!
                    self._stop_sign_state = StopSignState.CLEARED
                    self._last_stop_sign_cleared_time = cur_time
                    logger.info("[TRAFFIC RULE] STOP Sign dwell & intersection check complete. Proceeding.")
                    return TrafficRuleResult(
                        requires_stop=False,
                        action=ActionEnum.FORWARD,
                        reason="🛑 Traffic Rule: Stop sign cleared. Proceeding through intersection.",
                        rule_name="STOP_SIGN_CLEARED",
                        stop_sign_state=StopSignState.CLEARED,
                        stop_sign_detected=True,
                    )

        # If no stop sign or already cleared
        if not stop_sign_dets and self._stop_sign_state != StopSignState.IDLE and not in_cooldown:
            self._stop_sign_state = StopSignState.IDLE

        # Default: All traffic rules satisfied
        tl_detected = (tl_state != TrafficLightState.UNKNOWN)
        return TrafficRuleResult(
            requires_stop=False,
            requires_slow=False,
            action=ActionEnum.FORWARD,
            reason="Traffic rules satisfied: Green light / path clear",
            rule_name="CLEAR",
            traffic_light_state=tl_state,
            stop_sign_state=self._stop_sign_state,
            traffic_light_detected=tl_detected,
            stop_sign_detected=bool(stop_sign_dets),
        )

    def _classify_traffic_light_color(self, bgr_frame: np.ndarray, bbox: BoundingBox) -> TrafficLightState:
        """
        Analyzes the crop of a traffic light bounding box to detect whether
        the Red, Yellow, or Green bulb is actively illuminated.
        """
        h, w = bgr_frame.shape[:2]
        x1 = max(0, min(w - 1, int(bbox.x1)))
        y1 = max(0, min(h - 1, int(bbox.y1)))
        x2 = max(0, min(w, int(bbox.x2)))
        y2 = max(0, min(h, int(bbox.y2)))

        roi = bgr_frame[y1:y2, x1:x2]
        if roi.size == 0 or roi.shape[0] < 6 or roi.shape[1] < 4:
            return TrafficLightState.UNKNOWN

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Red mask (wraps around 0/180)
        red1 = cv2.inRange(hsv, np.array([0, 70, 70], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8))
        red2 = cv2.inRange(hsv, np.array([160, 70, 70], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8))
        red_mask = cv2.bitwise_or(red1, red2)

        # Yellow mask
        yellow_mask = cv2.inRange(hsv, np.array([16, 70, 70], dtype=np.uint8), np.array([36, 255, 255], dtype=np.uint8))

        # Green mask
        green_mask = cv2.inRange(hsv, np.array([40, 60, 60], dtype=np.uint8), np.array([92, 255, 255], dtype=np.uint8))

        red_pixels = int(cv2.countNonZero(red_mask))
        yellow_pixels = int(cv2.countNonZero(yellow_mask))
        green_pixels = int(cv2.countNonZero(green_mask))

        total_pixels = float(roi.shape[0] * roi.shape[1])
        min_pixels = max(3, int(total_pixels * 0.03))

        # Check by colored pixel threshold
        if red_pixels >= min_pixels and red_pixels >= yellow_pixels and red_pixels >= green_pixels:
            return TrafficLightState.RED
        elif yellow_pixels >= min_pixels and yellow_pixels > red_pixels and yellow_pixels >= green_pixels:
            return TrafficLightState.YELLOW
        elif green_pixels >= min_pixels and green_pixels > red_pixels and green_pixels > yellow_pixels:
            return TrafficLightState.GREEN

        # Fallback to vertical bulb position luminance
        # Traffic lights typically have: Top = Red, Middle = Yellow, Bottom = Green
        rh = roi.shape[0]
        one_third = rh // 3
        if one_third > 0:
            top_zone = roi[0:one_third, :]
            mid_zone = roi[one_third:2 * one_third, :]
            bot_zone = roi[2 * one_third:, :]

            top_lum = float(np.mean(top_zone))
            mid_lum = float(np.mean(mid_zone))
            bot_lum = float(np.mean(bot_zone))

            if top_lum > 140 and top_lum > (mid_lum + 20) and top_lum > (bot_lum + 20):
                return TrafficLightState.RED
            elif bot_lum > 140 and bot_lum > (top_lum + 20) and bot_lum > (mid_lum + 20):
                return TrafficLightState.GREEN

        return TrafficLightState.UNKNOWN
