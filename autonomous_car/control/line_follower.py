"""
control/line_follower.py — 4-in-1 Hybrid Autonomous Perception, Navigation & Active Servo Radar.

Unifies four core technologies into ONE synchronized autonomous vehicle driver:
1. ArUco / AprilTag Visual Waypoint Navigation (3D metric homing, arrival detection)
2. Computer Vision Lane Tracking (LKA center tracking, road edge containment)
3. Embedded ONNX Neural Pilot (SidewalkPilot-v3.1b.onnx driveability inference)
4. Active Pan Servo Obstacle Scanner (Multi-sector visual + sonar radar, dynamic detours)
"""

import os
import sys
import time
import math
import threading
from enum import Enum
from typing import Optional, Tuple, Dict, Any, List
import cv2
import numpy as np

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
for p in [curr_dir, parent_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from vision.lane_detector import LaneDetector, Direction, LaneResult
    from navigation.visual_navigator import VisualNavigator, VisualTarget, NavigationVector
    from navigation.aruco_navigator import ArucoNavigator, ArucoPose
    from behavioral_cloning import PilotModel
    from control.serial_comm import VehicleCommand
    from control.obstacle_avoider import AvoidConfig, AvoidState, ObstacleAvoider
    from vision.free_space import FreeSpace, FreeSpaceAnalyzer, choose_heading
    from vision.lane_tracker import ThreeLineLaneDetector, constrain_side_clearance
    from control.side_look import SideLook
except ImportError:
    from autonomous_car.vision.lane_detector import LaneDetector, Direction, LaneResult
    from autonomous_car.navigation.visual_navigator import VisualNavigator, VisualTarget, NavigationVector
    from autonomous_car.navigation.aruco_navigator import ArucoNavigator, ArucoPose
    from autonomous_car.behavioral_cloning import PilotModel
    from autonomous_car.control.serial_comm import VehicleCommand
    from autonomous_car.control.obstacle_avoider import AvoidConfig, AvoidState, ObstacleAvoider
    from autonomous_car.vision.free_space import FreeSpace, FreeSpaceAnalyzer, choose_heading
    from autonomous_car.vision.lane_tracker import ThreeLineLaneDetector, constrain_side_clearance
    from autonomous_car.control.side_look import SideLook


class NavigationMode(str, Enum):
    HYBRID_FUSION = "HYBRID_FUSION"              # 4-in-1 Unified Fusion Driver (Default)
    ARUCO_WAYPOINT = "ARUCO_WAYPOINT"            # Dedicated ArUco landmark tracking
    LANE_FOLLOWING = "LANE_FOLLOWING"            # Computer vision road line tracking
    SANOGUZHAN_CLASSIC = "SANOGUZHAN_CLASSIC"    # Perspective + histogram lane controller
    AI_ONNX_PILOT = "AI_ONNX_PILOT"              # Neural network end-to-end driving


class LineFollowerState(str, Enum):
    IDLE = "IDLE"
    HYBRID_NAVIGATING = "HYBRID_NAVIGATING"
    TRACKING = "TRACKING"
    HOMING_TO_ARUCO = "HOMING_TO_ARUCO"
    OBSTACLE_AVOIDANCE_ONNX = "OBSTACLE_AVOIDANCE_ONNX"
    OBSTACLE_AVOIDING_DETOUR = "OBSTACLE_AVOIDING_DETOUR"
    OBSTACLE_STOP = "OBSTACLE_STOP"
    STOP_SIGN_PAUSE = "STOP_SIGN_PAUSE"
    ARRIVED_AT_DESTINATION = "ARRIVED_AT_DESTINATION"
    LOST_LANE = "LOST_LANE"


class SurveyMode(str, Enum):
    OFF = "OFF"
    SEARCH_SWEEP = "SEARCH_SWEEP"
    ACTIVE_RADAR = "ACTIVE_RADAR"


VISION_BLOCK_CLEARANCE = 0.20   # centre-sector floor fraction below this = blocked
VISION_BLOCK_FRAMES = 4         # consecutive frames needed (rejects glare/shadow flicker)
STEER_CLEAR_CM = 90.0           # steer around anything the camera ranges closer than this


class ActiveCameraServoScanner:
    """
    Active Pan Servo Controller for Camera.
    - Locks forward (90°) during regular driving.
    - Performs gentle sweep (65° -> 90° -> 115°) when searching for distant ArUco tags.
    - Performs targeted multi-sector inspection (135° Left, 45° Right) during obstacle detours.
    """

    # Slew-limited sweep: the servo glides in small steps instead of jumping 25 degrees,
    # which removes camera shake. The car is held still while the camera is off-centre
    # (see is_sweeping) because a panned view invalidates the lane geometry.
    SWEEP_SPEED_DPS = 45.0       # glide speed
    SWEEP_AMPLITUDE = 22         # degrees either side of centre
    SWEEP_INTERVAL_S = 6.0       # drive this long between sweeps
    SETTLE_S = 0.25              # let the image settle at each end before moving on

    def __init__(self, dwell_time_s: float = 0.40) -> None:
        self.current_angle: int = 90
        self.dwell_time_s: float = dwell_time_s
        self.mode: SurveyMode = SurveyMode.OFF
        self._pos: float = 90.0
        self._targets: List[int] = []
        self._hold_until: float = 0.0
        self._last_t: float = time.time()
        self._next_sweep: float = time.time() + self.SWEEP_INTERVAL_S
        self._lock = threading.Lock()

    def set_mode(self, mode: SurveyMode) -> None:
        with self._lock:
            self.mode = mode
            if mode == SurveyMode.OFF:
                self.reset()

    def set_dwell_time(self, dwell_s: float) -> None:
        with self._lock:
            self.dwell_time_s = max(0.15, min(1.5, dwell_s))

    def reset(self) -> None:
        self._pos = 90.0
        self.current_angle = 90
        self._targets = []
        self._next_sweep = time.time() + self.SWEEP_INTERVAL_S

    @property
    def is_sweeping(self) -> bool:
        return bool(self._targets) or abs(self._pos - 90.0) > 3.0

    def step_search(self, now: float) -> int:
        with self._lock:
            dt = max(0.0, min(0.2, now - self._last_t))
            self._last_t = now
            if not self._targets and now >= self._next_sweep:
                a = self.SWEEP_AMPLITUDE
                self._targets = [90 - a, 90 + a, 90]
            if self._targets and now >= self._hold_until:
                goal = self._targets[0]
                step = self.SWEEP_SPEED_DPS * dt
                delta = goal - self._pos
                if abs(delta) <= step:
                    self._pos = float(goal)
                    self._targets.pop(0)
                    self._hold_until = now + self.SETTLE_S
                    if not self._targets:
                        self._next_sweep = now + self.SWEEP_INTERVAL_S
                else:
                    self._pos += step if delta > 0 else -step
            self.current_angle = int(round(self._pos))
            return self.current_angle


class StopSignDetector:
    """Detects Red Octagonal Stop Signs using HSV color segmentation."""

    def __init__(self, min_area: float = 1200.0) -> None:
        self.min_area = min_area

    def detect(self, bgr_frame: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int, int, int]]]:
        if bgr_frame is None or bgr_frame.size == 0:
            return False, None
        # Fast half-scale detection for real-time latency (< 2ms)
        small = cv2.resize(bgr_frame, (320, 240), interpolation=cv2.INTER_NEAREST)
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 120, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        scale = 2
        min_area_scaled = self.min_area / 4.0
        for c in contours:
            area = cv2.contourArea(c)
            if area >= min_area_scaled:
                x, y, w, h = cv2.boundingRect(c)
                aspect = float(w) / max(1, h)
                if 0.70 <= aspect <= 1.30:
                    return True, (x * scale, y * scale, w * scale, h * scale)
        return False, None


class LineFollower:
    """
    Unified 4-in-1 Hybrid Autonomous Vehicle Navigation Engine.
    Fuses ArUco Landmark Waypoints + CV Lane Detection + ONNX Neural Pilot + Active Servo Obstacle Radar.
    """

    def __init__(
        self,
        deadzone_px: int = 22,
        safe_distance_cm: float = 42.0,       # Stop & avoid buffer (generous to prevent any collision)
        caution_distance_cm: float = 65.0,    # Deceleration & obstacle awareness buffer
        lost_lane_timeout_s: float = 1.0,
        enable_auto_avoidance: bool = True,
        model_path: str = "models/SidewalkPilot-v3.1b.onnx",
        marker_size_cm: float = 10.0,
        lane_layout: str = "classic",
        servo_search: bool = True,
    ) -> None:
        # "three_line": solid outer tapes + dashed centre tape (two lanes); the
        # classic detector assumes one lane between two lines.
        self.lane_layout = lane_layout
        # False = the camera never sweeps by itself to hunt for a missing ArUco tag;
        # it only turns to fixed positions on command (see control/side_look.py).
        self.servo_search = servo_search
        self.deadzone_px = deadzone_px
        self.safe_distance_cm = safe_distance_cm
        self.caution_distance_cm = caution_distance_cm
        self.lost_lane_timeout_s = lost_lane_timeout_s
        self.enable_auto_avoidance = enable_auto_avoidance

        # Navigation & Perception Subsystems
        self.aruco_navigator = ArucoNavigator(marker_size_cm=marker_size_cm, arrival_dist_cm=28.0)
        self.lane_detector = (
            ThreeLineLaneDetector(deadzone_pixels=deadzone_px)
            if lane_layout == "three_line"
            else LaneDetector(deadzone_pixels=deadzone_px)
        )
        self.visual_navigator = VisualNavigator()
        self.servo_scanner = ActiveCameraServoScanner(dwell_time_s=0.40)
        self.side_look = SideLook()   # owns the servo: centre, manual hold, fixed left/right looks
        self.stop_sign_detector = StopSignDetector(min_area=1400.0)

        # AI ONNX Model
        self.pilot_model = PilotModel(model_path)
        self.ai_loaded = False
        self._init_ai()

        # Operational Mode & State
        self.nav_mode: NavigationMode = NavigationMode.HYBRID_FUSION
        self.is_active: bool = False
        self.state: LineFollowerState = LineFollowerState.IDLE

        self._last_lane_seen_time: float = time.time()
        self._last_command: str = "STOP"
        self._lock = threading.Lock()

        # Sequential Waypoint Route List ("Link List")
        self.route_list: List[int] = [0, 1, 2]
        self.route_index: int = 0
        self.route_mode_enabled: bool = False
        self._route_arrival_time: float = 0.0
        self._route_arrival_dwell_s: float = 1.2
        self._route_dwelling: bool = False

        # Running Adaptive Floor Ground-Plane Model (Lab Color Space)
        self._running_floor_mean: Optional[np.ndarray] = None
        self._running_floor_std: Optional[np.ndarray] = None

        # Reactive sonar avoidance (stop/pivot/bypass state machine)
        self.avoider = ObstacleAvoider(AvoidConfig(
            stop_cm=safe_distance_cm,
            caution_cm=caution_distance_cm,
            clear_cm=max(safe_distance_cm + 20.0, 55.0),
        ))
        self._detour_phase: int = 0
        self._detour_servo_angle: int = 90
        self._avoid_gate_forward: bool = True
        self._frame_idx: int = 0
        self._vision_block_frames: int = 0
        self.free_space = FreeSpaceAnalyzer()
        self.fs: Optional[FreeSpace] = None
        self._is_floor: Optional[np.ndarray] = None

        # Stop Sign Pause State
        self._stop_pause_start: float = 0.0
        self._stop_pause_duration: float = 3.0
        self._last_stop_sign_time: float = 0.0

        # Spatial Clearance
        self.sector_clearances = {"left": 1.0, "center": 1.0, "right": 1.0}

        # Telemetry
        self.latest_error: float = 0.0
        self.latest_direction: str = "STRAIGHT"
        self.latest_distance: Optional[float] = None
        self.latest_cmd: str = "STOP"
        self.latest_birdseye: Optional[np.ndarray] = None
        self.stop_sign_visible: bool = False
        self.ai_inference_ms: float = 0.0
        self.ai_steer: float = 0.0
        self.fused_steer: float = 0.0
        self._steer_smooth: float = 0.0
        self.dest_distance_est_m: Optional[float] = None

    def _init_ai(self) -> None:
        try:
            self.ai_loaded = self.pilot_model.load()
            if self.ai_loaded:
                print(f"[AI] SidewalkPilot ONNX model loaded (backend: {self.pilot_model.backend}).")
            else:
                print(f"[AI] Pilot model unavailable: {self.pilot_model.error}")
        except Exception as e:
            print(f"[AI] Pilot init error: {e}")

    def start(self) -> None:
        with self._lock:
            self.is_active = True
            if self.nav_mode == NavigationMode.HYBRID_FUSION:
                self.state = LineFollowerState.HYBRID_NAVIGATING
            elif self.nav_mode == NavigationMode.ARUCO_WAYPOINT:
                self.state = LineFollowerState.HOMING_TO_ARUCO
            else:
                self.state = LineFollowerState.TRACKING

    def stop(self) -> None:
        with self._lock:
            self.is_active = False
            self.state = LineFollowerState.IDLE
            self.latest_cmd = "STOP"
            self._detour_phase = 0
            self._detour_servo_angle = 90
            self.avoider.reset()
            self._steer_smooth = 0.0
            self._vision_block_frames = 0
            self._last_command = "STOP"
            self._obstacle_frames = 0

    def set_navigation_mode(self, mode: NavigationMode) -> None:
        with self._lock:
            self.nav_mode = mode

    def set_aruco_target(self, tag_id: Optional[int]) -> None:
        with self._lock:
            self.aruco_navigator.set_target_tag(tag_id)
            if self.is_active:
                if self.nav_mode == NavigationMode.HYBRID_FUSION:
                    self.state = LineFollowerState.HYBRID_NAVIGATING
                else:
                    self.state = LineFollowerState.HOMING_TO_ARUCO

    def set_destination(self, x: int, y: int) -> None:
        with self._lock:
            self.visual_navigator.set_manual_destination(x, y)
            self.dest_distance_est_m = None
            if self.is_active:
                self.state = LineFollowerState.HYBRID_NAVIGATING

    def clear_destination(self) -> None:
        with self._lock:
            self.visual_navigator.clear_manual_destination()
            self.dest_distance_est_m = None
            if self.is_active:
                self.state = LineFollowerState.HYBRID_NAVIGATING

    def set_route_list(self, route: List[int]) -> None:
        """Sets the sequence of ArUco tags for linked route navigation."""
        with self._lock:
            if route:
                self.route_list = [int(x) for x in route]
                self.route_index = 0
                if self.route_mode_enabled:
                    self.aruco_navigator.set_target_tag(self.route_list[0])

    def toggle_route_mode(self, enabled: Optional[bool] = None) -> bool:
        """Enables or disables sequential linked waypoint route tracking."""
        with self._lock:
            if enabled is None:
                self.route_mode_enabled = not self.route_mode_enabled
            else:
                self.route_mode_enabled = bool(enabled)
            if self.route_mode_enabled and self.route_list:
                self.aruco_navigator.set_target_tag(self.route_list[self.route_index])
            return self.route_mode_enabled

    def advance_route_link(self) -> int:
        """Manually or automatically advances to the next waypoint in the route list."""
        with self._lock:
            if self.route_list:
                self.route_index = (self.route_index + 1) % len(self.route_list)
                next_tag = self.route_list[self.route_index]
                self.aruco_navigator.set_target_tag(next_tag)
                return next_tag
            return 0

    def set_survey_mode(self, mode: SurveyMode) -> None:
        self.servo_scanner.set_mode(mode)

    def set_survey_dwell(self, dwell_s: float) -> None:
        self.servo_scanner.set_dwell_time(dwell_s)

    def _evaluate_floor_freespace(self, bgr_frame: np.ndarray) -> Tuple[float, float]:
        """
        Calculates driveability / clearance score [0.0 to 1.0] and average wall/barrier
        contact horizon ratio for the camera view.
        Uses adaptive ground plane Lab color modeling to detect flat walls, cabinets,
        and drawers that lack Canny edges (eliminating the inverted edge flaw).
        Returns:
            clearance_score (0.0 to 1.0)
            horizon_y_ratio (0.0=top, 1.0=bumper contact)
        """
        if bgr_frame is None or bgr_frame.size == 0:
            return 1.0, 0.0

        h, w = bgr_frame.shape[:2]

        # 1. Sample clean ground patch in front of bumper (bottom 15% center)
        sample_y1 = int(h * 0.70)
        sample_y2 = int(h * 0.85)
        sample_x1 = int(w * 0.30)
        sample_x2 = int(w * 0.70)
        floor_patch = bgr_frame[sample_y1:sample_y2, sample_x1:sample_x2]

        lab_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2LAB)
        if floor_patch.size > 0:
            patch_lab = lab_frame[sample_y1:sample_y2, sample_x1:sample_x2]
            mean_lab = np.mean(patch_lab, axis=(0, 1))
            std_lab = np.maximum(np.std(patch_lab, axis=(0, 1)), [10.0, 5.0, 5.0])
            if self._running_floor_mean is None:
                self._running_floor_mean = mean_lab
                self._running_floor_std = std_lab
            else:
                self._running_floor_mean = 0.85 * self._running_floor_mean + 0.15 * mean_lab
                self._running_floor_std = 0.85 * self._running_floor_std + 0.15 * std_lab

        if self._running_floor_mean is None:
            return 0.5, 0.5

        # 2. Evaluate driving ROI (bottom 55% of frame)
        roi_top = int(h * 0.45)
        roi_lab = lab_frame[roi_top:, :]
        diff = np.abs(roi_lab.astype(np.float32) - self._running_floor_mean)
        norm_diff = diff / self._running_floor_std
        dist_map = np.sqrt(np.sum(norm_diff ** 2, axis=2))

        # Drivable floor pixels have color close to the ground model (dist_map <= 2.8)
        is_floor = (dist_map <= 2.8).astype(np.uint8)
        floor_ratio = float(np.count_nonzero(is_floor)) / float(is_floor.size)

        # 3. Column-wise contact horizon (where floor terminates)
        step = max(2, w // 40)
        horizons = []
        roi_h = roi_lab.shape[0]
        for col in range(0, w, step):
            col_data = is_floor[:, col]
            # Scan upwards from bumper (bottom of ROI)
            rev = col_data[::-1]
            non_floor = np.where(rev == 0)[0]
            if len(non_floor) > 4:
                # Wall/obstacle contact height in ROI
                contact_y = roi_h - non_floor[3]
            else:
                contact_y = 0
            horizons.append(contact_y)

        avg_horizon_roi = float(np.mean(horizons)) if horizons else 0.0
        # Low average horizon in ROI means clear open path; high means obstacle is close
        proximity_factor = float(np.clip(1.0 - (avg_horizon_roi / float(roi_h * 0.65)), 0.05, 1.0))

        # Combined clearance: floor area ratio * distance factor
        clearance = float(np.clip(0.60 * floor_ratio + 0.40 * proximity_factor, 0.05, 0.98))
        return round(clearance, 2), round(float(roi_top + avg_horizon_roi) / float(h), 2)

    def _evaluate_visual_clearance(self, bgr_frame: np.ndarray) -> float:
        """Evaluates clearance using ground-plane floor free space."""
        clr, _ = self._evaluate_floor_freespace(bgr_frame)
        return clr

    def _update_sector_clearance(self, bgr_frame: np.ndarray) -> None:
        """
        Ultra-fast unified 3-sector floor freespace clearance calculation (< 9ms).
        Evaluates LAB distance map once on a downsampled 160x60 grid and partitions into 3 sectors.
        """
        if bgr_frame is None or bgr_frame.size == 0:
            return

        h, w = bgr_frame.shape[:2]
        roi_top = int(h * 0.45)
        roi = cv2.resize(bgr_frame[roi_top:, :], (160, 60), interpolation=cv2.INTER_NEAREST)
        lab_frame = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

        # Ground patch near bumper (bottom 25% of ROI)
        patch = lab_frame[45:58, 48:112]
        # Learn the floor only from the straight-ahead view: a side look can be full of
        # wall or obstacle and would teach the model that they are floor.
        if patch.size > 0 and self._detour_phase == 0 and not self.side_look.active:
            mean_lab = np.mean(patch, axis=(0, 1))
            std_lab = np.maximum(np.std(patch, axis=(0, 1)), [10.0, 5.0, 5.0])
            if self._running_floor_mean is None:
                self._running_floor_mean = mean_lab
                self._running_floor_std = std_lab
            else:
                self._running_floor_mean = 0.85 * self._running_floor_mean + 0.15 * mean_lab
                self._running_floor_std = 0.85 * self._running_floor_std + 0.15 * std_lab

        if self._running_floor_mean is None:
            return

        diff = np.abs(lab_frame.astype(np.float32) - self._running_floor_mean)
        norm_diff = diff / self._running_floor_std
        dist_map = np.sum(norm_diff ** 2, axis=2)
        floor_mask = dist_map <= 8.0
        self._is_floor = floor_mask
        is_floor = floor_mask.astype(np.float32)

        sec_w = 160 // 3
        self.sector_clearances["left"] = round(float(np.clip(np.mean(is_floor[:, :sec_w]), 0.05, 0.98)), 2)
        self.sector_clearances["center"] = round(float(np.clip(np.mean(is_floor[:, sec_w:2 * sec_w]), 0.05, 0.98)), 2)
        self.sector_clearances["right"] = round(float(np.clip(np.mean(is_floor[:, 2 * sec_w:]), 0.05, 0.98)), 2)

    def process_frame(
        self,
        bgr_frame: np.ndarray,
        current_distance_cm: Optional[float] = None,
        dist_ts: Optional[float] = None,
    ) -> Tuple[str, Optional[int], np.ndarray, Dict[str, Any]]:
        """Drive decision for one frame; the follower also owns the camera servo.

        A fixed left/right look (or a manual slider hold) overrides the servo angle.
        While the camera is turned the lane view is skewed, so a driving car stops
        until it is back at centre. Obstacle warnings from the side view are added
        to the telemetry and the HUD; they advise, they never start the car.
        """
        cmd, servo, annotated, telem = self._process_frame_core(bgr_frame, current_distance_cm, dist_ts)
        now = time.time()
        self.side_look.update(now, bgr_frame, self._running_floor_mean,
                              getattr(self, "_running_floor_std", None))
        angle = self.side_look.angle(now)
        if angle is not None:
            servo = angle
            if self.is_active and cmd != "STOP":
                cmd = "STOP"
                self.latest_cmd = "STOP"
        status = self.side_look.status(now)
        telem["side_look"] = status
        self._draw_side_look(annotated, status)
        return cmd, servo, annotated, telem

    def _draw_side_look(self, frame: np.ndarray, status: Dict[str, Any]) -> None:
        """Camera-position label and the side obstacle warning on the HUD."""
        if frame is None:
            return
        if status["active"] and status["angle"] is not None:
            angle = status["angle"]
            label = ("CAM LEFT" if angle > self.side_look.centre_angle + 3
                     else "CAM RIGHT" if angle < self.side_look.centre_angle - 3 else "CAM CENTRE")
            cv2.putText(frame, f"{label} {angle}", (10, 52), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 220, 120), 1, cv2.LINE_AA)
        if status["warning"]:
            cv2.rectangle(frame, (8, 58), (300, 82), (0, 0, 200), -1)
            cv2.putText(frame, f"! {status['warning']}", (12, 75), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (255, 255, 255), 1, cv2.LINE_AA)

    def _process_frame_core(
        self,
        bgr_frame: np.ndarray,
        current_distance_cm: Optional[float] = None,
        dist_ts: Optional[float] = None,
    ) -> Tuple[str, Optional[int], np.ndarray, Dict[str, Any]]:
        """
        Main 4-in-1 Perception and Navigation Processing Loop.
        Returns:
            motor_cmd ('FORWARD', 'LEFT', 'RIGHT', 'STOP')
            servo_cmd (int angle 0-180 or None)
            annotated_frame (HUD overlay)
            telemetry (dict)
        """
        now = time.time()
        self.latest_distance = current_distance_cm
        h, w = bgr_frame.shape[:2]
        annotated = bgr_frame.copy()

        # ── 1. ArUco Marker Detection & 3D Pose Estimation ────────────────────
        # Marker detection is the most expensive CV step (~half the frame budget on a
        # Pi 4) and tags move slowly, so refresh it every 3rd frame and reuse the result.
        if self._frame_idx % 3 == 0 or not hasattr(self, "_cached_tags"):
            self._cached_tags = self.aruco_navigator.detect_markers(bgr_frame)
        detected_tags = self._cached_tags
        annotated = self.aruco_navigator.draw_hud(annotated, detected_tags)

        # ── 2. Computer Vision Lane Detection (Sanoguzhan LKA) ───────────────
        if self.side_look.active:
            # The camera is turned: the ground-plane geometry no longer applies, and pale
            # bars on a wall would be read as tape. Skip lane detection until it is back
            # at centre (a driving car is held stopped meanwhile, see process_frame).
            lane_res = LaneResult(direction=Direction.NO_LANE)
        else:
            lane_res = self.lane_detector.detect_from_bgr(bgr_frame)
        self.latest_birdseye = getattr(lane_res, "birdseye_frame", None)
        if lane_res.annotated_frame is not None:
            annotated = lane_res.annotated_frame.copy()
            annotated = self.aruco_navigator.draw_hud(annotated, detected_tags)
        self.latest_error = lane_res.error
        self.latest_direction = lane_res.direction.value

        # ── 3. Visual Click Destination Tracking ──────────────────────────────
        target = self.visual_navigator.find_destination_marker(bgr_frame)
        is_targeting_pixel = (target is not None)
        if is_targeting_pixel and target is not None:
            self._annotate_destination_hud(annotated, target)
            self.dest_distance_est_m = getattr(target, 'estimated_distance_m', None)

        # ── 4. Spatial Clearance Analysis ─────────────────────────────────────
        self._update_sector_clearance(bgr_frame)
        sonar_fresh = (current_distance_cm if (dist_ts is None or now - dist_ts < 0.5) else None)
        if self._is_floor is not None:
            self.fs = self.free_space.update(self._is_floor, sonar_fresh)

        # ── 5. Stop Sign Detection ────────────────────────────────────────────
        has_stop_sign, stop_bbox = self.stop_sign_detector.detect(bgr_frame)
        self.stop_sign_visible = has_stop_sign
        if has_stop_sign and stop_bbox is not None:
            sx, sy, sw, sh = stop_bbox
            cv2.rectangle(annotated, (sx, sy), (sx + sw, sy + sh), (0, 0, 255), 3)
            cv2.putText(annotated, "STOP SIGN", (sx, max(20, sy - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

        # ── 6. ONNX Neural Pilot Inference (Active Drive Only) ────────────────
        # Only run deep learning inference when autonomous driving is actively enabled
        # to prevent unnecessary CPU load and excessive Pi heating during idle/standby.
        self._frame_idx += 1
        if self.ai_loaded and self.is_active and (self._frame_idx % 2 == 0):  # ~halves CPU; steering is smoothed anyway
            try:
                t0 = time.time()
                steer, throttle = self.pilot_model.predict(bgr_frame)
                self.ai_inference_ms = round((time.time() - t0) * 1000.0, 1)
                self.ai_steer = steer
            except Exception:
                pass

        # Draw HUD gauges
        self._annotate_sector_radar(annotated, self.sector_clearances)

        # If vehicle is inactive, remain idle
        if not self.is_active:
            self.state = LineFollowerState.IDLE
            self.latest_cmd = "STOP"
            self._draw_hud_reticle(annotated)
            self._draw_status_banner(annotated, "● SYSTEM IDLE · READY", (180, 190, 205))
            self._draw_bottom_telemetry_hud(annotated, 0.0, current_distance_cm, "STOP")
            return "STOP", 90, annotated, self.get_telemetry()

        # ── 7. Stop Sign Safety Pause ─────────────────────────────────────────
        if has_stop_sign and (now - self._last_stop_sign_time) > 10.0 and self.state != LineFollowerState.STOP_SIGN_PAUSE:
            self.state = LineFollowerState.STOP_SIGN_PAUSE
            self._stop_pause_start = now
            self._last_stop_sign_time = now

        if self.state == LineFollowerState.STOP_SIGN_PAUSE:
            elapsed = now - self._stop_pause_start
            remaining = max(0.0, self._stop_pause_duration - elapsed)
            if remaining > 0:
                self.latest_cmd = "STOP"
                self._draw_status_banner(annotated, f"STOP SIGN PAUSE ({remaining:.1f}s)", (0, 0, 255))
                self._draw_survey_dial(annotated, 90, "PAUSE")
                return "STOP", 90, annotated, self.get_telemetry()
            else:
                self.state = LineFollowerState.HYBRID_NAVIGATING

        # ── 8. Sonar-driven reactive avoidance (brake → pivot to open side → bypass) ──
        # Vision backup for what a single narrow sonar beam misses (thin, angled or soft
        # objects): a blocked centre view for several frames counts as a close obstacle.
        center_clear = self.sector_clearances.get("center", 0.5)
        self._vision_block_frames = self._vision_block_frames + 1 if center_clear < VISION_BLOCK_CLEARANCE else 0
        sonar_cm = current_distance_cm
        left_clear = self.sector_clearances.get("left", 0.5)
        right_clear = self.sector_clearances.get("right", 0.5)
        fs = self.fs
        if fs is not None and fs.calibrated:
            # Sonar-calibrated camera ranges: the camera also sees what the beam misses
            # (off-axis, thin or angled obstacles) and sees far walls early.
            if sonar_cm is not None and self._detour_phase in (0, 4):
                sonar_cm = min(sonar_cm, fs.forward_cm)
            left_clear, right_clear = fs.side_clearance()
        elif (self._vision_block_frames >= VISION_BLOCK_FRAMES and sonar_cm is not None
                and self._detour_phase == 0):
            sonar_cm = min(sonar_cm, self.avoider.cfg.stop_cm - 1.0)   # uncalibrated fallback
        if self.lane_layout == "three_line":
            # A detour may only turn toward the centre / the other lane, never over a solid outer line.
            left_clear, right_clear = constrain_side_clearance(left_clear, right_clear, lane_res)
        decision = self.avoider.update(now, sonar_cm, dist_ts, left_clear, right_clear)
        self._detour_phase = decision.phase
        if decision.cmd is not None:
            cmd = decision.cmd
            if not self.enable_auto_avoidance and decision.state not in (AvoidState.SENSOR_FAULT,):
                cmd = "STOP"
            self.state = (LineFollowerState.OBSTACLE_STOP
                          if decision.state in (AvoidState.STUCK, AvoidState.SENSOR_FAULT)
                          else LineFollowerState.OBSTACLE_AVOIDING_DETOUR)
            self.latest_cmd = cmd
            self._last_command = "STOP"
            self._steer_smooth = 0.0
            self._vision_block_frames = 0
            self.fused_steer = -0.5 if cmd == "LEFT" else (0.5 if cmd == "RIGHT" else 0.0)
            color = (0, 0, 255) if self.state == LineFollowerState.OBSTACLE_STOP else (0, 140, 255)
            self._draw_status_banner(annotated, decision.note, color)
            self._draw_bottom_telemetry_hud(annotated, self.fused_steer, current_distance_cm, cmd)
            return cmd, 90, annotated, self.get_telemetry()
        self._avoid_gate_forward = decision.gate_forward

        # ── 9. Normal Lane Following & ArUco Destination Homing ───────────────
        # Combines Lane Assistant (proper lane tracking) + ArUco Landmark Destination
        target_tag_id = self.aruco_navigator.target_tag_id
        target_pose = detected_tags.get(target_tag_id) if target_tag_id is not None else None

        # Check Tag Arrival Condition
        if target_pose is not None and target_pose.distance_cm <= self.aruco_navigator.arrival_dist_cm:
            if self.route_mode_enabled and len(self.route_list) > 1:
                # In Linked Route List Mode: pause momentarily, then auto-advance to next link
                if not self._route_dwelling:
                    self._route_dwelling = True
                    self._route_arrival_time = now

                elapsed_dwell = now - self._route_arrival_time
                if elapsed_dwell < self._route_arrival_dwell_s:
                    self.state = LineFollowerState.ARRIVED_AT_DESTINATION
                    self.latest_cmd = "STOP"
                    cur_tag = self.route_list[self.route_index]
                    self._draw_status_banner(annotated, f"🎯 ARRIVED AT LINK #{cur_tag} ({target_pose.distance_cm:.0f}cm) — ADVANCING...", (0, 255, 0))
                    self._draw_survey_dial(annotated, 90, "CENTER")
                    self._draw_route_link_list_hud(annotated)
                    return "STOP", 90, annotated, self.get_telemetry()
                else:
                    # Dwell complete: advance to the next link in the route list
                    self._route_dwelling = False
                    self.route_index = (self.route_index + 1) % len(self.route_list)
                    next_tag = self.route_list[self.route_index]
                    self.aruco_navigator.set_target_tag(next_tag)
                    self.state = LineFollowerState.HYBRID_NAVIGATING
            else:
                self.state = LineFollowerState.ARRIVED_AT_DESTINATION
                self.latest_cmd = "STOP"
                self._draw_status_banner(annotated, f"🎯 ARRIVED AT DESTINATION TAG #{target_pose.tag_id} ({target_pose.distance_cm:.0f}cm)!", (0, 255, 0))
                self._draw_survey_dial(annotated, 90, "CENTER")
                self._draw_route_link_list_hud(annotated)
                return "STOP", 90, annotated, self.get_telemetry()

        # Check Click-Destination Arrival Condition
        if is_targeting_pixel and target is not None:
            has_reached_pixel = (
                target.target_y > (h - 60)
                or (target.target_y > (h - 120) and current_distance_cm is not None and current_distance_cm < 25.0)
            )
            if has_reached_pixel:
                self.state = LineFollowerState.ARRIVED_AT_DESTINATION
                self.latest_cmd = "STOP"
                self.visual_navigator.clear_manual_destination()
                self._draw_status_banner(annotated, "🎯 ARRIVED AT CLICK DESTINATION!", (0, 255, 0))
                self._draw_route_link_list_hud(annotated)
                return "STOP", 90, annotated, self.get_telemetry()

        # Calculate Individual Steering Proposals:
        # A. ArUco Waypoint Steer
        aruco_steer = 0.0
        has_aruco_guidance = False
        if target_pose is not None:
            # Bearing in degrees (-30 to +30 deg maps to -1.0 to +1.0)
            aruco_steer = float(np.clip(target_pose.bearing_deg / 25.0, -1.0, 1.0))
            has_aruco_guidance = True
        elif is_targeting_pixel and target is not None:
            aruco_steer = float(np.clip(target.bearing_deg / 25.0, -1.0, 1.0))
            has_aruco_guidance = True

        # B. Lane Centering Steer
        lane_steer = 0.0
        has_lane_guidance = False
        if lane_res.contour_found or lane_res.lane_center_x is not None:
            # Lateral error in px (-120 to +120 maps to -1.0 to +1.0)
            lane_steer = float(np.clip(lane_res.error / 100.0, -1.0, 1.0))
            has_lane_guidance = True

        # C. ONNX Pilot Steer
        onnx_steer = float(np.clip(self.ai_steer, -1.0, 1.0)) if self.ai_loaded else 0.0

        # D. Fusion Arbiter & Mode Selection
        if self.nav_mode == NavigationMode.AI_ONNX_PILOT:
            fused = onnx_steer
        elif self.nav_mode == NavigationMode.LANE_FOLLOWING:
            fused = lane_steer if has_lane_guidance else onnx_steer
        elif self.nav_mode == NavigationMode.SANOGUZHAN_CLASSIC:
            if has_lane_guidance:
                fused = lane_steer
            else:
                self.state = LineFollowerState.LOST_LANE
                self.latest_cmd = "STOP"
                self._draw_status_banner(annotated, "SANOGUZHAN CLASSIC: LANE LOST", (0, 0, 255))
                self._draw_bottom_telemetry_hud(annotated, 0.0, current_distance_cm, "STOP")
                return "STOP", 90, annotated, self.get_telemetry()
        elif self.nav_mode == NavigationMode.ARUCO_WAYPOINT:
            fused = aruco_steer if has_aruco_guidance else onnx_steer
        else:
            # HYBRID_FUSION mode: Follow lane properly, home to ArUco destination when targeted
            if has_aruco_guidance:
                if has_lane_guidance:
                    fused = (0.55 * lane_steer) + (0.45 * aruco_steer)
                else:
                    fused = aruco_steer
            elif has_lane_guidance:
                # Follow lane properly with high precision PD centering
                fused = lane_steer
            else:
                # Fallback to ONNX model free space when no lane lines are visible
                fused = onnx_steer

        # Enforce Hard Lane Boundaries on steering:
        # Clamps output within [min_allowed_steering, max_allowed_steering] so vehicle cannot
        # breach or drive outside detected lane lines, preventing room wall / drawer collisions.
        min_allowed = getattr(lane_res, "min_allowed_steering", -1.0)
        max_allowed = getattr(lane_res, "max_allowed_steering", 1.0)
        if min_allowed > -1.0 or max_allowed < 1.0:
            fused = float(np.clip(fused, min_allowed, max_allowed))

        # Smooth frame-to-frame jitter (EMA) before thresholding.
        self._steer_smooth = 0.6 * self._steer_smooth + 0.4 * fused
        if self.lane_layout == "three_line":
            # The running average must not carry an earlier outward command over a solid line.
            self._steer_smooth = float(np.clip(self._steer_smooth, min_allowed, max_allowed))
        fused = self._steer_smooth
        # Route through free space: keep the goal/lane heading unless the camera (ranged by the
        # sonar) shows it blocked, then take the nearest open direction.
        if fs is not None and fs.calibrated:
            heading = choose_heading(fs.ranges_cm, fused * 25.0, STEER_CLEAR_CM)
            if heading is not None:
                fused = float(np.clip(heading / 25.0, -1.0, 1.0))
                if self.lane_layout == "three_line":
                    # Free space must not steer the car over a solid outer line.
                    fused = float(np.clip(fused, min_allowed, max_allowed))
        self.fused_steer = round(fused, 2)

        # E. Camera Servo Pan Behavior in Hybrid Mode:
        # If searching for an ArUco tag that is currently not in view, perform smart horizon scan
        servo_out = 90
        if target_tag_id is not None and not has_aruco_guidance and self.servo_search:
            servo_out = self.servo_scanner.step_search(now)
            survey_label = f"SEARCH_TAG_{target_tag_id}"
        elif target_tag_id is not None and not has_aruco_guidance:
            survey_label = f"TAG_{target_tag_id}_NOT_VISIBLE"   # no sweep: the camera stays put
        else:
            if self.servo_scanner.is_sweeping:
                self.servo_scanner.reset()
            survey_label = "TRACKING_CENTER"

        # Determine Final Vehicle Drive Command
        # Hysteresis: enter a turn at 0.20, hold it until steer falls under 0.10.
        enter, leave = 0.20, 0.10
        prev = self._last_command
        if self.fused_steer < -enter or (prev == "LEFT" and self.fused_steer < -leave):
            final_cmd = "LEFT"
        elif self.fused_steer > enter or (prev == "RIGHT" and self.fused_steer > leave):
            final_cmd = "RIGHT"
        else:
            final_cmd = "FORWARD"

        # A panned camera sees a skewed lane, so stay put until it is back at centre.
        if servo_out != 90 and self.servo_scanner.is_sweeping:
            final_cmd = "STOP"

        # Proximity Caution Alert from Stationary Ultrasonic Sensor
        caution_text = ""
        if current_distance_cm is not None and current_distance_cm < self.caution_distance_cm:
            caution_text = f" [CAUTION {int(current_distance_cm)}cm]"

        self.state = LineFollowerState.HYBRID_NAVIGATING
        self.latest_cmd = final_cmd
        self._last_command = final_cmd

        # Inside the caution band the avoider pulses forward motion (creep) so the
        # car sheds speed before it has to brake; turns are never gated.
        if final_cmd == "FORWARD" and not self._avoid_gate_forward:
            final_cmd = "STOP"
            self.latest_cmd = "STOP"

        # Status text formatting
        tag_str = f"Tag #{target_tag_id}" if target_tag_id is not None else "Free Transit"
        banner_msg = f"FUSION AUTO: {final_cmd} (Steer: {self.fused_steer:+.2f} | {tag_str}){caution_text}"
        banner_color = (0, 200, 255) if caution_text else (0, 220, 0)
        self._draw_status_banner(annotated, banner_msg, banner_color)

        # Render sleek HUD Overlays
        self._draw_hud_reticle(annotated)
        self._draw_fused_steering_trajectory(annotated, self.fused_steer)
        self._draw_bottom_telemetry_hud(annotated, self.fused_steer, current_distance_cm, final_cmd)
        self._draw_route_link_list_hud(annotated)

        return final_cmd, servo_out, annotated, self.get_telemetry()

    def _draw_hud_reticle(self, frame: np.ndarray) -> None:
        """Draws a subtle, sleek aerospace center horizon reticle at optical center."""
        h, w = frame.shape[:2]
        cx, cy = w // 2, int(h * 0.55)
        gap, arm = 7, 13
        c_reticle = (180, 240, 210)
        cv2.line(frame, (cx - gap - arm, cy), (cx - gap, cy), c_reticle, 1, cv2.LINE_AA)
        cv2.line(frame, (cx + gap, cy), (cx + gap + arm, cy), c_reticle, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy - gap - arm), (cx, cy - gap), c_reticle, 1, cv2.LINE_AA)
        cv2.line(frame, (cx, cy + gap), (cx, cy + gap + arm), c_reticle, 1, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 2, (0, 255, 200), -1, cv2.LINE_AA)

    def _draw_fused_steering_trajectory(self, frame: np.ndarray, fused_steer: float) -> None:
        """Draws a sleek translucent dynamic guidance trajectory arc on the road plane."""
        h, w = frame.shape[:2]
        cx = w // 2
        bottom_y = h - 22
        target_x = int(cx + fused_steer * (w * 0.38))
        target_y = int(h * 0.60)

        pts = np.array([
            [cx - 14, bottom_y],
            [int(cx - 8 + fused_steer * 35), int(h * 0.78)],
            [target_x, target_y],
            [int(cx + 8 + fused_steer * 35), int(h * 0.78)],
            [cx + 14, bottom_y]
        ], np.int32)

        overlay = frame.copy()
        cv2.fillPoly(overlay, [pts], (180, 240, 100))
        cv2.addWeighted(overlay, 0.22, frame, 0.78, 0, frame)
        cv2.circle(frame, (target_x, target_y), 4, (0, 255, 220), -1, cv2.LINE_AA)

    def _draw_bottom_telemetry_hud(
        self,
        frame: np.ndarray,
        fused_steer: float,
        dist_cm: Optional[float],
        cmd: str
    ) -> None:
        """Draws a clean, non-overlapping bottom telemetry bar with glassmorphic backing."""
        h, w = frame.shape[:2]
        by = h - 20
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, by), (w, h), (12, 16, 26), -1)
        cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)
        cv2.line(frame, (0, by), (w, by), (45, 55, 75), 1)

        # Formatting items
        cmd_col = (0, 255, 200) if cmd == "FORWARD" else ((0, 140, 255) if cmd in ("LEFT", "RIGHT") else (100, 100, 255))
        cv2.putText(frame, f"CMD: {cmd}", (10, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.34, cmd_col, 1, cv2.LINE_AA)

        steer_txt = f"STEER: {fused_steer:+.2f}"
        cv2.putText(frame, steer_txt, (w // 2 - 35, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (220, 225, 235), 1, cv2.LINE_AA)

        if dist_cm is not None and dist_cm < 400:
            d_col = (50, 230, 100) if dist_cm > 60 else ((0, 180, 255) if dist_cm > 35 else (0, 0, 255))
            d_txt = f"US: {int(dist_cm)}cm"
        else:
            d_col = (50, 230, 100)
            d_txt = "US: CLEAR"
        cv2.putText(frame, d_txt, (w - 78, h - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.34, d_col, 1, cv2.LINE_AA)

    def _draw_route_link_list_hud(self, frame: np.ndarray) -> None:
        """Renders the linked waypoint route chain inside a sleek translucent badge."""
        if not self.route_mode_enabled or not self.route_list:
            return
        chain_tokens = [f"[{t}]" if idx == self.route_index else f"{t}" for idx, t in enumerate(self.route_list)]
        route_str = "ROUTE: " + " -> ".join(chain_tokens)
        bx, by = 8, 26
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.34
        (tw, th), _ = cv2.getTextSize(route_str, font, scale, 1)

        overlay = frame.copy()
        cv2.rectangle(overlay, (bx, by), (bx + tw + 10, by + th + 8), (15, 20, 32), -1)
        cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)
        cv2.rectangle(frame, (bx, by), (bx + tw + 10, by + th + 8), (56, 189, 248), 1, cv2.LINE_AA)
        cv2.putText(frame, route_str, (bx + 5, by + th + 4), font, scale, (56, 189, 248), 1, cv2.LINE_AA)

    def _draw_fused_steering_dial(
        self,
        frame: np.ndarray,
        fused: float,
        aruco_s: float,
        lane_s: float,
        onnx_s: float
    ) -> None:
        """Delegates to trajectory arc to prevent cluttering text."""
        self._draw_fused_steering_trajectory(frame, fused)

    def _annotate_destination_hud(self, frame: np.ndarray, target: VisualTarget) -> np.ndarray:
        """Draws a sleek holographic destination reticle with distance vector."""
        h, w = frame.shape[:2]
        cx, cy = w // 2, h - 20
        tx, ty = target.target_x, target.target_y

        cv2.line(frame, (cx, cy), (tx, ty), (0, 240, 200), 1, cv2.LINE_AA)
        cv2.circle(frame, (tx, ty), 12, (0, 255, 220), 1, cv2.LINE_AA)
        cv2.circle(frame, (tx, ty), 2, (0, 255, 100), -1, cv2.LINE_AA)

        dist_label = f"{self.dest_distance_est_m:.1f}m" if self.dest_distance_est_m else "GOAL"
        lbl = f"🎯 {dist_label} ({target.bearing_deg:+.0f}°)"
        cv2.putText(frame, lbl, (tx + 14, ty + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 255, 220), 1, cv2.LINE_AA)
        return frame

    def _annotate_sector_radar(self, frame: np.ndarray, clearances: Dict[str, float]) -> None:
        """Deprecated from camera HUD; handled cleanly in dashboard Radar Card."""
        pass

    def _draw_survey_dial(self, frame: np.ndarray, angle: int, mode_str: str) -> None:
        """Deprecated from camera HUD; handled cleanly in dashboard Servo Card."""
        pass

    def _draw_status_banner(self, frame: np.ndarray, text: str, color_bgr: Tuple[int, int, int]) -> None:
        """Draws a compact translucent top pill badge rather than an opaque screen block."""
        h, w = frame.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.38
        (tw, th), _ = cv2.getTextSize(text, font, scale, 1)
        bx = (w - tw) // 2 - 8
        by = 5
        bw = tw + 16
        bh = th + 9

        overlay = frame.copy()
        cv2.rectangle(overlay, (bx, by), (bx + bw, by + bh), (15, 20, 32), -1)
        cv2.addWeighted(overlay, 0.75, frame, 0.25, 0, frame)
        cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), color_bgr, 1, cv2.LINE_AA)
        cv2.putText(frame, text, (bx + 8, by + bh - 3), font, scale, color_bgr, 1, cv2.LINE_AA)

    def get_telemetry(self) -> Dict[str, Any]:
        with self._lock:
            detected_tags_list = []
            for tag_id, p in self.aruco_navigator.latest_detected_tags.items():
                detected_tags_list.append({
                    "id": p.tag_id,
                    "dist_cm": p.distance_cm,
                    "bearing_deg": p.bearing_deg,
                    "x_cm": p.x_cm,
                    "z_cm": p.z_cm,
                })

            target = self.visual_navigator.manual_target
            if isinstance(target, (tuple, list)):
                dest_coords = [int(target[0]), int(target[1])]
            elif target is not None:
                dest_coords = [getattr(target, 'target_x', 0), getattr(target, 'target_y', 0)]
            else:
                dest_coords = None

            return {
                "active": self.is_active,
                "state": self.state.value,
                "nav_mode": self.nav_mode.value,
                "error_px": round(self.latest_error, 1),
                "direction": self.latest_direction,
                "distance_cm": self.latest_distance,
                "command": self.latest_cmd,
                "fused_steer": self.fused_steer,
                "servo_angle": (self.side_look.angle(time.time())
                                if self.side_look.active
                                else (self._detour_servo_angle if self._detour_phase > 0
                                      else self.servo_scanner.current_angle)),
                "side_look": self.side_look.status(time.time()),
                "detour_phase": self._detour_phase,
                "avoid_state": self.avoider.state.value,
                "stop_sign": self.stop_sign_visible,
                "ai_loaded": self.ai_loaded,
                "ai_steer": round(self.ai_steer, 2),
                "ai_inference_ms": self.ai_inference_ms,
                "dest_active": target is not None,
                "dest_target": dest_coords,
                "dest_distance_m": self.dest_distance_est_m,
                "sector_clearances": self.sector_clearances,
                "free_space_cm": [round(r) for r in self.fs.ranges_cm] if self.fs else None,
                "range_calibrated": bool(self.fs and self.fs.calibrated),
                "aruco_tags": detected_tags_list,
                "aruco_target_id": self.aruco_navigator.target_tag_id,
                "route_mode": self.route_mode_enabled,
                "route_list": list(self.route_list),
                "route_index": self.route_index,
                "route_active_tag": self.route_list[self.route_index] if self.route_list else None,
            }
