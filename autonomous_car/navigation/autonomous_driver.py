import os
import sys
import time
import logging
import threading
from typing import Optional, Dict, Any, List

import numpy as np

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from autonomous_car.config import AppConfig, AutonomousConfig
from autonomous_car.hardware.motors import MotorController
from autonomous_car.hardware.servo import ServoController
from autonomous_car.vision.camera import Camera
from autonomous_car.ai.classes import ActionEnum, DecisionResult, DetectionResult, BoundingBox
from autonomous_car.ai.detector import YOLODetector
from autonomous_car.ai.tracker import ObjectTracker
from autonomous_car.ai.risk_analyzer import RiskAnalyzer
from autonomous_car.ai.corridor_scanner import CorridorScanner
from autonomous_car.ai.server_client import ServerAIClient
from autonomous_car.ai.visualization import Visualizer

logger = logging.getLogger(__name__)


class AutonomousDriver:
    """
    Intelligent Autonomous Driving Controller for RC Car.
    Fuses on-board YOLO perception, real-time corridor hazard scanning,
    pan-servo path discovery, and edge server AI model guidance.
    """

    def __init__(
        self,
        config: AutonomousConfig,
        motors: MotorController,
        servo: ServoController,
        camera: Camera,
    ) -> None:
        self.config: AutonomousConfig = config
        self.motors: MotorController = motors
        self.servo: ServoController = servo
        self.camera: Camera = camera

        # State management
        self.is_enabled: bool = self.config.enabled
        self.mode: str = self.config.mode.lower()  # "hybrid", "local_ai", "server_ai"
        self.cruise_speed: float = self.config.cruise_speed
        self.avoid_speed: float = self.config.avoid_speed
        self.show_overlay: bool = self.config.show_ai_overlay

        # AI Subsystems
        self.detector = YOLODetector(
            model_path=self.config.model_name,
            confidence=self.config.confidence_threshold,
            target_classes=self.config.target_classes,
        )
        self.tracker = ObjectTracker(self.detector)
        self.risk_analyzer = RiskAnalyzer()
        self.corridor_scanner = CorridorScanner(lane_width_ratio=self.config.lane_width_ratio)
        self.server_client = ServerAIClient(base_url=self.config.server_url)

        # Telemetry & Runtime state
        self._current_decision: DecisionResult = DecisionResult(
            action=ActionEnum.STOP,
            reason="Autonomous system standby (Manual mode)",
            hazard_detected=False
        )
        self._current_detections: List[DetectionResult] = []
        self._current_risks: Dict[int, Dict[str, float]] = {}
        self._fps: float = 0.0
        self._latency_ms: float = 0.0
        self._last_loop_time: float = time.time()
        self._is_scanning_path: bool = False
        self._stop_wait_start: float = 0.0
        self._is_waiting_at_stop: bool = False

        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Start edge server monitoring in background
        self.server_client.start()

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------
    def start(self) -> None:
        """Starts background perception and autonomous driving loop."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._driving_loop, daemon=True, name="auto-driver-loop")
        self._thread.start()
        logger.info("[AUTO DRIVER] Background autonomous perception loop started.")

    def stop(self) -> None:
        self._running = False
        self.is_enabled = False
        self.motors.stop()
        self.server_client.stop()

    def set_enabled(self, enabled: bool) -> bool:
        with self._lock:
            self.is_enabled = bool(enabled)
            if not self.is_enabled:
                self.motors.stop()
                self._current_decision = DecisionResult(
                    action=ActionEnum.STOP,
                    reason="Autonomous Mode Disengaged (Manual Control Active)"
                )
            else:
                self._current_decision = DecisionResult(
                    action=ActionEnum.FORWARD,
                    reason="Autonomous Mode Engaged"
                )
        logger.info(f"[AUTO DRIVER] Autonomous Drive: {'ENABLED' if self.is_enabled else 'DISABLED'}")
        return self.is_enabled

    def set_mode(self, mode: str) -> str:
        mode_clean = mode.lower().strip()
        if mode_clean in ["hybrid", "local_ai", "server_ai"]:
            with self._lock:
                self.mode = mode_clean
            logger.info(f"[AUTO DRIVER] Mode set to: {self.mode.upper()}")
        return self.mode

    def set_config_params(self, params: Dict[str, Any]) -> None:
        with self._lock:
            if "cruise_speed" in params:
                self.cruise_speed = max(0.2, min(1.0, float(params["cruise_speed"])))
            if "avoid_speed" in params:
                self.avoid_speed = max(0.2, min(1.0, float(params["avoid_speed"])))
            if "show_overlay" in params:
                self.show_overlay = bool(params["show_overlay"])
            if "server_url" in params:
                self.server_client.update_server_url(str(params["server_url"]))

    # -------------------------------------------------------------------------
    # Main Autonomous Perception & Driving Loop
    # -------------------------------------------------------------------------
    def _driving_loop(self) -> None:
        fps_smoothing = 0.9
        last_t = time.time()

        while self._running:
            try:
                loop_start = time.time()

                # 1. Fetch raw frame from camera
                bgr_frame = self.camera.get_bgr_frame()
                if bgr_frame is None:
                    time.sleep(0.04)
                    continue

                h, w = bgr_frame.shape[:2]

                # 2. Perception & Decision computation
                decision = self._evaluate_environment_and_decide(bgr_frame, w, h)

                # 3. Apply Decision to Motors if Autonomous mode is active
                if self.is_enabled and not self._is_scanning_path:
                    self._execute_motor_action(decision)

                # 4. Measure metrics
                now = time.time()
                dt = now - last_t
                last_t = now
                if dt > 0:
                    current_fps = 1.0 / dt
                    self._fps = (self._fps * fps_smoothing) + (current_fps * (1.0 - fps_smoothing))

                self._latency_ms = (now - loop_start) * 1000.0

            except Exception as e:
                logger.error(f"[AUTO DRIVER] Unexpected loop error: {e}")
                if self.is_enabled:
                    self.motors.stop()
                time.sleep(0.05)

            time.sleep(0.02)  # ~35-40 Hz cycle cap

    # -------------------------------------------------------------------------
    # Perception Evaluation & Path Finding
    # -------------------------------------------------------------------------
    def _evaluate_environment_and_decide(self, frame: np.ndarray, w: int, h: int) -> DecisionResult:
        # Check Server AI if in SERVER_AI or HYBRID mode
        if self.mode in ("hybrid", "server_ai") and self.server_client.is_connected:
            server_dec = self.server_client.get_latest_decision(max_age_sec=0.7)
            if server_dec is not None:
                with self._lock:
                    self._current_decision = server_dec
                return server_dec
            elif self.mode == "server_ai":
                # Server disconnected and in pure server mode -> fail-safe stop
                stop_dec = DecisionResult(
                    action=ActionEnum.STOP,
                    reason="Server Model Offline (Waiting for Server AI...)",
                    source="server"
                )
                with self._lock:
                    self._current_decision = stop_dec
                return stop_dec

        # --- Local Perception Pipeline ---
        # A. YOLO Tracking & Detections
        detections = self.tracker.track(frame)
        risks = self.risk_analyzer.update(detections, time.time(), w, h)

        # B. Real-time Corridor Obstacle Scanner
        corridor_info = self.corridor_scanner.analyze(frame)
        cl_left = corridor_info["left_clearance"]
        cl_right = corridor_info["right_clearance"]
        cl_center = corridor_info["center_clearance"]
        center_blocked_by_cv = corridor_info["center_blocked"]

        with self._lock:
            self._current_detections = detections
            self._current_risks = risks

        # C. Rule Engine for Objects (Bus, Vehicles, Pedestrians, Obstacles)
        lane_margin = (1.0 - self.config.lane_width_ratio) / 2.0
        lane_x1 = w * lane_margin
        lane_x2 = w * (1.0 - lane_margin)

        critical_hazard: Optional[DetectionResult] = None
        bus_hazard: Optional[DetectionResult] = None
        stop_sign_detected = False
        avoid_hazard_side: Optional[str] = None

        for d in detections:
            bbox = d.bbox
            cls_lower = d.class_name.lower()
            risk_data = risks.get(d.track_id or id(d), {"risk_score": 0.1, "ttc": float('inf'), "norm_area": 0.0})
            risk_score = risk_data["risk_score"]
            ttc = risk_data["ttc"]
            norm_area = risk_data["norm_area"]

            # 1. Immediate Rule: Stop on Bus!
            if "bus" in cls_lower:
                # Any bus visible in driving field
                if norm_area > 0.03 or not (bbox.x2 < lane_x1 or bbox.x1 > lane_x2):
                    bus_hazard = d
                    break

            # 2. Stop Sign handling
            if "stop" in cls_lower and norm_area > 0.03:
                stop_sign_detected = True

            # 3. Check corridor intersection
            in_corridor = not (bbox.x2 < lane_x1 or bbox.x1 > lane_x2)

            if in_corridor or risk_score > 0.65:
                if norm_area >= self.config.stop_area_threshold or ttc < 1.5 or risk_score > 0.85:
                    critical_hazard = d
                    break
                elif norm_area >= min(self.config.slow_area_threshold, 0.03) or ttc < 3.0 or in_corridor:
                    # Obstacle is ahead in corridor, determine avoidance steering side
                    if bbox.center_x < (w / 2):
                        avoid_hazard_side = "left"   # Obstacle on left -> avoid by steering right
                    else:
                        avoid_hazard_side = "right"  # Obstacle on right -> avoid by steering left

        # ---------------------------------------------------------------------
        # DECISION LOGIC:
        # ---------------------------------------------------------------------

        # Priority 1: Bus detected in path -> IMMEDIATE STOP!
        if bus_hazard:
            dec = DecisionResult(
                action=ActionEnum.STOP,
                reason=f"EMERGENCY STOP: Bus detected in path (Conf {bus_hazard.confidence:.0%})",
                hazard_detected=True,
                obstacle_side="center",
                clearance_left=cl_left,
                clearance_right=cl_right,
                source="local"
            )
            with self._lock: self._current_decision = dec
            return dec

        # Priority 2: Stop Sign
        if stop_sign_detected:
            now = time.time()
            if not self._is_waiting_at_stop:
                self._is_waiting_at_stop = True
                self._stop_wait_start = now
                dec = DecisionResult(
                    action=ActionEnum.STOP,
                    reason="Stop sign detected. Bringing vehicle to halt.",
                    hazard_detected=True,
                    source="local"
                )
                with self._lock: self._current_decision = dec
                return dec
            else:
                elapsed = now - self._stop_wait_start
                if elapsed < 2.0:
                    dec = DecisionResult(
                        action=ActionEnum.WAIT,
                        reason=f"Waiting at stop sign ({elapsed:.1f}s / 2.0s)",
                        hazard_detected=True,
                        source="local"
                    )
                    with self._lock: self._current_decision = dec
                    return dec
        else:
            self._is_waiting_at_stop = False

        # Priority 3: Critical Collision Hazard in Corridor
        if critical_hazard:
            dec = DecisionResult(
                action=ActionEnum.STOP,
                reason=f"EMERGENCY STOP: {critical_hazard.class_name.upper()} directly in driving path!",
                hazard_detected=True,
                obstacle_side="center",
                clearance_left=cl_left,
                clearance_right=cl_right,
                source="local"
            )
            # If obstacle blocks the vehicle and scanning is enabled, trigger pan discovery
            if self.config.scan_on_block and self.is_enabled and not self._is_scanning_path:
                threading.Thread(target=self._execute_path_finding_scan, daemon=True).start()

            with self._lock: self._current_decision = dec
            return dec

        # Priority 4: Front Corridor Blocked by Visual Obstacles (Corridor Scanner)
        if center_blocked_by_cv:
            # Check if one side has significantly better path
            if cl_right > cl_left and cl_right > 0.45:
                dec = DecisionResult(
                    action=ActionEnum.RIGHT,
                    reason=f"Finding better path: Steering Right around obstacle (Right Clear: {int(cl_right*100)}%)",
                    hazard_detected=True,
                    obstacle_side="left",
                    clearance_left=cl_left,
                    clearance_right=cl_right,
                    suggested_steering=self.avoid_speed,
                    suggested_throttle=self.cruise_speed * 0.75,
                    source="local"
                )
            elif cl_left > cl_right and cl_left > 0.45:
                dec = DecisionResult(
                    action=ActionEnum.LEFT,
                    reason=f"Finding better path: Steering Left around obstacle (Left Clear: {int(cl_left*100)}%)",
                    hazard_detected=True,
                    obstacle_side="right",
                    clearance_left=cl_left,
                    clearance_right=cl_right,
                    suggested_steering=-self.avoid_speed,
                    suggested_throttle=self.cruise_speed * 0.75,
                    source="local"
                )
            else:
                # Path dead ahead is completely blocked on both near sides -> Stop and trigger environmental scan
                dec = DecisionResult(
                    action=ActionEnum.STOP,
                    reason="Path corridor obstructed. Searching for clear navigational path...",
                    hazard_detected=True,
                    obstacle_side="center",
                    clearance_left=cl_left,
                    clearance_right=cl_right,
                    source="local"
                )
                if self.config.scan_on_block and self.is_enabled and not self._is_scanning_path:
                    threading.Thread(target=self._execute_path_finding_scan, daemon=True).start()

            with self._lock: self._current_decision = dec
            return dec

        # Priority 5: Steering Avoidance around dynamic obstacles
        if avoid_hazard_side == "left":
            dec = DecisionResult(
                action=ActionEnum.RIGHT,
                reason="Obstacle detected on Left corridor side -> Steering Right to avoid",
                hazard_detected=True,
                obstacle_side="left",
                clearance_left=cl_left,
                clearance_right=cl_right,
                suggested_steering=self.avoid_speed,
                suggested_throttle=self.cruise_speed * 0.8,
                source="local"
            )
            with self._lock: self._current_decision = dec
            return dec
        elif avoid_hazard_side == "right":
            dec = DecisionResult(
                action=ActionEnum.LEFT,
                reason="Obstacle detected on Right corridor side -> Steering Left to avoid",
                hazard_detected=True,
                obstacle_side="right",
                clearance_left=cl_left,
                clearance_right=cl_right,
                suggested_steering=-self.avoid_speed,
                suggested_throttle=self.cruise_speed * 0.8,
                source="local"
            )
            with self._lock: self._current_decision = dec
            return dec

        # Priority 6: Path Clear -> Smooth Cruising Forward
        dec = DecisionResult(
            action=ActionEnum.FORWARD,
            reason="Driving corridor clear. Cruising forward.",
            hazard_detected=False,
            clearance_left=cl_left,
            clearance_right=cl_right,
            suggested_steering=0.0,
            suggested_throttle=self.cruise_speed,
            source="local"
        )
        with self._lock: self._current_decision = dec
        return dec

    # -------------------------------------------------------------------------
    # Environmental Path-Finding Scan (Automated Camera Pan & Clearance Search)
    # -------------------------------------------------------------------------
    def _execute_path_finding_scan(self) -> None:
        """
        Executes automated camera servo sweep to find an alternate clear path:
        1. Halts motors safely.
        2. Pans Camera Servo Left (145°) and measures open clearance.
        3. Pans Camera Servo Right (35°) and measures open clearance.
        4. Recenters Camera Servo (90°).
        5. Steers vehicle towards the clearer trajectory!
        """
        if self._is_scanning_path:
            return

        self._is_scanning_path = True
        logger.info("[PATH FINDING] Initiating environmental clearance scan...")

        # Halt car during scan
        self.motors.stop()
        with self._lock:
            self._current_decision = DecisionResult(
                action=ActionEnum.SCAN,
                reason="Scanning environment to find alternate clear path...",
                hazard_detected=True
            )

        try:
            # 1. Pan Left
            self.servo.set_angle(145.0)
            time.sleep(0.4)
            left_frame = self.camera.get_bgr_frame()
            left_clearance = self.corridor_scanner.measure_pan_clearance(left_frame)

            # 2. Pan Right
            self.servo.set_angle(35.0)
            time.sleep(0.4)
            right_frame = self.camera.get_bgr_frame()
            right_clearance = self.corridor_scanner.measure_pan_clearance(right_frame)

            # 3. Recenter
            self.servo.set_angle(90.0)
            time.sleep(0.2)

            logger.info(f"[PATH SCAN] Left Clearance: {left_clearance:.2f} | Right Clearance: {right_clearance:.2f}")

            # 4. Steer towards clearest trajectory
            if self.is_enabled:
                if left_clearance > right_clearance and left_clearance > 0.35:
                    logger.info("[PATH SCAN] Chosen path: LEFT. Steering vehicle to clear hazard...")
                    self.motors.drive(throttle=self.cruise_speed * 0.7, steering=-self.avoid_speed)
                    time.sleep(0.6)
                elif right_clearance > left_clearance and right_clearance > 0.35:
                    logger.info("[PATH SCAN] Chosen path: RIGHT. Steering vehicle to clear hazard...")
                    self.motors.drive(throttle=self.cruise_speed * 0.7, steering=self.avoid_speed)
                    time.sleep(0.6)
                else:
                    logger.warning("[PATH SCAN] All directions blocked. Waiting safely...")
                    self.motors.stop()

        except Exception as e:
            logger.error(f"[PATH SCAN] Error during servo sweep: {e}")
        finally:
            self.servo.set_angle(90.0)
            self._is_scanning_path = False

    # -------------------------------------------------------------------------
    # Motor Action Execution
    # -------------------------------------------------------------------------
    def _execute_motor_action(self, decision: DecisionResult) -> None:
        action = decision.action

        if action in (ActionEnum.STOP, ActionEnum.WAIT, ActionEnum.SCAN, ActionEnum.UNKNOWN):
            self.motors.stop()
        elif action == ActionEnum.FORWARD:
            self.motors.drive(throttle=decision.suggested_throttle, steering=decision.suggested_steering)
        elif action == ActionEnum.LEFT:
            self.motors.drive(throttle=decision.suggested_throttle, steering=decision.suggested_steering)
        elif action == ActionEnum.RIGHT:
            self.motors.drive(throttle=decision.suggested_throttle, steering=decision.suggested_steering)
        elif action == ActionEnum.SLOW:
            self.motors.drive(throttle=self.cruise_speed * 0.45, steering=0.0)

    # -------------------------------------------------------------------------
    # HUD Frame Generation
    # -------------------------------------------------------------------------
    def annotate_frame(self, bgr_frame: np.ndarray) -> np.ndarray:
        """Applies Visualizer HUD overlays onto frame."""
        if not self.show_overlay or bgr_frame is None:
            return bgr_frame

        vis_frame = bgr_frame.copy()
        with self._lock:
            dec = self._current_decision
            dets = list(self._current_detections)
            risks = dict(self._current_risks)
            fps = self._fps
            latency = self._latency_ms
            mode_str = self.mode
            auto_en = self.is_enabled

        return Visualizer.draw(
            frame=vis_frame,
            detections=dets,
            risks=risks,
            decision=dec,
            fps=fps,
            latency_ms=latency,
            lane_width_ratio=self.config.lane_width_ratio,
            mode_str=mode_str,
            auto_enabled=auto_en
        )

    # -------------------------------------------------------------------------
    # Status API Dict
    # -------------------------------------------------------------------------
    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            dec = self._current_decision
            server_stat = self.server_client.get_status()
            dets_summary = [
                {
                    "class": d.class_name,
                    "confidence": round(d.confidence, 2),
                    "track_id": d.track_id,
                    "box": [int(d.bbox.x1), int(d.bbox.y1), int(d.bbox.x2), int(d.bbox.y2)],
                    "risk": round(self._current_risks.get(d.track_id or id(d), {}).get("risk_score", 0.0), 2),
                    "ttc": round(self._current_risks.get(d.track_id or id(d), {}).get("ttc", float('inf')), 1)
                }
                for d in self._current_detections
            ]

            return {
                "autonomous_enabled": self.is_enabled,
                "mode": self.mode,
                "action": dec.action.value,
                "reason": dec.reason,
                "hazard_detected": dec.hazard_detected,
                "clearance_left": getattr(dec, "clearance_left", 1.0),
                "clearance_right": getattr(dec, "clearance_right", 1.0),
                "fps": round(self._fps, 1),
                "latency_ms": round(self._latency_ms, 1),
                "cruise_speed": self.cruise_speed,
                "avoid_speed": self.avoid_speed,
                "show_overlay": self.show_overlay,
                "is_scanning": self._is_scanning_path,
                "detections_count": len(self._current_detections),
                "detections": dets_summary,
                "server_ai": server_stat
            }
