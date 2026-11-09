import os
import sys
import time
import math
import logging
import threading
from typing import Optional, Dict, Any, List

import numpy as np
import cv2

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

try:
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
    from autonomous_car.ai.paid_decision_client import PaidDecisionClient
    from autonomous_car.ai.visualization import Visualizer
    from autonomous_car.sensors.ultrasonic import UltrasonicSensors, SingleSensorData
    from autonomous_car.navigation.waypoint import Waypoint, WaypointNavigator, pixel_to_world, world_to_pixel
    from autonomous_car.navigation.marker_route import MarkerRouteNavigator
    from autonomous_car.control.safety_supervisor import SafetySupervisor, MotionCommand
except ImportError:
    from config import AppConfig, AutonomousConfig
    from hardware.motors import MotorController
    from hardware.servo import ServoController
    from vision.camera import Camera
    from ai.classes import ActionEnum, DecisionResult, DetectionResult, BoundingBox
    from ai.detector import YOLODetector
    from ai.tracker import ObjectTracker
    from ai.risk_analyzer import RiskAnalyzer
    from ai.corridor_scanner import CorridorScanner
    from ai.server_client import ServerAIClient
    from ai.paid_decision_client import PaidDecisionClient
    from ai.visualization import Visualizer
    from sensors.ultrasonic import UltrasonicSensors, SingleSensorData
    from navigation.waypoint import Waypoint, WaypointNavigator, pixel_to_world, world_to_pixel
    from navigation.marker_route import MarkerRouteNavigator
    from control.safety_supervisor import SafetySupervisor, MotionCommand

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
        ultrasonic_sensors: Optional[UltrasonicSensors] = None,
    ) -> None:
        self.config: AutonomousConfig = config
        self.motors: MotorController = motors
        self.servo: ServoController = servo
        self.camera: Camera = camera
        self.ultrasonic_sensors = ultrasonic_sensors
        self._front_distance_cm: Optional[float] = None
        self._rear_distance_cm: Optional[float] = None
        self._front_sensor_data: Optional[SingleSensorData] = None
        self._last_sensor_read = 0.0
        self._sensor_read_interval = 0.12
        self._sensor_lock = threading.Lock()
        self._sensor_stop = threading.Event()
        self._sensor_thread: Optional[threading.Thread] = None

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
        self.tracker = ObjectTracker(
            self.detector,
            min_inference_interval_sec=getattr(self.config, "yolo_inference_interval_sec", 0.35),
        )
        self.risk_analyzer = RiskAnalyzer()
        self.corridor_scanner = CorridorScanner(lane_width_ratio=self.config.lane_width_ratio)
        self.waypoint_nav = WaypointNavigator(AppConfig().waypoint)
        # Establish one shared world-frame convention at startup.  The
        # dashboard, camera projection, and SLAM mapper all use 90° as the
        # initial forward heading (+Y); leaving odometry at its dataclass
        # default would rotate every first map/camera target by 90°.
        self.waypoint_nav.odometry.reset(0.0, 0.0, 90.0)
        self.marker_navigator = MarkerRouteNavigator(
            marker_size_cm=getattr(self.config, "marker_size_cm", 8.0),
            dictionary_name=getattr(self.config, "marker_dictionary", "DICT_4X4_50"),
            camera_fov_h_deg=getattr(AppConfig().waypoint, "fov_h_deg", 62.2),
            arrival_distance_cm=getattr(self.config, "marker_arrival_distance_cm", 35.0),
            throttle=getattr(self.config, "marker_throttle", 0.30),
        )
        initial_marker_route = getattr(self.config, "marker_route", [])
        if initial_marker_route:
            self.marker_navigator.set_route(
                initial_marker_route,
                enabled=bool(getattr(self.config, "marker_navigation_enabled", False)),
            )
        try:
            from navigation.visual_navigator import VisualNavigator
            self.visual_navigator = VisualNavigator(camera_fov_h_deg=62.2)
        except Exception:
            self.visual_navigator = None

        try:
            from vision.lane_detector import LaneDetector, LaneResult
            self.lane_detector = LaneDetector()
        except Exception:
            try:
                from autonomous_car.vision.lane_detector import LaneDetector, LaneResult
                self.lane_detector = LaneDetector()
            except Exception:
                self.lane_detector = None

        try:
            from ai.traffic_rules import TrafficRuleEngine, TrafficRuleResult
            self.traffic_engine = TrafficRuleEngine()
        except Exception:
            try:
                from autonomous_car.ai.traffic_rules import TrafficRuleEngine, TrafficRuleResult
                self.traffic_engine = TrafficRuleEngine()
            except Exception:
                self.traffic_engine = None

        self.server_client = ServerAIClient(base_url=self.config.server_url)
        self.paid_ai = PaidDecisionClient(
            endpoint=getattr(self.config, "paid_ai_endpoint", ""),
            api_key=os.getenv(getattr(self.config, "paid_ai_api_key_env", "RCCAR_PAID_AI_API_KEY"), ""),
            model=getattr(self.config, "paid_ai_model", "gpt-4o-mini"),
            interval_sec=getattr(self.config, "paid_ai_interval_sec", 1.0),
            timeout_sec=getattr(self.config, "paid_ai_timeout_sec", 4.0),
        )
        if self.mode in ("paid_ai", "hybrid_paid"):
            self.paid_ai.start()
        if self.mode not in ("hybrid", "server_ai"):
            self.server_client.stop()

        # Dedicated Safety Supervisor enforcing override authority
        self.safety_supervisor = SafetySupervisor()

        # Visual Odometry & SLAM Navigation Bridge (ORB-SLAM Inspired)
        try:
            from slam.slam_navigation_bridge import SLAMNavigationBridge
            self.slam_bridge: Optional[SLAMNavigationBridge] = SLAMNavigationBridge()
        except Exception:
            try:
                from autonomous_car.slam.slam_navigation_bridge import SLAMNavigationBridge
                self.slam_bridge = SLAMNavigationBridge()
            except Exception as e:
                logger.debug(f"[AUTO DRIVER] SLAM bridge init fallback: {e}")
                self.slam_bridge = None

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
        self._is_blind: bool = False
        self._blind_reason: str = ""
        self._frame_mean_brightness: float = 0.0
        self._frame_std_contrast: float = 0.0
        self._last_obstacle_pose: Optional[Dict[str, Any]] = None
        self._active_3d_target: Optional[Dict[str, Any]] = None
        self._current_reprojected_pin: Optional[Dict[str, Any]] = None
        self._destination_reached: bool = False
        self.active_gaze_tracking: bool = True
        self._mapped_3d_obstacles: List[Dict[str, Any]] = []
        self.gps_origin: Dict[str, Any] = {
            "latitude": getattr(self.config, "gps_origin_lat", 9.5916),
            "longitude": getattr(self.config, "gps_origin_lon", 76.5222),
            "initialized": False,
        }

        # Ground Station Loss-of-Link Failsafe State
        self._ground_station_last_seen: float = time.time()
        self._ground_station_link_active: bool = True
        self._loss_of_link_failsafe_active: bool = False
        self._ground_station_timeout_sec: float = getattr(self.config, "ground_station_timeout_sec", 2.0)

        # Lane Assist & Traffic Rules State
        self._current_lane_result: Optional[Any] = None
        self._current_traffic_result: Optional[Any] = None
        self.lane_assist_enabled: bool = getattr(self.config, "enable_lane_assist", True)
        self.traffic_rules_enabled: bool = getattr(self.config, "enable_traffic_rules", True)

        self._lock = threading.RLock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        # Incremented whenever motion ownership changes.  Path-scan workers
        # capture this value and cancel themselves if manual control or a new
        # AI session supersedes them.
        self._control_generation = 0

        # The offline profile must not create a network dependency. Start the
        # optional server client only for explicitly selected server modes.
        if self.mode in ("hybrid", "server_ai"):
            self.server_client.start()

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------
    def start(self) -> None:
        """Starts background perception and autonomous driving loop."""
        if self._running:
            return
        self._running = True
        self._sensor_stop.clear()
        if self.ultrasonic_sensors is not None:
            self._sensor_thread = threading.Thread(
                target=self._ultrasonic_loop,
                daemon=True,
                name="ultrasonic-sensor-loop",
            )
            self._sensor_thread.start()
        self._thread = threading.Thread(target=self._driving_loop, daemon=True, name="auto-driver-loop")
        self._thread.start()
        logger.info("[AUTO DRIVER] Background autonomous perception loop started.")

    def stop(self) -> None:
        with self._lock:
            self._running = False
            self.is_enabled = False
            self._control_generation += 1
        self._sensor_stop.set()
        self.motors.stop()
        self.server_client.stop()
        sensor_thread = self._sensor_thread
        if sensor_thread is not None and sensor_thread is not threading.current_thread():
            sensor_thread.join(timeout=1.0)
        self._sensor_thread = None

    def _ultrasonic_loop(self) -> None:
        """Continuously sample HC-SR04 without waiting for camera inference."""
        while self._running and not self._sensor_stop.is_set():
            self._refresh_ultrasonic(force=True)
            self._sensor_stop.wait(self._sensor_read_interval)

    def _refresh_ultrasonic(self, force: bool = False) -> None:
        """Refresh the obstacle sensor independently of camera perception.

        Ultrasonic freshness is a hard safety input.  It must continue to be
        updated while the vision pipeline is paused, while a servo path scan
        is running, and while the detector is busy.  Previously sampling was
        embedded in the perception branch, so path scans could make valid
        sonar data appear stale and cause every safety-checked motor command
        to stop.
        """
        if self.ultrasonic_sensors is None:
            return

        with self._sensor_lock:
            now_mono = time.monotonic()
            if not force and now_mono - self._last_sensor_read < self._sensor_read_interval:
                return

            try:
                sensor_data = self.ultrasonic_sensors.read_all()
                # Timestamp on completion so a slow pulse read is not
                # reported as stale before the first autonomous command.
                sample_time = time.time()
                sensor_data.front.timestamp = sample_time
                sensor_data.rear.timestamp = sample_time
                self._front_sensor_data = sensor_data.front
                self._front_distance_cm = sensor_data.front.distance_cm
                self._rear_distance_cm = sensor_data.rear.distance_cm
            except Exception as sensor_error:
                # A read failure is a sensor fault, never a clear path. Keep
                # an invalid timestamped sample so forward motion is rejected
                # until a valid reading arrives.
                now = time.time()
                self._front_sensor_data = SingleSensorData(
                    distance_cm=None,
                    obstacle_warning=False,
                    emergency_stop=True,
                    is_valid=False,
                    timestamp=now,
                    status="FAULT",
                )
                self._front_distance_cm = None
                logger.warning(f"[AUTO DRIVER] Ultrasonic refresh failed: {sensor_error}")
            finally:
                self._last_sensor_read = time.monotonic()

    def set_enabled(self, enabled: bool) -> bool:
        with self._lock:
            requested = bool(enabled)
            changed = self.is_enabled != requested
            if changed:
                self._control_generation += 1
            self.is_enabled = requested
            if not self.is_enabled:
                self.motors.stop()
                self._current_decision = DecisionResult(
                    action=ActionEnum.STOP,
                    reason="Autonomous Mode Disengaged (Manual Control Active)"
                )
            else:
                self._destination_reached = False
                self._current_decision = DecisionResult(
                    action=ActionEnum.FORWARD,
                    reason="Autonomous Mode Engaged"
                )
        if changed and requested and self.ultrasonic_sensors is not None:
            # Prime the hard safety input before the first autonomous command.
            self._refresh_ultrasonic(force=True)
        if changed:
            logger.info(f"[AUTO DRIVER] Autonomous Drive: {'ENABLED' if self.is_enabled else 'DISABLED'}")
        return self.is_enabled

    @property
    def is_blind(self) -> bool:
        with self._lock:
            return self._is_blind

    def set_mode(self, mode: str) -> str:
        mode_clean = mode.lower().strip()
        if mode_clean in ["hybrid", "local_ai", "server_ai", "paid_ai", "hybrid_paid"]:
            with self._lock:
                self.mode = mode_clean
            if mode_clean in ("hybrid", "server_ai"):
                self.server_client.start()
            else:
                self.server_client.stop()
            if mode_clean in ("paid_ai", "hybrid_paid"):
                self.paid_ai.start()
            else:
                self.paid_ai.stop()
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
            if "lane_assist" in params:
                self.lane_assist_enabled = bool(params["lane_assist"])
            if "enable_lane_assist" in params:
                self.lane_assist_enabled = bool(params["enable_lane_assist"])
            if "traffic_rules" in params:
                self.traffic_rules_enabled = bool(params["traffic_rules"])
            if "enable_traffic_rules" in params:
                self.traffic_rules_enabled = bool(params["enable_traffic_rules"])

    def configure_marker_route(self, marker_ids: List[int], enabled: bool = True) -> Dict[str, Any]:
        """Configure an explicit offline region route such as [10, 20, 30]."""
        route = self.marker_navigator.set_route(marker_ids, enabled=enabled)
        self._destination_reached = False
        self.clear_target_pin()
        if route and enabled:
            # A pan camera is not a stable visual odometry sensor while the
            # vehicle is moving. Center it before starting marker following.
            self._set_servo_angle(90.0, auto_relax=False)
            self.set_enabled(True)
        else:
            self.set_enabled(False)
        return self.marker_navigator.status(reason="Marker route configured").as_dict()

    def clear_marker_route(self) -> Dict[str, Any]:
        self.marker_navigator.clear_route()
        self.motors.stop()
        self.set_enabled(False)
        return self.marker_navigator.status(reason="Marker route cleared").as_dict()

    def get_marker_status(self) -> Dict[str, Any]:
        return self.marker_navigator.status(reason="Marker route status").as_dict()

    def toggle_lane_assist(self, enabled: Optional[bool] = None) -> bool:
        """Enables, disables, or toggles active Lane Keeping Assist and boundary departure protection."""
        with self._lock:
            self.lane_assist_enabled = not self.lane_assist_enabled if enabled is None else bool(enabled)
            logger.info(f"[AUTO DRIVER] Lane Assist set to: {'ENABLED' if self.lane_assist_enabled else 'DISABLED'}")
            return self.lane_assist_enabled

    def toggle_traffic_rules(self, enabled: Optional[bool] = None) -> bool:
        """Enables, disables, or toggles traffic light and stop sign obedience."""
        with self._lock:
            self.traffic_rules_enabled = not self.traffic_rules_enabled if enabled is None else bool(enabled)
            if not self.traffic_rules_enabled and self.traffic_engine:
                self.traffic_engine.reset()
            logger.info(f"[AUTO DRIVER] Traffic Rules set to: {'ENABLED' if self.traffic_rules_enabled else 'DISABLED'}")
            return self.traffic_rules_enabled

    def add_destination(self, x: float, y: float, label: str = "") -> Dict[str, Any]:
        self._destination_reached = False
        waypoint = self.waypoint_nav.add_waypoint(float(x), float(y), label)
        return self._waypoint_dict(waypoint)

    def clear_destinations(self) -> None:
        self._destination_reached = False
        self.motors.stop()
        self.waypoint_nav.clear_waypoints()

    def reset_pose(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 90.0) -> None:
        self.waypoint_nav.odometry.reset(float(x), float(y), float(heading_deg))

    @staticmethod
    def _waypoint_dict(waypoint: Waypoint) -> Dict[str, Any]:
        return {"id": waypoint.id, "x": waypoint.x, "y": waypoint.y,
                "label": waypoint.label, "reached": waypoint.reached}

    def get_destinations(self) -> Dict[str, Any]:
        active = self.waypoint_nav.get_active_target()
        return {
            "pose": self.waypoint_nav.odometry.pose.__dict__.copy(),
            "active": self._waypoint_dict(active) if active else None,
            "waypoints": [self._waypoint_dict(item) for item in self.waypoint_nav.waypoints],
        }

    # -------------------------------------------------------------------------
    # Ground Station Loss-of-Link Failsafe Management
    # -------------------------------------------------------------------------
    def record_ground_station_heartbeat(self) -> None:
        """Call when receiving telemetry request, ping, or command from central ground station."""
        with self._lock:
            self._ground_station_last_seen = time.time()
            self._ground_station_link_active = True
            if self._loss_of_link_failsafe_active:
                logger.info("[GROUND STATION FAILSAFE] Central ground station link restored via telemetry heartbeat.")
                self._loss_of_link_failsafe_active = False

    def is_ground_station_connected(self) -> bool:
        """Checks if central ground station or edge AI server is actively connected."""
        with self._lock:
            gs_age = time.time() - self._ground_station_last_seen
            return self.server_client.is_connected or (gs_age < self._ground_station_timeout_sec)

    def is_loss_of_link_failsafe_active(self) -> bool:
        """True if the autonomous car is currently executing an onboard fallback mission due to ground station loss."""
        with self._lock:
            return self._loss_of_link_failsafe_active

    def get_ground_station_status(self) -> Dict[str, Any]:
        """Telemetry dictionary reporting ground station connectivity and failsafe status."""
        with self._lock:
            gs_age = time.time() - self._ground_station_last_seen
            connected = self.server_client.is_connected or (gs_age < self._ground_station_timeout_sec)
            active_target = self.waypoint_nav.get_active_target()
            return {
                "connected": connected,
                "loss_of_link_active": self._loss_of_link_failsafe_active,
                "failsafe_mode": "ONBOARD_TASK_FINISH" if self._loss_of_link_failsafe_active else "NORMAL",
                "seconds_since_last_contact": round(gs_age, 2),
                "task_active": active_target is not None,
                "active_target": {"x": active_target.x, "y": active_target.y, "label": active_target.label} if active_target else None,
                "task_finished": self._destination_reached,
            }

    # -------------------------------------------------------------------------
    # Main Autonomous Perception & Driving Loop
    # -------------------------------------------------------------------------
    def _driving_loop(self) -> None:
        fps_smoothing = 0.9
        last_t = time.time()

        while self._running:
            try:
                self.safety_supervisor.heartbeat()
                loop_start = time.time()

                # Keep the hard obstacle-safety input alive regardless of
                # whether perception is in standby, scanning, or inference.
                self._refresh_ultrasonic()

                # Manual standby must not consume the Pi with autonomous
                # perception. It also makes the ownership boundary explicit:
                # no autonomous command is produced until the car is armed or
                # a destination request enables autonomy.
                if not self.is_enabled and self.waypoint_nav.get_active_target() is None:
                    self.motors.stop()
                    with self._lock:
                        self._current_decision = DecisionResult(
                            action=ActionEnum.STOP,
                            reason="Manual standby: autonomous perception paused",
                            hazard_detected=False,
                            source="manual_standby",
                        )
                    time.sleep(0.10)
                    continue

                # 1. Fetch raw frame from camera
                bgr_frame = self.camera.get_bgr_frame()
                if bgr_frame is None or not self.camera.has_fresh_frame(max_age_seconds=1.0):
                    # Never continue driving on a stale frame after a camera
                    # timeout or disconnected CSI link.
                    if self.is_enabled:
                        self.motors.stop()
                    with self._lock:
                        self._current_decision = DecisionResult(
                            action=ActionEnum.STOP,
                            reason="Camera frame stale or unavailable",
                            hazard_detected=True,
                        )
                    time.sleep(0.04)
                    continue

                h, w = bgr_frame.shape[:2]

                # 2. Perception & Decision computation
                decision = self._evaluate_environment_and_decide(bgr_frame, w, h)

                # 3. Apply Decision to Motors if Autonomous mode is active
                now = time.time()
                dt = now - last_t
                last_t = now

                if self.is_enabled and not self._is_scanning_path:
                    self._execute_motor_action(decision)
                    self.waypoint_nav.odometry.update_motion(
                        throttle=decision.suggested_throttle,
                        steering=decision.suggested_steering,
                        dt=dt
                    )

                # Visual Odometry & SLAM Bridge Update (ORB-SLAM Pipeline)
                if self.slam_bridge is not None:
                    try:
                        cur_odom = self.waypoint_nav.odometry.pose
                        speed_cmd = (decision.suggested_throttle * 30.0) if (self.is_enabled and not self._is_scanning_path) else 0.0
                        self.slam_bridge.update(
                            bgr_frame=bgr_frame,
                            kinematic_x_cm=cur_odom.x,
                            kinematic_y_cm=cur_odom.y,
                            kinematic_heading_deg=cur_odom.heading_deg,
                            kinematic_speed_cm_s=speed_cmd,
                            dt=dt,
                            sonar_depth_cm=self._front_distance_cm,
                        )
                    except Exception as e:
                        logger.debug(f"[AUTO DRIVER] SLAM update error: {e}")

                # 4. Measure metrics
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
    def _evaluate_marker_route(self, frame: np.ndarray) -> Optional[DecisionResult]:
        """Return a safe marker-following command when a route is active."""
        if not self.marker_navigator.enabled:
            return None

        # Keep the camera fixed to the vehicle forward direction while moving.
        # Panning a monocular camera creates apparent scene motion and invalid
        # distance/pose estimates.
        current_servo = float(getattr(self.servo, "current_angle", 90.0))
        if abs(current_servo - 90.0) >= 2.0 and not self._is_scanning_path:
            self._set_servo_angle(90.0, auto_relax=False)
            current_servo = 90.0

        status = self.marker_navigator.update(frame, servo_angle_deg=current_servo)
        if status.route_complete:
            self.motors.stop()
            self.is_enabled = False
            self._destination_reached = True
            return DecisionResult(
                action=ActionEnum.STOP,
                reason="Marker route complete. Vehicle stopped safely.",
                suggested_steering=0.0,
                suggested_throttle=0.0,
                source="marker_route",
            )

        if status.arrived:
            return DecisionResult(
                action=ActionEnum.STOP,
                reason=status.reason,
                suggested_steering=0.0,
                suggested_throttle=0.0,
                source="marker_route",
            )

        if not status.visible:
            self.motors.stop()
            return DecisionResult(
                action=ActionEnum.STOP,
                reason=status.reason,
                hazard_detected=True,
                suggested_steering=0.0,
                suggested_throttle=0.0,
                source="marker_route",
            )

        steering = float(status.steering)
        action = ActionEnum.FORWARD
        if steering > 0.08:
            action = ActionEnum.RIGHT
        elif steering < -0.08:
            action = ActionEnum.LEFT

        distance = status.distance_cm or 999.0
        throttle = self.marker_navigator.throttle
        if distance < 60.0:
            throttle *= 0.75

        return DecisionResult(
            action=action,
            reason=(f"{status.reason}: {distance:.1f}cm, "
                    f"bearing {status.bearing_deg:+.1f}°"),
            suggested_steering=round(steering, 3),
            suggested_throttle=round(throttle, 3),
            source="marker_route",
        )

    def _evaluate_environment_and_decide(self, frame: np.ndarray, w: int, h: int) -> DecisionResult:
        # If currently executing an active camera path-finding scan, hold current scan decision
        if self._is_scanning_path:
            with self._lock:
                return self._current_decision

        # Destination arrival latch: If destination was already reached and no new target was assigned, hold full stop
        if self._destination_reached and self.waypoint_nav.get_active_target() is None:
            self.motors.stop()
            dec = DecisionResult(
                action=ActionEnum.STOP,
                reason="🎯 Target location reached. Vehicle stopped (holding position).",
                hazard_detected=False,
                suggested_steering=0.0,
                suggested_throttle=0.0,
                source="destination"
            )
            with self._lock:
                self._current_decision = dec
            return dec

        # Continuously update 3D target reprojection on every perception frame
        self.get_current_reprojected_pin()

        front_distance = self._front_distance_cm

        # =====================================================================
        # 1. HARDWARE PROXIMITY FAILSAFE: ULTRASONIC DISTANCE SENSOR
        # Proximity sensor takes immediate priority over vision.
        # =====================================================================
        if self.ultrasonic_sensors is not None:
            front_distance = self._front_distance_cm
            if front_distance is not None:
                sonar_avoid_thresh = getattr(self.config, 'sonar_avoid_distance_cm', 8.0)
                rev_thresh = getattr(self.ultrasonic_sensors.config, 'collision_reverse_distance_cm', 10.0)
                emerg_thresh = getattr(self.ultrasonic_sensors.config, 'emergency_stop_distance_cm', 16.0)
                warn_thresh = getattr(self.ultrasonic_sensors.config, 'warning_distance_cm', 30.0)

                # TIER 1: CRITICAL SONAR CLOSE-RANGE AVOIDANCE (< 8.0 cm / rev_thresh)
                # If sonar shows less than 8 cm, avoid that blocked path and redirect to a better path
                if front_distance < sonar_avoid_thresh or front_distance <= rev_thresh:
                    corridor_info = self.corridor_scanner.analyze(frame)
                    cl_left = corridor_info.get("left_clearance", 0.0)
                    cl_right = corridor_info.get("right_clearance", 0.0)

                    # Determine which side provides a better open path
                    if cl_right > cl_left and cl_right >= 0.15:
                        better_side = "RIGHT"
                        # Arc reverse: steer while reversing so vehicle nose redirects toward open right corridor
                        rev_steering = 0.50
                    elif cl_left > cl_right and cl_left >= 0.15:
                        better_side = "LEFT"
                        # Arc reverse: steer while reversing so vehicle nose redirects toward open left corridor
                        rev_steering = -0.50
                    else:
                        better_side = "RIGHT" if cl_right >= cl_left else "LEFT"
                        rev_steering = 0.45 if cl_right >= cl_left else -0.45

                    logger.warning(
                        f"[SONAR AVOIDANCE] Sonar showing {front_distance:.1f} cm (< {sonar_avoid_thresh:.0f} cm)! "
                        f"Avoiding blocked path -> reversing and redirecting toward better path ({better_side})."
                    )

                    decision = DecisionResult(
                        action=ActionEnum.REVERSE,
                        reason=f"Sonar {front_distance:.1f}cm < {sonar_avoid_thresh:.0f}cm: Avoiding blocked path. Reversing to take better path ({better_side}).",
                        hazard_detected=True,
                        obstacle_side="center",
                        clearance_left=cl_left,
                        clearance_right=cl_right,
                        suggested_steering=rev_steering,
                        suggested_throttle=-0.38,
                        source="ultrasonic",
                    )
                    with self._lock:
                        self._current_decision = decision

                    if self.config.scan_on_block and not self._is_scanning_path:
                        threading.Thread(
                            target=self._execute_path_finding_scan,
                            kwargs={"trigger_reason": "collision_reverse"},
                            daemon=True
                        ).start()

                    return decision

                # TIER 2: OBSTACLE IN FRONT (rev_thresh < dist <= emerg_thresh) -> EMERGENCY STOP & PATH FIND
                if front_distance <= emerg_thresh:
                    self.motors.stop()
                    decision = DecisionResult(
                        action=ActionEnum.STOP,
                        reason=f"Emergency stop: Obstacle at {front_distance:.1f} cm",
                        hazard_detected=True,
                        obstacle_side="center",
                        clearance_left=0.0,
                        clearance_right=0.0,
                        suggested_steering=0.0,
                        suggested_throttle=0.0,
                        source="ultrasonic",
                    )
                    with self._lock: self._current_decision = decision

                    # Trigger environmental scan to avoid blocked path and select better path
                    if self.config.scan_on_block and (self.is_enabled or self.waypoint_nav.get_active_target() is not None) and not self._is_scanning_path:
                        threading.Thread(
                            target=self._execute_path_finding_scan,
                            kwargs={"trigger_reason": "front_obstacle"},
                            daemon=True
                        ).start()

                    return decision

                # TIER 3: PROACTIVE APPROACH ZONE (15 cm < dist <= 30 cm) -> NO UNNECESSARY SCANNING
                # Gently steer around obstacle using camera corridor clearance
                if front_distance <= warn_thresh:
                    corridor_info = self.corridor_scanner.analyze(frame)
                    cl_left = corridor_info.get("left_clearance", 0.0)
                    cl_right = corridor_info.get("right_clearance", 0.0)

                    if cl_right >= cl_left and cl_right >= 0.20:
                        decision = DecisionResult(
                            action=ActionEnum.RIGHT,
                            reason=f"Obstacle at {front_distance:.1f} cm: Steering Right continuously (Right clear: {int(cl_right*100)}%).",
                            hazard_detected=True,
                            obstacle_side="left",
                            clearance_left=cl_left,
                            clearance_right=cl_right,
                            suggested_steering=self.avoid_speed,
                            suggested_throttle=round(self.cruise_speed * 0.85, 2),
                            source="ultrasonic_avoid",
                        )
                    elif cl_left > cl_right and cl_left >= 0.20:
                        decision = DecisionResult(
                            action=ActionEnum.LEFT,
                            reason=f"Obstacle at {front_distance:.1f} cm: Steering Left continuously (Left clear: {int(cl_left*100)}%).",
                            hazard_detected=True,
                            obstacle_side="right",
                            clearance_left=cl_left,
                            clearance_right=cl_right,
                            suggested_steering=-self.avoid_speed,
                            suggested_throttle=round(self.cruise_speed * 0.85, 2),
                            source="ultrasonic_avoid",
                        )
                    else:
                        self.motors.stop()
                        decision = DecisionResult(
                            action=ActionEnum.STOP,
                            reason=f"Front obstacle at {front_distance:.1f} cm with no open side corridor. Pausing & scanning for path...",
                            hazard_detected=True,
                            obstacle_side="center",
                            clearance_left=cl_left,
                            clearance_right=cl_right,
                            suggested_steering=0.0,
                            suggested_throttle=0.0,
                            source="ultrasonic",
                        )
                        if self.config.scan_on_block and (self.is_enabled or self.waypoint_nav.get_active_target() is not None) and not self._is_scanning_path:
                            threading.Thread(target=self._execute_path_finding_scan, kwargs={"trigger_reason": "front_obstacle"}, daemon=True).start()

                    with self._lock:
                        self._current_decision = decision
                    return decision

        # Marker navigation is deliberately evaluated after the ultrasonic
        # safety tiers and before the expensive YOLO pipeline.  It is an
        # explicit opt-in route mode, so normal local-AI behavior is unchanged.
        marker_decision = self._evaluate_marker_route(frame)
        if marker_decision is not None:
            with self._lock:
                self._current_decision = marker_decision
            return marker_decision

        # Ground Station & Server AI Connection Health Monitoring
        now_ts = time.time()
        gs_age = now_ts - self._ground_station_last_seen
        server_connected = self.server_client.is_connected
        gs_online = server_connected or (gs_age < self._ground_station_timeout_sec)

        # Check Server AI if explicitly in SERVER_AI mode
        if self.mode == "server_ai":
            server_dec = None
            if server_connected:
                server_dec = self.server_client.get_latest_decision(max_age_sec=0.7)

            if server_dec is not None:
                with self._lock:
                    self._ground_station_link_active = True
                    if self._loss_of_link_failsafe_active:
                        logger.info("[GROUND STATION FAILSAFE] Central ground station link restored! Resuming server AI synchronization.")
                        self._loss_of_link_failsafe_active = False
                    self._current_decision = server_dec
                return server_dec
            else:
                # Connection to central ground station server lost or stale!
                if not self._loss_of_link_failsafe_active:
                    self._loss_of_link_failsafe_active = True
                    self._ground_station_link_active = False
                    logger.warning(
                        "[GROUND STATION DISCONNECT] Connection to central ground station lost! "
                        "Engaging onboard autonomous model to finish task and avoid collisions."
                    )
        elif self.mode == "hybrid":
            # Hybrid mode may use the ground station as an advisory source,
            # but local_ai is deliberately offline and must never report a
            # ground-station loss while it is operating normally.
            if not gs_online:
                if not self._loss_of_link_failsafe_active and (self.is_enabled or self.waypoint_nav.get_active_target() is not None):
                    self._loss_of_link_failsafe_active = True
                    self._ground_station_link_active = False
                    logger.warning(
                        "[GROUND STATION DISCONNECT] Ground station link lost during active navigation! "
                        "Onboard model continuing autonomously to finish task and avoid collisions."
                    )
            else:
                if self._loss_of_link_failsafe_active:
                    self._loss_of_link_failsafe_active = False
                    self._ground_station_link_active = True
                    logger.info("[GROUND STATION FAILSAFE] Ground station link restored.")

        # --- Local Perception Pipeline ---
        # A. YOLO Tracking & Detections
        detections = self.tracker.track(frame)

        # Multi-Modal Obstacle Augmentation: Detect huge objects & unclassified obstacles
        try:
            large_obs = self.corridor_scanner.detect_large_obstacles(frame)
            for lo in large_obs:
                bx1, by1, bx2, by2 = lo["bbox"]
                overlap = any(
                    (d.bbox.x1 < bx2 and d.bbox.x2 > bx1 and d.bbox.y1 < by2 and d.bbox.y2 > by1)
                    for d in detections
                )
                if not overlap:
                    detections.append(DetectionResult(
                        class_name=lo.get("class_name", "obstacle"),
                        confidence=lo["confidence"],
                        bbox=BoundingBox(x1=bx1, y1=by1, x2=bx2, y2=by2),
                        track_id=None
                    ))
        except Exception as e:
            logger.debug(f"[AUTO DRIVER] Large obstacle scan error: {e}")

        # Risks update
        risks = self.risk_analyzer.update(detections, time.time(), w, h)

        # =====================================================================
        # 2. VISION INTEGRITY CHECK: DARKNESS & BLANK / BLACK SENSOR DETECTION
        # If camera frame is completely black, dark, or flat blank -> SUDDEN STOP!
        # Automatically pan the camera, scan for a non-blind path, and maneuver towards destination!
        # =====================================================================
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
        mean_val, std_val = cv2.meanStdDev(gray)
        mean_brightness = float(mean_val[0][0])
        std_contrast = float(std_val[0][0])

        dark_thresh = getattr(self.config, 'darkness_mean_threshold', 14.0)
        blank_thresh = getattr(self.config, 'blank_std_threshold', 4.0)

        is_blind = False
        blind_reason = ""
        # Only classify as blind if no high-confidence object is identified
        if len(detections) == 0:
            if mean_brightness < dark_thresh:
                is_blind = True
                blind_reason = f"DARK SCENE: Frame is too dark (mean: {mean_brightness:.1f} < {dark_thresh:.1f})"
            elif std_contrast < blank_thresh:
                is_blind = True
                blind_reason = f"BLANK SENSOR: Frame is flat/uniform (contrast std: {std_contrast:.1f} < {blank_thresh:.1f})"

        with self._lock:
            self._frame_mean_brightness = mean_brightness
            self._frame_std_contrast = std_contrast
            self._is_blind = is_blind
            self._blind_reason = blind_reason

        if is_blind:
            self.motors.stop()  # Immediate hardware halt
            dec = DecisionResult(
                action=ActionEnum.STOP,
                reason=f"BLIND SPOT: {blind_reason}. Turning camera to scan for clear path to destination...",
                hazard_detected=True,
                obstacle_side="center",
                clearance_left=0.0,
                clearance_right=0.0,
                suggested_steering=0.0,
                suggested_throttle=0.0,
                source="vision_blind"
            )
            with self._lock:
                self._current_decision = dec

            # Automatically pan camera, find a better non-blind spot, and steer towards destination
            if self.config.scan_on_block and (self.is_enabled or self.waypoint_nav.get_active_target() is not None) and not self._is_scanning_path:
                threading.Thread(target=self._execute_path_finding_scan, kwargs={"trigger_reason": "blind_spot"}, daemon=True).start()

            return dec

        # =====================================================================
        # 3. TRAFFIC RULES COMPLIANCE ENGINE (RED LIGHT & STOP SIGN DWELL)
        # =====================================================================
        if getattr(self, "traffic_engine", None) is not None and getattr(self, "traffic_rules_enabled", True):
            try:
                tr_res = self.traffic_engine.evaluate(
                    bgr_frame=frame,
                    detections=detections,
                    front_distance_cm=front_distance,
                    now=time.time()
                )
                with self._lock:
                    self._current_traffic_result = tr_res

                if tr_res.requires_stop:
                    self.motors.stop()
                    dec = DecisionResult(
                        action=ActionEnum.STOP,
                        reason=tr_res.reason,
                        hazard_detected=True,
                        obstacle_side="center",
                        suggested_steering=0.0,
                        suggested_throttle=0.0,
                        source="traffic_rule"
                    )
                    with self._lock: self._current_decision = dec
                    return dec
            except Exception as e:
                logger.debug(f"[AUTO DRIVER] Traffic rules evaluation error: {e}")

        # =====================================================================
        # 4. LANE ASSIST (LKA) & HARD BOUNDARY ENFORCEMENT
        # "Don't cross the lane boundaries"
        # =====================================================================
        lane_res = None
        lane_boundary_decision = None
        if getattr(self, "lane_detector", None) is not None and getattr(self, "lane_assist_enabled", True):
            try:
                lane_res = self.lane_detector.detect_from_bgr(frame)
                with self._lock:
                    self._current_lane_result = lane_res

                # Record a boundary correction, but do not return it yet. A lane
                # heuristic must never outrank a confirmed object hazard or a
                # waypoint command. Those decisions are resolved below first.
                if lane_res.is_left_boundary_violated:
                    lane_boundary_decision = DecisionResult(
                        action=ActionEnum.RIGHT,
                        reason="⚠️ Lane Assist: Left boundary departure prevented! Steering Right to remain in lane.",
                        hazard_detected=True,
                        obstacle_side="left",
                        clearance_left=0.0,
                        clearance_right=1.0,
                        suggested_steering=lane_res.corrective_steering,
                        suggested_throttle=round(self.cruise_speed * 0.70, 2),
                        source="lane_assist"
                    )
                elif lane_res.is_right_boundary_violated:
                    lane_boundary_decision = DecisionResult(
                        action=ActionEnum.LEFT,
                        reason="⚠️ Lane Assist: Right boundary departure prevented! Steering Left to remain in lane.",
                        hazard_detected=True,
                        obstacle_side="right",
                        clearance_left=1.0,
                        clearance_right=0.0,
                        suggested_steering=lane_res.corrective_steering,
                        suggested_throttle=round(self.cruise_speed * 0.70, 2),
                        source="lane_assist"
                    )
            except Exception as e:
                logger.debug(f"[AUTO DRIVER] Lane detection error: {e}")

        # B. Real-time Corridor Obstacle Scanner
        corridor_info = self.corridor_scanner.analyze(frame)
        cl_left = corridor_info["left_clearance"]
        cl_right = corridor_info["right_clearance"]
        cl_center = corridor_info["center_clearance"]
        center_blocked_by_cv = corridor_info["center_blocked"]

        with self._lock:
            self._current_detections = detections
            self._current_risks = risks

        # Map detected objects to 3D world ground coordinates for spatial mapping
        cur_pose = self.waypoint_nav.odometry.pose
        current_servo = getattr(self.servo, 'current_angle', 90.0)
        mapped_obs = []
        for det in detections:
            base_u = int((det.bbox.x1 + det.bbox.x2) / 2.0)
            base_v = int(det.bbox.y2)
            ox, oy, _, _ = pixel_to_world(
                u=base_u, v=base_v,
                car_x=cur_pose.x, car_y=cur_pose.y, heading_deg=cur_pose.heading_deg,
                servo_angle_deg=current_servo,
            )
            mapped_obs.append({
                "class": det.class_name,
                "world_x": ox,
                "world_y": oy,
                "confidence": round(det.confidence, 2),
            })
        self._mapped_3d_obstacles = mapped_obs

        # =====================================================================
        # CORE DIRECTIVE: CORRIDOR OBJECT DETECTION & SAFETY STOP
        # Identify critical collision hazards overlapping the active driving corridor.
        # Peripheral background objects (chairs/potted plants outside corridor) do not halt car.
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
            risk_score = risk_data.get("risk_score", 0.1)
            ttc = risk_data.get("ttc", float('inf'))
            norm_area = risk_data.get("norm_area", bbox.area / float(w * h))

            # 1. Immediate Rule: Stop on Bus!
            if "bus" in cls_lower:
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
            cur_pose = self.waypoint_nav.odometry.pose
            self._last_obstacle_pose = {
                "x": cur_pose.x,
                "y": cur_pose.y,
                "heading_deg": cur_pose.heading_deg,
                "obstacle_class": critical_hazard.class_name,
                "timestamp": time.time(),
            }
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

        # Lane boundary correction is advisory. Apply it only when there is no
        # active destination; waypoint navigation owns steering while a task is
        # active, and it must be allowed to reach its destination deterministically.
        if lane_boundary_decision is not None and self.waypoint_nav.get_active_target() is None:
            with self._lock:
                self._current_decision = lane_boundary_decision
            return lane_boundary_decision

        # Update 3D world target reprojection onto camera view & Active Visual Servoing
        active_destination = self.waypoint_nav.get_active_target()
        cur_pose = self.waypoint_nav.odometry.pose
        current_servo = getattr(self.servo, 'current_angle', 90.0)

        if active_destination is not None:
            dx = active_destination.x - cur_pose.x
            dy = active_destination.y - cur_pose.y
            dist_p = math.hypot(dx, dy)
            target_heading = math.degrees(math.atan2(dy, dx)) % 360.0
            bearing_to_target = target_heading - cur_pose.heading_deg
            while bearing_to_target > 180.0:
                bearing_to_target -= 360.0
            while bearing_to_target < -180.0:
                bearing_to_target += 360.0

            # ACTIVE VISUAL SERVOING (Gaze Locking onto Fixed 3D World Anchor):
            # As the vehicle maneuvers or rotates, actively pan the servo to maintain
            # visual lock on the target point where it was first fixed in the 3D world.
            # Enforce strict 90° left (180°) and 90° right (0°) rotation from center (90°).
            if self.is_enabled and self.active_gaze_tracking and not self._is_scanning_path:
                desired_bearing = max(-90.0, min(90.0, bearing_to_target))
                clamped_servo = max(0.0, min(180.0, 90.0 + desired_bearing))
                # Deadband & exponential smoothing to eliminate micro-jitter ("glitters")
                if abs(clamped_servo - current_servo) >= 3.0:
                    smoothed_servo = current_servo + 0.35 * (clamped_servo - current_servo)
                    self._set_servo_angle(smoothed_servo, auto_relax=False)
                    current_servo = smoothed_servo

            self.get_current_reprojected_pin(servo_angle=current_servo)
        else:
            with self._lock:
                self._current_reprojected_pin = None

        if active_destination is not None:
            angle_error, distance_cm, reached = self.waypoint_nav.compute_steering_to_target()
            if reached or self.waypoint_nav.get_active_target() is None:
                has_next = (self.waypoint_nav.get_active_target() is not None)
                if not has_next:
                    # Final destination reached! Stop vehicle completely, disengage auto drive, and prevent roaming
                    self.motors.stop()
                    self.is_enabled = False
                    self._destination_reached = True
                    if not self._is_scanning_path:
                        self._set_servo_angle(90.0, auto_relax=False)
                    reason_msg = f"🎯 DESTINATION REACHED! Arrived safely at ({active_destination.x:.0f},{active_destination.y:.0f}). Stopped."
                    if self._loss_of_link_failsafe_active:
                        reason_msg = f"🎯 [TASK FINISHED ONBOARD] Destination ({active_destination.x:.0f},{active_destination.y:.0f}) reached safely during ground station disconnect! Stopped."
                    dec = DecisionResult(
                        action=ActionEnum.STOP,
                        reason=reason_msg,
                        hazard_detected=False,
                        suggested_steering=0.0,
                        suggested_throttle=0.0,
                        source="onboard_failsafe" if self._loss_of_link_failsafe_active else "destination",
                    )
                    with self._lock:
                        self._current_decision = dec
                        self._active_3d_target = None
                        self._current_reprojected_pin = None
                    logger.info(
                        f"[AUTO DRIVER] Destination ({active_destination.x:.1f}, {active_destination.y:.1f}) "
                        f"reached successfully. Stopped vehicle and disengaged autonomous drive."
                    )
                    return dec
                else:
                    next_wp = self.waypoint_nav.get_active_target()
                    logger.info(
                        f"[AUTO DRIVER] Waypoint reached. Advancing to next target: "
                        f"{next_wp.label} at ({next_wp.x:.1f}, {next_wp.y:.1f})"
                    )

            # If front corridor is blocked by visual obstruction, steer around it towards open side
            if center_blocked_by_cv:
                if cl_right > cl_left and cl_right > 0.40:
                    dec = DecisionResult(
                        action=ActionEnum.RIGHT,
                        reason=f"Bypassing obstacle: Steering Right towards Target ({active_destination.x:.0f},{active_destination.y:.0f}) | Dist: {distance_cm:.0f}cm",
                        hazard_detected=True,
                        obstacle_side="left",
                        clearance_left=cl_left,
                        clearance_right=cl_right,
                        suggested_steering=self.avoid_speed,
                        suggested_throttle=self.cruise_speed * 0.70,
                        source="destination_avoid"
                    )
                    with self._lock: self._current_decision = dec
                    return dec
                elif cl_left > cl_right and cl_left > 0.40:
                    dec = DecisionResult(
                        action=ActionEnum.LEFT,
                        reason=f"Bypassing obstacle: Steering Left towards Target ({active_destination.x:.0f},{active_destination.y:.0f}) | Dist: {distance_cm:.0f}cm",
                        hazard_detected=True,
                        obstacle_side="right",
                        clearance_left=cl_left,
                        clearance_right=cl_right,
                        suggested_steering=-self.avoid_speed,
                        suggested_throttle=self.cruise_speed * 0.70,
                        source="destination_avoid"
                    )
                    with self._lock: self._current_decision = dec
                    return dec
                else:
                    # Corridor is completely obstructed ahead
                    dec = DecisionResult(
                        action=ActionEnum.STOP,
                        reason=f"Corridor obstructed en route to Target ({active_destination.x:.0f},{active_destination.y:.0f}). Scanning for clear path...",
                        hazard_detected=True,
                        obstacle_side="center",
                        clearance_left=cl_left,
                        clearance_right=cl_right,
                        suggested_steering=0.0,
                        suggested_throttle=0.0,
                        source="destination_blocked"
                    )
                    with self._lock: self._current_decision = dec
                    if self.config.scan_on_block and self.is_enabled and not self._is_scanning_path:
                        threading.Thread(target=self._execute_path_finding_scan, daemon=True).start()
                    return dec

            # Smooth, stable heading control - no aggressive bike-like swerving
            if abs(angle_error) <= 6.0:
                # Target is straight ahead within +/- 6° deadband -> drive straight forward
                steer = 0.0
                action = ActionEnum.FORWARD
            else:
                # Gentle proportional steering without aggressive swerving
                steer = -max(-0.45, min(0.45, angle_error / 55.0))
                action = ActionEnum.LEFT if angle_error > 6.0 else ActionEnum.RIGHT

            # Boundary Constraint: Never steer across lane boundaries!
            if lane_res is not None and lane_res.lane_detected:
                steer = max(lane_res.min_allowed_steering, min(lane_res.max_allowed_steering, steer))

            # Smooth cruising throttle (smooth & steady approach to target)
            align_factor = max(0.60, math.cos(math.radians(angle_error)))
            speed = max(0.26, min(self.cruise_speed, self.cruise_speed * align_factor))
            if distance_cm < 25.0:
                speed = max(0.24, speed * 0.70)

            reason_msg = f"Moving forward to Target ({active_destination.x:.0f},{active_destination.y:.0f}) | Dist: {distance_cm:.0f}cm"
            if self._loss_of_link_failsafe_active:
                reason_msg = f"[ONBOARD FAILSAFE] {reason_msg}"
            dec = DecisionResult(
                action=action,
                reason=reason_msg,
                clearance_left=cl_left,
                clearance_right=cl_right,
                suggested_throttle=round(speed, 2),
                suggested_steering=round(steer, 2),
                source="onboard_failsafe" if self._loss_of_link_failsafe_active else "destination",
            )
            with self._lock:
                self._current_decision = dec
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

        # Paid/cloud AI is advisory only and is reached after all local safety
        # gates. It cannot override a local obstacle or destination decision.
        self.paid_ai.submit_frame(frame, {
            "clearance_left": round(cl_left, 3),
            "clearance_right": round(cl_right, 3),
            "center_blocked": False,
            "detections": [d.class_name for d in detections],
        })
        if self.mode in ("paid_ai", "hybrid_paid"):
            paid_decision = self.paid_ai.get_latest_decision()
            if paid_decision is not None and paid_decision.action in (
                ActionEnum.FORWARD, ActionEnum.LEFT, ActionEnum.RIGHT
            ):
                with self._lock:
                    self._current_decision = paid_decision
                return paid_decision

        # Priority 5.5: Follow Lanes (Active Lane Keeping Assist)
        if lane_res is not None and lane_res.lane_detected and active_destination is None:
            steer = lane_res.suggested_steering
            action = ActionEnum.FORWARD if abs(steer) < 0.12 else (ActionEnum.RIGHT if steer > 0 else ActionEnum.LEFT)
            dec = DecisionResult(
                action=action,
                reason=f"Lane Assist: Centered in lane (Error: {lane_res.error:+.1f}px, Steer: {steer:+.2f})",
                clearance_left=cl_left,
                clearance_right=cl_right,
                suggested_steering=steer,
                suggested_throttle=self.cruise_speed,
                source="lane_assist"
            )
            with self._lock: self._current_decision = dec
            return dec

        # Priority 6: Path Clear -> Standby / Safe Hold unless Destination Target is assigned
        # Protects against unconstrained random roaming when no destination target is active
        if active_destination is None and (not getattr(self.config, 'roam_without_target', True) or self._loss_of_link_failsafe_active):
            self.motors.stop()
            reason_msg = "Standby: Awaiting target destination. Rover stopped (not roaming)."
            if self._loss_of_link_failsafe_active:
                reason_msg = "[ONBOARD FAILSAFE] Ground station disconnected & no target remaining. Holding position with active obstacle monitoring."
            dec = DecisionResult(
                action=ActionEnum.STOP,
                reason=reason_msg,
                hazard_detected=False,
                clearance_left=cl_left,
                clearance_right=cl_right,
                suggested_steering=0.0,
                suggested_throttle=0.0,
                source="onboard_failsafe" if self._loss_of_link_failsafe_active else "destination"
            )
            with self._lock: self._current_decision = dec
            return dec

        # Driving forward along clear corridor towards destination
        dec = DecisionResult(
            action=ActionEnum.FORWARD,
            reason="[ONBOARD FAILSAFE] Driving corridor clear. Cruising forward to target." if self._loss_of_link_failsafe_active else "Driving corridor clear. Cruising forward to target.",
            hazard_detected=False,
            clearance_left=cl_left,
            clearance_right=cl_right,
            suggested_steering=0.0,
            suggested_throttle=self.cruise_speed,
            source="onboard_failsafe" if self._loss_of_link_failsafe_active else "destination"
        )
        with self._lock: self._current_decision = dec
        return dec

    # -------------------------------------------------------------------------
    # Environmental Path-Finding Scan (Automated Camera Pan & Clearance Search)
    # -------------------------------------------------------------------------
    def _execute_path_finding_scan(self, trigger_reason: str = "obstacle") -> None:
        """
        Executes automated camera sweep to find a non-blind, unobstructed path to reach destination:
        1. Scan for Left (135°) then Right (45°) using smooth, slow, jitter-free movements.
        2. Hold while scanning Left for 1 to 2 seconds (default 1.5s) to sample HC sensor & vision.
        3. Scan for Right (45°) and hold for 1 to 2 seconds (default 1.5s) to sample HC sensor & vision.
        4. Analyze these two sides (corridor clearance, obstacle detections, physical HC distance, destination vector)
           and determine which side is better.
        5. Turn the camera to that particular chosen side and turn the vehicle towards that side!
        6. After the vehicle completes its turn, position the camera at the forward end portion (90.0° center)
           so forward driving and lane tracking are easy.
        Slow slewing eliminates all servo glitters (jitters) and twitching.
        """
        if self._is_scanning_path or not self._running or not self.is_enabled:
            return

        scan_generation = self._control_generation

        def scan_cancelled() -> bool:
            return (
                not self._running
                or not self.is_enabled
                or scan_generation != self._control_generation
            )

        self._is_scanning_path = True
        init_pose = self.waypoint_nav.odometry.pose
        active_dest = self.waypoint_nav.get_active_target()
        reason_label = "Blind Spot" if trigger_reason == "blind_spot" else "Obstacle Obstruction"

        if self._destination_reached or (active_dest is not None and active_dest.reached):
            logger.info("[PATH SCAN] Target location already reached. Aborting scan and holding stop position.")
            self.motors.stop()
            self._is_scanning_path = False
            return

        dest_str = f"({round(active_dest.x)},{round(active_dest.y)})" if active_dest else "None"
        logger.info(
            f"[PATH SCAN] {reason_label} at Pose: "
            f"x={init_pose.x:.1f}cm, y={init_pose.y:.1f}cm, heading={init_pose.heading_deg:.1f}° | "
            f"Active Destination: {dest_str}"
        )

        # If triggered by sonar collision / close obstacle (< 8 cm), create reverse clearance first
        if trigger_reason in ("collision_reverse", "sonar_avoid"):
            if scan_cancelled():
                self._is_scanning_path = False
                return
            logger.info("[PATH SCAN] Creating reverse clearance to avoid bumper scraping during scan...")
            self._execute_motor_action(DecisionResult(
                action=ActionEnum.REVERSE,
                reason="Path scan reverse clearance",
                hazard_detected=True,
                suggested_throttle=-0.38,
                suggested_steering=0.0,
                source="path_scan",
            ))
            time.sleep(0.35)
            if scan_cancelled():
                self._is_scanning_path = False
                return

        # Halt car during scan
        self.motors.stop()
        with self._lock:
            self._current_decision = DecisionResult(
                action=ActionEnum.SCAN,
                reason=f"{reason_label} at ({init_pose.x:.0f},{init_pose.y:.0f}). Initiating Left/Right scan to find better path...",
                hazard_detected=True,
                source="path_scan"
            )

        try:
            dark_thresh = getattr(self.config, 'darkness_mean_threshold', 14.0)
            blank_thresh = getattr(self.config, 'blank_std_threshold', 4.0)
            
            # Use configured hold duration (1.5s default for 1-2s requirement); fast in mock simulation tests
            hold_sec = getattr(self.config, 'scan_hold_duration_sec', 1.5)
            is_simulation = not getattr(self.servo, "_use_gpio", False)
            if is_simulation and not hasattr(self.servo, "is_initialized"):
                hold_sec = min(hold_sec, 0.02)

            step_deg = getattr(self.config, 'scan_smooth_step_deg', 2.0)
            step_delay = getattr(self.config, 'scan_smooth_delay_sec', 0.02)
            if is_simulation and not hasattr(self.servo, "is_initialized"):
                step_delay = 0.0

            # Compute desired bearing to destination if target is set
            desired_bearing = 0.0
            if active_dest is not None:
                dx = active_dest.x - init_pose.x
                dy = active_dest.y - init_pose.y
                target_heading = math.degrees(math.atan2(dy, dx)) % 360.0
                desired_bearing = target_heading - init_pose.heading_deg
                while desired_bearing > 180.0: desired_bearing -= 360.0
                while desired_bearing < -180.0: desired_bearing += 360.0

            def _evaluate_side(side_name: str, angle_deg: float) -> Optional[dict]:
                """Pans smoothly to side, holds for 1-2s, and evaluates visual & ultrasonic metrics."""
                if scan_cancelled():
                    return None
                logger.info(f"[PATH SCAN] 1. Smoothly scanning to {side_name.upper()} ({angle_deg:.0f}°)...")
                with self._lock:
                    self._current_decision = DecisionResult(
                        action=ActionEnum.SCAN,
                        reason=f"Scanning {side_name.upper()} ({angle_deg:.0f}°): slow smooth movement...",
                        hazard_detected=True,
                        source="path_scan"
                    )

                # Smooth slow movement to eliminate jitter/glitters
                self._set_servo_angle(
                    angle_deg,
                    auto_relax=False,
                    smooth=True,
                    step_delay_sec=step_delay,
                    step_deg=step_deg
                )
                if scan_cancelled():
                    return None

                logger.info(f"[PATH SCAN] 2. Holding at {side_name.upper()} ({angle_deg:.0f}°) for {hold_sec:.1f}s...")
                with self._lock:
                    self._current_decision = DecisionResult(
                        action=ActionEnum.SCAN,
                        reason=f"Holding at {side_name.upper()} for {hold_sec:.1f}s: analyzing clearance & obstacles...",
                        hazard_detected=True,
                        source="path_scan"
                    )

                # Stabilizing settling pause for camera sensor auto-exposure
                settle_time = min(0.20, hold_sec * 0.2)
                if settle_time > 0:
                    time.sleep(settle_time)
                if scan_cancelled():
                    return None

                # Sample physical distance from HC-SR04 ultrasonic sensor at this angle
                hc_samples = []
                if self.ultrasonic_sensors is not None:
                    try:
                        with self._sensor_lock:
                            d1 = self.ultrasonic_sensors.get_front_distance()
                        if d1 is not None:
                            hc_samples.append(d1)
                    except Exception as e:
                        logger.debug(f"[PATH SCAN] HC sensor read error on {side_name}: {e}")

                frame = self.camera.get_bgr_frame()
                valid_frame = (
                    isinstance(frame, np.ndarray)
                    and frame.size > 0
                    and frame.ndim in (2, 3)
                )
                if not valid_frame:
                    brightness = 0.0
                    contrast = 0.0
                    is_blind = True
                    clearance = 0.0
                    objects = []
                else:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if len(frame.shape) == 3 else frame
                    m_val, s_val = cv2.meanStdDev(gray)
                    brightness = float(m_val[0][0])
                    contrast = float(s_val[0][0])
                    is_blind = (brightness < dark_thresh) or (contrast < blank_thresh)

                    # Visual corridor clearance
                    clearance = self.corridor_scanner.measure_pan_clearance(frame)

                    # YOLO + large physical obstacle detection
                    dets = self.tracker.track(frame)
                    try:
                        large_obs = self.corridor_scanner.detect_large_obstacles(frame)
                        for lo in large_obs:
                            bx1, by1, bx2, by2 = lo["bbox"]
                            overlap = any(
                                (d.bbox.x1 < bx2 and d.bbox.x2 > bx1 and d.bbox.y1 < by2 and d.bbox.y2 > by1)
                                for d in dets
                            )
                            if not overlap:
                                dets.append(DetectionResult(
                                    class_name="obstacle",
                                    confidence=lo["confidence"],
                                    bbox=BoundingBox(x1=bx1, y1=by1, x2=bx2, y2=by2),
                                    track_id=None
                                ))
                    except Exception:
                        pass
                    objects = [d for d in dets if d.confidence >= 0.25]

                # Complete the requested 1-2 second hold duration
                remain_hold = max(0.0, hold_sec - settle_time)
                if remain_hold > 0:
                    # Take second HC reading near end of hold window
                    if self.ultrasonic_sensors is not None and remain_hold > 0.15:
                        time.sleep(remain_hold / 2.0)
                        try:
                            with self._sensor_lock:
                                d2 = self.ultrasonic_sensors.get_front_distance()
                            if d2 is not None:
                                hc_samples.append(d2)
                        except Exception:
                            pass
                        time.sleep(remain_hold / 2.0)
                    else:
                        time.sleep(remain_hold)

                if scan_cancelled():
                    return None

                # Best representative HC distance
                hc_dist = min(hc_samples) if hc_samples else None

                # Compute destination alignment
                cam_rel = angle_deg - 90.0
                alignment = 0.5
                if active_dest is not None:
                    bearing_diff = abs(cam_rel - desired_bearing)
                    alignment = max(0.0, 1.0 - (bearing_diff / 180.0))

                # Compute HC score: the HC-SR04 is stationary on the front bumper.
                # Only apply front proximity penalty when camera is facing forward (abs(cam_rel) <= 25°).
                # Lateral paths are evaluated using camera optical clearance to avoid false penalties.
                hc_score_mod = 0.0
                hc_blocked = False
                hc_str = f"{hc_dist:.1f}cm" if hc_dist is not None else "None"
                is_facing_front = abs(cam_rel) <= 25.0
                if is_facing_front and hc_dist is not None:
                    if hc_dist <= 20.0:
                        hc_blocked = True
                        hc_score_mod = -350.0  # Physical collision hazard in front
                    elif hc_dist <= 35.0:
                        hc_score_mod = -150.0
                    elif hc_dist <= 60.0:
                        hc_score_mod = (hc_dist / 60.0) * 20.0
                    else:
                        hc_score_mod = min(60.0, (hc_dist / 100.0) * 60.0)

                # Total side evaluation score
                if is_blind:
                    score = -1000.0
                else:
                    score = (clearance * 50.0) - (len(objects) * 120.0) + (alignment * 45.0) + (min(100.0, brightness) / 10.0) + hc_score_mod

                res = {
                    "side": side_name,
                    "angle": angle_deg,
                    "is_blind": is_blind,
                    "brightness": brightness,
                    "contrast": contrast,
                    "clearance": clearance,
                    "objects": objects,
                    "hc_distance": hc_dist,
                    "hc_blocked": hc_blocked,
                    "alignment": alignment,
                    "score": score,
                    "desc": f"HC={hc_str}, Clearance={clearance:.2f}, Objs={len(objects)}, Align={alignment:.2f}, Score={score:.1f}"
                }
                logger.info(f"[PATH SCAN RESULT] {side_name.upper()} ({angle_deg:.0f}°): {res['desc']}")
                return res

            # =================================================================
            # 1. SCAN FOR LEFT (135°) AND HOLD FOR 1 TO 2 SECONDS
            # =================================================================
            left_eval = _evaluate_side("left", 135.0)
            if left_eval is None:
                return

            # =================================================================
            # 2. SCAN FOR RIGHT (45°) AND HOLD FOR 1 TO 2 SECONDS
            # =================================================================
            right_eval = _evaluate_side("right", 45.0)
            if right_eval is None:
                return

            if scan_cancelled():
                return

            # =================================================================
            # 3. ANALYZE THESE TWO SIDES AND FIND WHICH SIDE IS BETTER
            # =================================================================
            logger.info(
                f"[PATH SCAN ANALYSIS] Left Score: {left_eval['score']:.1f} (HC: {left_eval['hc_distance']}cm) | "
                f"Right Score: {right_eval['score']:.1f} (HC: {right_eval['hc_distance']}cm)"
            )

            # Decision Logic:
            # - If both blind: fail safe
            # - If one blind, choose non-blind
            # - If one physically blocked by HC (<=20cm), choose the other
            # - Otherwise choose side with highest composite score
            if left_eval["is_blind"] and right_eval["is_blind"]:
                logger.warning("[PATH SCAN] Both Left and Right are BLIND SPOTS! Halting vehicle safely.")
                self.motors.stop()
                with self._lock:
                    self._current_decision = DecisionResult(
                        action=ActionEnum.STOP,
                        reason="ALL PATHS BLIND: Both Left and Right are blind/dark. Vehicle halted.",
                        hazard_detected=True,
                        source="vision_blind"
                    )
                return

            if left_eval["is_blind"] and not right_eval["is_blind"]:
                best_eval = right_eval
            elif right_eval["is_blind"] and not left_eval["is_blind"]:
                best_eval = left_eval
            elif left_eval["hc_blocked"] and not right_eval["hc_blocked"]:
                best_eval = right_eval
            elif right_eval["hc_blocked"] and not left_eval["hc_blocked"]:
                best_eval = left_eval
            else:
                best_eval = left_eval if left_eval["score"] >= right_eval["score"] else right_eval

            best_side = best_eval["side"]
            best_angle = best_eval["angle"]
            best_hc = best_eval["hc_distance"]
            best_hc_str = f"{best_hc:.1f}cm" if best_hc is not None else "N/A"

            logger.info(
                f"[PATH SCAN WINNER] Chosen Side: {best_side.upper()} ({best_angle:.0f}°) | "
                f"HC Dist: {best_hc_str} | Score: {best_eval['score']:.1f} | "
                f"Clearance: {best_eval['clearance']:.2f} | Objs: {len(best_eval['objects'])}"
            )

            # =================================================================
            # 4. TURN THE CAM TO THAT PARTICULAR SIDE AND TURN THE VEHICLE
            # =================================================================
            # Turn camera smoothly to the winning side
            logger.info(f"[PATH SCAN] Turning camera to chosen side: {best_side.upper()} ({best_angle:.0f}°)...")
            with self._lock:
                self._current_decision = DecisionResult(
                    action=ActionEnum.SCAN,
                    reason=f"Chosen path: {best_side.upper()} ({best_angle:.0f}°). Turning camera and steering vehicle...",
                    hazard_detected=True,
                    source="path_scan"
                )
            self._set_servo_angle(
                best_angle,
                auto_relax=False,
                smooth=True,
                step_delay_sec=step_delay,
                step_deg=step_deg
            )
            time.sleep(0.35)
            if scan_cancelled():
                return

            # Turn vehicle towards the chosen side
            try:
                from control.serial_comm import VehicleCommand
            except ImportError:
                from autonomous_car.control.serial_comm import VehicleCommand

            should_maneuver = self.is_enabled and not scan_cancelled()
            if should_maneuver:
                if best_side == "left":
                    logger.info(f"[PATH SCAN] Steering vehicle LEFT into open corridor...")
                    self._execute_motor_action(DecisionResult(
                        action=ActionEnum.LEFT,
                        reason="Path scan selected left corridor",
                        hazard_detected=True,
                        suggested_throttle=self.cruise_speed,
                        suggested_steering=-0.75,
                        source="path_scan",
                    ))
                    time.sleep(0.35)
                    if scan_cancelled():
                        return
                    self.waypoint_nav.odometry.update(VehicleCommand.LEFT, dt=0.35)

                    # Seamlessly continue cruising along clear path
                    self._execute_motor_action(DecisionResult(
                        action=ActionEnum.FORWARD,
                        reason="Path scan continuing through left corridor",
                        suggested_throttle=self.cruise_speed,
                        suggested_steering=0.0,
                        source="path_scan",
                    ))
                    time.sleep(0.30)
                    if scan_cancelled():
                        return
                    self.waypoint_nav.odometry.update(VehicleCommand.FORWARD, dt=0.30)

                else:  # right
                    logger.info(f"[PATH SCAN] Steering vehicle RIGHT into open corridor...")
                    self._execute_motor_action(DecisionResult(
                        action=ActionEnum.RIGHT,
                        reason="Path scan selected right corridor",
                        hazard_detected=True,
                        suggested_throttle=self.cruise_speed,
                        suggested_steering=0.75,
                        source="path_scan",
                    ))
                    time.sleep(0.35)
                    if scan_cancelled():
                        return
                    self.waypoint_nav.odometry.update(VehicleCommand.RIGHT, dt=0.35)

                    # Seamlessly continue cruising along clear path
                    self._execute_motor_action(DecisionResult(
                        action=ActionEnum.FORWARD,
                        reason="Path scan continuing through right corridor",
                        suggested_throttle=self.cruise_speed,
                        suggested_steering=0.0,
                        source="path_scan",
                    ))
                    time.sleep(0.30)
                    if scan_cancelled():
                        return
                    self.waypoint_nav.odometry.update(VehicleCommand.FORWARD, dt=0.30)

                post_pose = self.waypoint_nav.odometry.pose
                logger.info(
                    f"[PATH SCAN] Maneuver complete! New Pose: "
                    f"x={post_pose.x:.1f}cm, y={post_pose.y:.1f}cm, heading={post_pose.heading_deg:.1f}°. "
                    f"Vehicle aligned with open corridor."
                )

                # Check if maneuver reached the destination
                if active_dest is not None:
                    _, dist_cm, reached = self.waypoint_nav.compute_steering_to_target()
                    if reached or self.waypoint_nav.get_active_target() is None:
                        logger.info(f"[PATH SCAN] Destination reached ({dist_cm:.1f}cm)! Stopping rover.")
                        self.motors.stop()
                        self.is_enabled = False
                        self._destination_reached = True
                        dec = DecisionResult(
                            action=ActionEnum.STOP,
                            reason=f"🎯 DESTINATION REACHED! Arrived safely at ({active_dest.x:.0f},{active_dest.y:.0f}). Stopped.",
                            hazard_detected=False,
                            suggested_steering=0.0,
                            suggested_throttle=0.0,
                            source="destination"
                        )
                        with self._lock:
                            self._current_decision = dec
                            self._active_3d_target = None
                            self._current_reprojected_pin = None
                        return

            # Reset blind state so navigation loop continues without being blocked
            with self._lock:
                self._is_blind = False
                self._blind_reason = ""

        except Exception as e:
            logger.error(f"[PATH SCAN] Error during servo sweep: {e}")
        finally:
            # =================================================================
            # 5. AFTER THAT POSITION THE CAMERA AT THIS END PORTION (90° CENTER)
            # =================================================================
            logger.info("[PATH SCAN] Maneuver finished: positioning camera back to center (90.0°)...")
            step_deg = getattr(self.config, 'scan_smooth_step_deg', 2.0)
            step_delay = getattr(self.config, 'scan_smooth_delay_sec', 0.02)
            if is_simulation and not hasattr(self.servo, "is_initialized"):
                step_delay = 0.0
            self._set_servo_angle(
                90.0,
                auto_relax=False,
                smooth=True,
                step_delay_sec=step_delay,
                step_deg=step_deg
            )
            self._is_scanning_path = False

    def _set_servo_angle(
        self,
        angle: float,
        auto_relax: bool = True,
        smooth: bool = False,
        step_delay_sec: float = 0.02,
        step_deg: float = 2.0
    ) -> None:
        """Safely commands pan servo angle with fallback for mock servos and optional anti-jitter smooth slewing."""
        if self.servo is None:
            return
        if smooth and hasattr(self.servo, "set_angle_smooth"):
            try:
                self.servo.set_angle_smooth(
                    angle,
                    step_deg=step_deg,
                    step_delay_sec=step_delay_sec,
                    auto_relax=auto_relax
                )
                return
            except Exception as e:
                logger.debug(f"[AUTO DRIVER] Smooth servo set error: {e}")
        try:
            self.servo.set_angle(angle, auto_relax=auto_relax)
        except TypeError:
            self.servo.set_angle(angle)

    # -------------------------------------------------------------------------
    # Motor Action Execution
    # -------------------------------------------------------------------------
    def _execute_motor_action(self, decision: DecisionResult) -> None:
        # Manual control and shutdown have absolute ownership.  This guard is
        # intentionally repeated here because path scans run in worker threads
        # and may finish after the dashboard has disarmed AI.
        if not self.is_enabled:
            self.motors.stop()
            return

        action = decision.action

        if action in (ActionEnum.STOP, ActionEnum.WAIT, ActionEnum.SCAN, ActionEnum.UNKNOWN):
            self.motors.stop()
            return

        cmd = MotionCommand(
            throttle=decision.suggested_throttle,
            steering=decision.suggested_steering,
            action=action.value if hasattr(action, 'value') else str(action),
            source="autonomous_driver"
        )
        # Path scans and other bounded maneuvers can run in a worker thread
        # while the main loop is deliberately paused. The command itself is
        # proof that the safety-controlled driver is alive, so refresh the
        # watchdog immediately before validation.
        self.safety_supervisor.heartbeat()
        camera_age = None
        if hasattr(self.camera, "frame_age_seconds"):
            try:
                camera_age = float(self.camera.frame_age_seconds())
            except (TypeError, ValueError):
                # Alternate camera backends and test doubles may not expose a
                # numeric age; the perception loop still checks frame validity.
                camera_age = None
        safe_res = self.safety_supervisor.validate_command(
            proposed=cmd,
            ultrasonic=self._front_sensor_data,
            camera_frame_age_sec=camera_age,
            is_blind=self._is_blind,
        )
        if not safe_res.allowed or safe_res.action == "STOP":
            self.motors.stop()
            with self._lock:
                self._current_decision = DecisionResult(
                    action=ActionEnum.STOP,
                    reason=f"[SAFETY] {safe_res.reason}",
                    hazard_detected=True,
                )
            return

        self.motors.drive(throttle=safe_res.safe_throttle, steering=safe_res.safe_steering)

    @staticmethod
    def _odometry_command(action: ActionEnum):
        from control.serial_comm import VehicleCommand
        return {
            ActionEnum.FORWARD: VehicleCommand.FORWARD,
            ActionEnum.LEFT: VehicleCommand.LEFT,
            ActionEnum.RIGHT: VehicleCommand.RIGHT,
            ActionEnum.REVERSE: VehicleCommand.BACKWARD,
        }.get(action, VehicleCommand.STOP)

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

        cur_pose = self.waypoint_nav.odometry.pose
        pin_render = self.get_current_reprojected_pin() or self._active_3d_target
        return Visualizer.draw(
            frame=vis_frame,
            detections=dets,
            risks=risks,
            decision=dec,
            fps=fps,
            latency_ms=latency,
            lane_width_ratio=self.config.lane_width_ratio,
            mode_str=mode_str,
            auto_enabled=auto_en,
            target_pin=pin_render,
            current_pose={"x": cur_pose.x, "y": cur_pose.y, "heading_deg": cur_pose.heading_deg},
            last_obstacle_pose=self._last_obstacle_pose,
        )

    # -------------------------------------------------------------------------
    # Status API Dict
    # -------------------------------------------------------------------------
    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            dec = self._current_decision
            server_stat = self.server_client.get_status()
            cur_pose = self.waypoint_nav.odometry.pose
            pin_data = self.get_current_reprojected_pin() or self._active_3d_target
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
            sensor_age = None
            sensor_status = "UNAVAILABLE"
            if self._front_sensor_data is not None:
                sensor_status = self._front_sensor_data.status
                if self._front_sensor_data.timestamp > 0:
                    sensor_age = round(max(0.0, time.time() - self._front_sensor_data.timestamp), 3)

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
                "is_blind": self._is_blind,
                "blind_reason": self._blind_reason,
                "frame_brightness": round(self._frame_mean_brightness, 1),
                "frame_contrast": round(self._frame_std_contrast, 1),
                "camera_frame_age_s": self.camera.frame_age_seconds(),
                "cruise_speed": self.cruise_speed,
                "avoid_speed": self.avoid_speed,
                "show_overlay": self.show_overlay,
                "is_scanning": self._is_scanning_path,
                "front_distance_cm": self._front_distance_cm,
                "rear_distance_cm": self._rear_distance_cm,
                "front_sensor_age_s": sensor_age,
                "front_sensor_status": sensor_status,
                "detections_count": len(self._current_detections),
                "detections": dets_summary,
                "target_pin": pin_data,
                "marker_route": self.get_marker_status(),
                "current_pose": {"x": round(cur_pose.x, 1), "y": round(cur_pose.y, 1), "heading_deg": round(cur_pose.heading_deg, 1)},
                "last_obstacle_pose": self._last_obstacle_pose,
                "servo_angle": round(getattr(self.servo, 'current_angle', 90.0), 1),
                "active_gaze_tracking": self.active_gaze_tracking,
                "mapped_obstacles": self._mapped_3d_obstacles,
                "gps": self.get_current_gps(),
                "server_ai": server_stat,
                "ground_station": self.get_ground_station_status(),
                "lane_assist": {
                    "enabled": self.lane_assist_enabled,
                    "lane_detected": getattr(self._current_lane_result, "lane_detected", False) if self._current_lane_result else False,
                    "error_px": getattr(self._current_lane_result, "error", 0.0) if self._current_lane_result else 0.0,
                    "departure_warning": getattr(self._current_lane_result, "departure_warning", None) if self._current_lane_result else None,
                    "left_line_x": getattr(self._current_lane_result, "left_line_x", None) if self._current_lane_result else None,
                    "right_line_x": getattr(self._current_lane_result, "right_line_x", None) if self._current_lane_result else None,
                    "suggested_steering": getattr(self._current_lane_result, "suggested_steering", 0.0) if self._current_lane_result else 0.0,
                },
                "traffic_rules": {
                    "enabled": self.traffic_rules_enabled,
                    "rule_name": getattr(self._current_traffic_result, "rule_name", "NONE") if self._current_traffic_result else "NONE",
                    "traffic_light": getattr(self._current_traffic_result.traffic_light_state, "value", "UNKNOWN") if (self._current_traffic_result and hasattr(self._current_traffic_result, "traffic_light_state")) else "UNKNOWN",
                    "stop_sign": getattr(self._current_traffic_result.stop_sign_state, "value", "IDLE") if (self._current_traffic_result and hasattr(self._current_traffic_result, "stop_sign_state")) else "IDLE",
                    "dwell_remaining_sec": getattr(self._current_traffic_result, "dwell_remaining_sec", 0.0) if self._current_traffic_result else 0.0,
                },
                "paid_ai": self.paid_ai.get_status(),
                "destinations": self.get_destinations(),
                "slam": self.slam_bridge.get_telemetry() if self.slam_bridge is not None else None,
            }

    def reset_slam(self) -> Dict[str, Any]:
        """Resets the Visual SLAM map and visual odometry tracker."""
        if self.slam_bridge is not None:
            self.slam_bridge.reset()
            return {"status": "success", "message": "Visual SLAM map and pose reset."}
        return {"status": "error", "message": "SLAM bridge not initialized"}

    def set_target_pin(self, x: int, y: int) -> Dict[str, Any]:
        """
        Projects tapped camera pixel (x, y) into 3D world ground coordinate (world_x, world_y),
        anchors it to the vehicle's global odometry frame, and registers it as the destination!
        """
        current_servo = getattr(self.servo, 'current_angle', 90.0)
        wp, info = self.waypoint_nav.set_3d_destination_from_camera(x, y, servo_angle_deg=current_servo)
        self._active_3d_target = info
        bearing = round(math.degrees(math.atan2(info["rel_lateral_cm"], info["rel_forward_cm"])), 1)
        self._current_reprojected_pin = {
            "x": x,
            "y": y,
            "world_x": info["world_x"],
            "world_y": info["world_y"],
            "dist_cm": info["distance_cm"],
            "bearing_deg": bearing,
            "in_fov": True,
            "label": f"3D Target ({info['world_x']:.0f},{info['world_y']:.0f})"
        }
        # Do not retain the tapped pixel as a moving navigation target.  The
        # pixel is only the measurement used to create the fixed world anchor
        # above; subsequent steering/reprojection must use that world anchor.
        self.set_enabled(True)
        logger.info(
            f"[AUTO DRIVER 3D DESTINATION] Plotted target at ({x}, {y}) [Servo {current_servo:.1f}°] -> World 3D: "
            f"X={info['world_x']:.1f}cm, Y={info['world_y']:.1f}cm | Dist: {info['distance_cm']:.1f}cm | Bearing: {bearing:+.1f}°"
        )
        return self._current_reprojected_pin

    def set_target_3d(self, world_x: float, world_y: float) -> Dict[str, Any]:
        """
        Registers a fixed 3D ground target directly in world coordinates (world_x, world_y).
        Anchors it in odometry frame and initiates active gaze tracking.
        """
        self.waypoint_nav.clear_waypoints()
        wp = self.waypoint_nav.add_waypoint(world_x, world_y, label=f"3D Target ({world_x:.0f},{world_y:.0f})")
        cur_pose = self.waypoint_nav.odometry.pose
        dx = world_x - cur_pose.x
        dy = world_y - cur_pose.y
        dist = math.hypot(dx, dy)
        rad = math.radians(cur_pose.heading_deg)
        Y_rel = (dx * math.cos(rad)) + (dy * math.sin(rad))
        X_rel = (dx * math.sin(rad)) - (dy * math.cos(rad))
        bearing = math.degrees(math.atan2(X_rel, Y_rel))
        info = {
            "world_x": round(world_x, 1),
            "world_y": round(world_y, 1),
            "rel_forward_cm": round(Y_rel, 1),
            "rel_lateral_cm": round(X_rel, 1),
            "distance_cm": round(dist, 1),
        }
        self._active_3d_target = info

        current_servo = getattr(self.servo, 'current_angle', 90.0)
        reproj = self.waypoint_nav.get_reprojected_pixel(servo_angle_deg=current_servo)
        if reproj is not None:
            u_p, v_p, d_p, b_p = reproj
            self._current_reprojected_pin = {
                "x": u_p, "y": v_p, "dist_cm": d_p, "bearing_deg": b_p,
                "in_fov": (0 <= u_p <= 640 and 0 <= v_p <= 480),
                "world_x": round(world_x, 1), "world_y": round(world_y, 1),
                "label": f"3D Target ({world_x:.0f},{world_y:.0f})"
            }
        else:
            self._current_reprojected_pin = {
                "x": -1, "y": -1, "dist_cm": round(dist, 1), "bearing_deg": round(bearing, 1),
                "in_fov": False, "world_x": round(world_x, 1), "world_y": round(world_y, 1),
                "label": f"3D Target ({world_x:.0f},{world_y:.0f})"
            }
        self.set_enabled(True)
        logger.info(
            f"[AUTO DRIVER 3D DESTINATION] Plotted target via 3D Map at World ({world_x:.1f}, {world_y:.1f})cm | "
            f"Dist: {dist:.1f}cm | Rel: Fwd {Y_rel:.1f}cm, Lat {X_rel:.1f}cm"
        )
        return self._current_reprojected_pin

    def set_gaze_tracking(self, enabled: bool) -> bool:
        """Enables or disables active pan-servo gaze locking on 3D target."""
        self.active_gaze_tracking = bool(enabled)
        if not self.active_gaze_tracking and not self._is_scanning_path:
            self._set_servo_angle(90.0, auto_relax=False)
        logger.info(f"[AUTO DRIVER] Active Gaze Tracking: {'ENABLED' if self.active_gaze_tracking else 'DISABLED'}")
        return self.active_gaze_tracking

    def clear_target_pin(self) -> None:
        """Clears 3D visual destination target and re-centers pan servo."""
        self._destination_reached = False
        self._active_3d_target = None
        self._current_reprojected_pin = None
        self.motors.stop()
        self.waypoint_nav.clear_waypoints()
        if self.visual_navigator:
            self.visual_navigator.clear_manual_destination()
        if not self._is_scanning_path:
            self._set_servo_angle(90.0, auto_relax=False)
        logger.info("[AUTO DRIVER] 3D Destination pin cleared. Servo re-centered to 90°.")

    def get_current_reprojected_pin(self, servo_angle: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """
        Dynamically calculates the 2D image pixel reprojection of the active 3D world target
        based on the vehicle's current odometry pose and current camera pan servo angle.
        If the target is outside the camera FOV or behind the camera, returns in_fov=False with
        bearing/distance intact so the dashboard UI and reticle behave accurately.
        """
        active_dest = self.waypoint_nav.get_active_target()
        if active_dest is None:
            with self._lock:
                self._current_reprojected_pin = None
            return None

        cur_pose = self.waypoint_nav.odometry.pose
        current_servo = float(servo_angle) if servo_angle is not None else float(getattr(self.servo, 'current_angle', 90.0))

        dx = active_dest.x - cur_pose.x
        dy = active_dest.y - cur_pose.y
        dist_p = math.hypot(dx, dy)
        target_heading = math.degrees(math.atan2(dy, dx)) % 360.0
        bearing_to_target = target_heading - cur_pose.heading_deg
        while bearing_to_target > 180.0:
            bearing_to_target -= 360.0
        while bearing_to_target < -180.0:
            bearing_to_target += 360.0

        reproj = self.waypoint_nav.get_reprojected_pixel(servo_angle_deg=current_servo)
        if reproj is not None:
            u_p, v_p, d_p, b_p = reproj
            pin = {
                "x": int(u_p),
                "y": int(v_p),
                "dist_cm": round(d_p, 1),
                "bearing_deg": round(b_p, 1),
                "in_fov": bool(0 <= u_p <= 640 and 0 <= v_p <= 480),
                "world_x": round(active_dest.x, 1),
                "world_y": round(active_dest.y, 1),
                "label": active_dest.label,
                "servo_angle": round(current_servo, 1),
            }
        else:
            pin = {
                "x": -1,
                "y": -1,
                "dist_cm": round(dist_p, 1),
                "bearing_deg": round(bearing_to_target, 1),
                "in_fov": False,
                "world_x": round(active_dest.x, 1),
                "world_y": round(active_dest.y, 1),
                "label": active_dest.label,
                "servo_angle": round(current_servo, 1),
            }

        with self._lock:
            self._current_reprojected_pin = pin
        return pin

    def set_gps_origin(self, lat: float, lon: float) -> Dict[str, Any]:
        """Sets real-world GPS origin (lat, lon) for metric (cm) odometry frame."""
        self.gps_origin = {
            "latitude": float(lat),
            "longitude": float(lon),
            "initialized": True,
        }
        logger.info(f"[GPS] Origin set to ({lat:.6f}, {lon:.6f})")
        return self.gps_origin

    def set_target_gps(self, lat: float, lon: float, label: str = "GPS Target") -> Dict[str, Any]:
        """
        Converts real-world target (lat, lon) to local metric ground coordinates (cm)
        using WGS-84 Flat-Earth Equirectangular projection relative to GPS origin.
        """
        if not self.gps_origin.get("initialized"):
            self.gps_origin["latitude"] = float(lat)
            self.gps_origin["longitude"] = float(lon)
            self.gps_origin["initialized"] = True

        lat_0 = math.radians(self.gps_origin["latitude"])
        delta_lat = float(lat) - self.gps_origin["latitude"]
        delta_lon = float(lon) - self.gps_origin["longitude"]

        # 1 deg latitude ≈ 111,139 meters = 11,113,900 cm
        # 1 deg longitude ≈ 111,139 * cos(lat0) meters = 11,113,900 * cos(lat0) cm
        cm_per_deg_lat = 11113900.0
        cm_per_deg_lon = 11113900.0 * math.cos(lat_0)

        world_y = delta_lat * cm_per_deg_lat  # North = +Y
        world_x = delta_lon * cm_per_deg_lon  # East = +X

        pin = self.set_target_3d(world_x, world_y)
        pin["target_lat"] = float(lat)
        pin["target_lon"] = float(lon)
        pin["label"] = label
        logger.info(f"[GPS TARGET] Plotted ({lat:.6f}, {lon:.6f}) -> Local 3D ({world_x:.1f}, {world_y:.1f})cm")
        return pin

    def get_current_gps(self) -> Dict[str, Any]:
        """Estimates car's current real-world GPS coordinates based on odometry pose."""
        cur_pose = self.waypoint_nav.odometry.pose
        lat_0 = self.gps_origin.get("latitude", 9.5916)
        lon_0 = self.gps_origin.get("longitude", 76.5222)
        lat_0_rad = math.radians(lat_0)

        cm_per_deg_lat = 11113900.0
        cm_per_deg_lon = 11113900.0 * math.cos(lat_0_rad)

        est_lat = lat_0 + (cur_pose.y / cm_per_deg_lat)
        est_lon = lon_0 + (cur_pose.x / cm_per_deg_lon)

        return {
            "latitude": round(est_lat, 7),
            "longitude": round(est_lon, 7),
            "origin": dict(self.gps_origin),
            "pose_x_cm": round(cur_pose.x, 1),
            "pose_y_cm": round(cur_pose.y, 1),
            "heading_deg": round(cur_pose.heading_deg, 1)
        }

