"""
Navigation Planner Module.
Fuses inputs from Lane Detector, Junction Detector, Ultrasonic Sensors, YOLO Object Detector,
and Waypoint Navigator. Executes Automatic Camera Servo Scanning & Obstacle Avoidance Rotation.
"""

import os
import sys
import time
from typing import Optional, Tuple

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from navigation.state_machine import NavigationStateMachine, State
    from navigation.waypoint import WaypointNavigator
    from vision.lane_detector import LaneResult, Direction
    from vision.junction_detector import JunctionResult, JunctionType
    from vision.object_detector import ObjectDetectionResult
    from sensors.ultrasonic import DualUltrasonicData, UltrasonicSensors
    from control.controller import VehicleController
    from utils.logger import car_logger
except ImportError:
    from autonomous_car.navigation.state_machine import NavigationStateMachine, State
    from autonomous_car.navigation.waypoint import WaypointNavigator
    from autonomous_car.vision.lane_detector import LaneResult, Direction
    from autonomous_car.vision.junction_detector import JunctionResult, JunctionType
    from autonomous_car.vision.object_detector import ObjectDetectionResult
    from autonomous_car.sensors.ultrasonic import DualUltrasonicData, UltrasonicSensors
    from autonomous_car.control.controller import VehicleController
    from autonomous_car.utils.logger import car_logger

class NavigationPlanner:
    def __init__(
        self,
        state_machine: NavigationStateMachine,
        controller: VehicleController,
        waypoint_navigator: Optional[WaypointNavigator] = None
    ) -> None:
        self.state_machine: NavigationStateMachine = state_machine
        self.controller: VehicleController = controller
        self.waypoint_nav: Optional[WaypointNavigator] = waypoint_navigator
        self._enabled: bool = False  # Start in Manual Control Mode by default for safety
        self._is_scanning: bool = False

    def enable(self) -> None:
        self._enabled = True
        self.state_machine.transition_to(State.FOLLOW_LANE, "Planner enabled")

    def disable(self) -> None:
        self._enabled = False
        self.state_machine.transition_to(State.MANUAL_DRIVE, "Planner disabled / Manual Mode")
        self.controller.stop()

    def execute_environmental_scan(self, ultrasonic_sensors: Optional[UltrasonicSensors] = None) -> State:
        """
        Automated Obstacle Avoidance Scanning:
        1. Stops vehicle.
        2. Automatically rotates Camera Servo to Left (150°) & measures clearance.
        3. Automatically rotates Camera Servo to Right (30°) & measures clearance.
        4. Recenters Camera Servo to 90°.
        5. Automatically rotates car towards the path with maximum open clearance.
        """
        if self._is_scanning or self.state_machine.current_state == State.MANUAL_DRIVE:
            return self.state_machine.current_state

        self._is_scanning = True
        self.state_machine.transition_to(State.SCANNING_ENVIRONMENT, "Obstacle Ahead: Automatically scanning environment")
        self.controller.stop()

        scan_delay = self.controller.config.servo.scan_delay_sec
        left_dist = 0.0
        right_dist = 0.0

        # 1. Automatically rotate Camera Servo Left (150 degrees)
        car_logger.info("[AUTO-SCAN]: Rotating Camera Servo Left (150°)...")
        self.controller.pan_camera_left()
        time.sleep(scan_delay)
        if ultrasonic_sensors:
            left_dist = ultrasonic_sensors.read_all().front.distance_cm
        else:
            left_dist = self.controller.serial_comm.get_ultrasonic_distances()[0]

        # 2. Automatically rotate Camera Servo Right (30 degrees)
        car_logger.info("[AUTO-SCAN]: Rotating Camera Servo Right (30°)...")
        self.controller.pan_camera_right()
        time.sleep(scan_delay)
        if ultrasonic_sensors:
            right_dist = ultrasonic_sensors.read_all().front.distance_cm
        else:
            right_dist = self.controller.serial_comm.get_ultrasonic_distances()[0]

        # 3. Recenter Camera Servo (90 degrees)
        car_logger.info("[AUTO-SCAN]: Recentering Camera Servo (90°)...")
        self.controller.pan_camera_center()
        time.sleep(0.2)

        car_logger.info(f"[SCAN RESULTS]: Left Open Space = {left_dist:.1f} cm | Right Open Space = {right_dist:.1f} cm")

        self._is_scanning = False
        warning_thresh = self.controller.config.sensor.warning_distance_cm

        # Automatically execute car rotation towards clearest path
        if left_dist > right_dist and left_dist > warning_thresh:
            self.state_machine.transition_to(State.TURN_LEFT, f"Auto-Rotate Decision: Turn Left (Clearance {left_dist:.1f}cm)")
            self.controller.turnLeft(1.0)
            return State.TURN_LEFT

        elif right_dist > left_dist and right_dist > warning_thresh:
            self.state_machine.transition_to(State.TURN_RIGHT, f"Auto-Rotate Decision: Turn Right (Clearance {right_dist:.1f}cm)")
            self.controller.turnRight(1.0)
            return State.TURN_RIGHT

        else:
            self.state_machine.transition_to(State.STOP, "Auto-Rotate Decision: All paths blocked")
            self.controller.stop()
            return State.STOP

    def update(
        self,
        lane_res: LaneResult,
        junction_res: JunctionResult,
        sensor_data: DualUltrasonicData,
        yolo_res: Optional[ObjectDetectionResult] = None,
        ultrasonic_sensors: Optional[UltrasonicSensors] = None
    ) -> State:
        # In MANUAL_DRIVE or when disabled, do not override user manual control
        if not self._enabled or self._is_scanning or self.state_machine.current_state == State.MANUAL_DRIVE:
            return self.state_machine.current_state

        if self.waypoint_nav:
            self.waypoint_nav.odometry.update(self.controller.serial_comm.last_command)

        # 1. OBSTACLE SCANNING & AUTOMATIC ROTATION PRIORITY
        if sensor_data.front.obstacle_warning or sensor_data.front.emergency_stop or (yolo_res and yolo_res.obstacle_detected):
            return self.execute_environmental_scan(ultrasonic_sensors)

        if yolo_res and yolo_res.stop_sign_detected:
            self.state_machine.transition_to(State.STOP, "YOLO Stop Sign Detected")
            self.controller.stop()
            return State.STOP

        # 2. WAYPOINT NAVIGATION PRIORITY
        if self.waypoint_nav and self.waypoint_nav.get_active_target() is not None:
            angle_error_deg, dist_cm, reached = self.waypoint_nav.compute_steering_to_target()
            
            if reached:
                if self.waypoint_nav.get_active_target() is None:
                    self.state_machine.transition_to(State.WAYPOINT_REACHED, "All Waypoints Reached")
                    self.controller.stop()
                    return State.WAYPOINT_REACHED

            self.state_machine.transition_to(State.NAVIGATING_WAYPOINTS, "Navigating to Waypoint Target")

            turn_threshold_deg = 15.0
            if angle_error_deg > turn_threshold_deg:
                self.controller.turnLeft(0.8)
            elif angle_error_deg < -turn_threshold_deg:
                self.controller.turnRight(0.8)
            else:
                self.controller.moveForward(1.0)

            return State.NAVIGATING_WAYPOINTS

        # 3. JUNCTION PRIORITY
        if junction_res.junction_type == JunctionType.LEFT_TURN:
            self.state_machine.transition_to(State.TURN_LEFT, "Junction: Left Turn Detected")
            self.controller.turnLeft(1.0)
            return State.TURN_LEFT

        elif junction_res.junction_type == JunctionType.RIGHT_TURN:
            self.state_machine.transition_to(State.TURN_RIGHT, "Junction: Right Turn Detected")
            self.controller.turnRight(1.0)
            return State.TURN_RIGHT

        elif junction_res.junction_type == JunctionType.DEAD_END:
            self.state_machine.transition_to(State.STOP, "Junction: Dead End Detected")
            self.controller.stop()
            return State.STOP

        # 4. NORMAL LANE FOLLOWING MODE
        if lane_res.direction == Direction.NO_LANE:
            self.state_machine.transition_to(State.STOP, "No Lane Contours Detected")
            self.controller.stop()
            return State.STOP

        self.state_machine.transition_to(State.FOLLOW_LANE, "Following Lane")
        steering = self.controller.compute_steering(lane_res.error)
        self.controller.apply_steering_control(steering)

        return self.state_machine.current_state
