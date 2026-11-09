"""
Waypoint Navigation and 2D Dead-Reckoning Odometry module.
"""

import os
import sys
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict, Any

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from config import WaypointConfig
    from control.serial_comm import VehicleCommand
    from utils.logger import car_logger
except ImportError:
    from autonomous_car.config import WaypointConfig
    from autonomous_car.control.serial_comm import VehicleCommand
    from autonomous_car.utils.logger import car_logger

@dataclass
class Point2D:
    x: float
    y: float

@dataclass
class Waypoint:
    id: int
    x: float
    y: float
    label: str = ""
    reached: bool = False

@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    # The complete system defines the initial forward direction as +Y.
    # Keep this aligned with the SLAM mapper, dashboard, and reset endpoint.
    heading_deg: float = 90.0

class OdometryEstimator:
    def __init__(self, config: WaypointConfig) -> None:
        self.config: WaypointConfig = config
        self.pose: Pose2D = Pose2D(heading_deg=90.0)
        self._last_update: float = time.perf_counter()

    def reset(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 90.0) -> None:
        self.pose = Pose2D(x=x, y=y, heading_deg=heading_deg)
        self._last_update = time.perf_counter()

    def update_motion(self, throttle: float, steering: float, dt: Optional[float] = None) -> Pose2D:
        now = time.perf_counter()
        if dt is None:
            dt = now - self._last_update
        self._last_update = now

        if dt <= 0 or (abs(throttle) < 0.05 and abs(steering) < 0.05):
            return self.pose

        # Proportional Steering update:
        # steering < 0 turns LEFT (+deg in standard frame), steering > 0 turns RIGHT (-deg)
        if abs(steering) >= 0.06:
            turn_rate = getattr(self.config, 'turn_rate_deg_s', 45.0)
            self.pose.heading_deg = (self.pose.heading_deg - (steering * turn_rate * dt)) % 360.0

        # Proportional Forward/Backward translation:
        if abs(throttle) >= 0.06:
            nominal_speed = getattr(self.config, 'nominal_speed_cm_s', 25.0)
            dist = throttle * nominal_speed * dt
            rad = math.radians(self.pose.heading_deg)
            self.pose.x += dist * math.cos(rad)
            self.pose.y += dist * math.sin(rad)

        return self.pose

    def update(self, command: Optional[VehicleCommand], dt: Optional[float] = None) -> Pose2D:
        if command is None or command == VehicleCommand.STOP:
            now = time.perf_counter()
            self._last_update = now
            return self.pose

        if command == VehicleCommand.FORWARD:
            return self.update_motion(1.0, 0.0, dt)
        elif command == VehicleCommand.BACKWARD:
            return self.update_motion(-1.0, 0.0, dt)
        elif command == VehicleCommand.LEFT:
            return self.update_motion(0.0, -1.0, dt)
        elif command == VehicleCommand.RIGHT:
            return self.update_motion(0.0, 1.0, dt)
        return self.pose

class WaypointNavigator:
    def __init__(self, config: WaypointConfig) -> None:
        self.config: WaypointConfig = config
        self.odometry: OdometryEstimator = OdometryEstimator(config)
        self.waypoints: List[Waypoint] = []
        self._current_index: int = 0
        self._next_id: int = 1

    def add_waypoint(self, x: float, y: float, label: str = "") -> Waypoint:
        wp = Waypoint(
            id=self._next_id,
            x=x,
            y=y,
            label=label if label else f"Point {self._next_id}",
            reached=False
        )
        self._next_id += 1
        self.waypoints.append(wp)
        car_logger.info(f"[WAYPOINT PLOTTED]: ID={wp.id} at ({wp.x:.1f}, {wp.y:.1f})")
        return wp

    def clear_waypoints(self) -> None:
        self.waypoints.clear()
        self._current_index = 0
        self._next_id = 1
        car_logger.info("All waypoints cleared.")

    def get_active_target(self) -> Optional[Waypoint]:
        if 0 <= self._current_index < len(self.waypoints):
            return self.waypoints[self._current_index]
        return None

    def compute_steering_to_target(self) -> Tuple[float, float, bool]:
        target = self.get_active_target()
        if target is None:
            return (0.0, 0.0, True)

        pose = self.odometry.pose
        dx = target.x - pose.x
        dy = target.y - pose.y
        dist = math.hypot(dx, dy)

        if dist <= self.config.arrival_tolerance_cm:
            target.reached = True
            car_logger.info(f"[WAYPOINT REACHED]: {target.label} at ({target.x:.1f}, {target.y:.1f})")
            self._current_index += 1
            return (0.0, dist, True)

        target_heading = math.degrees(math.atan2(dy, dx)) % 360.0
        angle_error = target_heading - pose.heading_deg
        while angle_error > 180.0:
            angle_error -= 360.0
        while angle_error < -180.0:
            angle_error += 360.0

        # Restriction: Maximum turn is 90° from center to the left (+90°) or right (-90°)
        angle_error = max(-90.0, min(90.0, angle_error))

        return (angle_error, dist, False)

    def set_3d_destination_from_camera(self, u: int, v: int, servo_angle_deg: float = 90.0) -> Tuple[Waypoint, Dict[str, float]]:
        """
        Projects tapped camera pixel (u, v) into 3D world ground coordinate (X_w, Y_w)
        anchored in the global odometry frame, accounting for camera pan servo angle.
        """
        pose = self.odometry.pose
        X_w, Y_w, X_rel, Y_rel = pixel_to_world(
            u=u, v=v,
            car_x=pose.x, car_y=pose.y, heading_deg=pose.heading_deg,
            servo_angle_deg=servo_angle_deg,
            cam_height_cm=getattr(self.config, 'cam_height_cm', 12.0),
            cam_pitch_deg=getattr(self.config, 'cam_pitch_deg', 15.0),
            fov_h_deg=getattr(self.config, 'fov_h_deg', 62.2),
            fov_v_deg=getattr(self.config, 'fov_v_deg', 48.8),
        )
        self.clear_waypoints()
        wp = self.add_waypoint(X_w, Y_w, label=f"3D Target ({X_w:.0f},{Y_w:.0f})")
        info = {
            "world_x": X_w,
            "world_y": Y_w,
            "rel_forward_cm": Y_rel,
            "rel_lateral_cm": X_rel,
            "distance_cm": round(math.hypot(X_rel, Y_rel), 1),
            "initial_pixel_x": u,
            "initial_pixel_y": v,
            "servo_angle_deg": round(servo_angle_deg, 1),
        }
        return wp, info

    def get_reprojected_pixel(self, servo_angle_deg: float = 90.0) -> Optional[Tuple[int, int, float, float]]:
        """
        Reprojects active destination waypoint onto current camera view.
        Returns (u, v, dist_cm, bearing_deg) or None if behind camera.
        """
        target = self.get_active_target()
        if target is None:
            return None
        pose = self.odometry.pose
        return world_to_pixel(
            X_w=target.x, Y_w=target.y,
            car_x=pose.x, car_y=pose.y, heading_deg=pose.heading_deg,
            servo_angle_deg=servo_angle_deg,
            cam_height_cm=getattr(self.config, 'cam_height_cm', 12.0),
            cam_pitch_deg=getattr(self.config, 'cam_pitch_deg', 15.0),
            fov_h_deg=getattr(self.config, 'fov_h_deg', 62.2),
            fov_v_deg=getattr(self.config, 'fov_v_deg', 48.8),
        )


def pixel_to_world(
    u: int, v: int,
    car_x: float, car_y: float, heading_deg: float,
    servo_angle_deg: float = 90.0,
    img_w: int = 640, img_h: int = 480,
    cam_height_cm: float = 12.0,
    cam_pitch_deg: float = 15.0,
    fov_h_deg: float = 62.2,
    fov_v_deg: float = 48.8,
) -> Tuple[float, float, float, float]:
    """
    Inverse perspective mapping: Projects camera pixel (u, v) onto floor plane (Z=0).
    Accounts for camera pan servo orientation relative to vehicle chassis.
    Returns (X_world, Y_world, X_rel, Y_rel) in centimeters.
    """
    cx = img_w / 2.0
    cy = img_h / 2.0

    # Vertical ray depression angle below horizon
    phi_v = ((v - cy) / cy) * (fov_v_deg / 2.0)
    alpha = cam_pitch_deg + phi_v

    min_alpha = 3.5  # degrees
    if alpha < min_alpha:
        forward_dist_cm = 250.0
    else:
        forward_dist_cm = cam_height_cm / math.tan(math.radians(alpha))
        forward_dist_cm = max(15.0, min(350.0, forward_dist_cm))

    # Horizontal ray yaw angle relative to camera optical axis
    psi_h = ((u - cx) / cx) * (fov_h_deg / 2.0)
    lateral_dist_cm = forward_dist_cm * math.tan(math.radians(psi_h))

    # Camera optical axis heading in world coordinates:
    # 90° servo is straight ahead; >90° is left (+); <90° is right (-)
    cam_yaw_deg = heading_deg + (servo_angle_deg - 90.0)
    cam_rad = math.radians(cam_yaw_deg)

    # Global odometry frame (0° = +X, 90° = +Y):
    # Camera forward unit vector: (cos(cam_rad), sin(cam_rad))
    # Camera right lateral vector: (sin(cam_rad), -cos(cam_rad))
    X_w = car_x + (forward_dist_cm * math.cos(cam_rad)) + (lateral_dist_cm * math.sin(cam_rad))
    Y_w = car_y + (forward_dist_cm * math.sin(cam_rad)) - (lateral_dist_cm * math.cos(cam_rad))

    # Relative to vehicle body frame:
    car_rad = math.radians(heading_deg)
    dx = X_w - car_x
    dy = Y_w - car_y
    Y_rel = (dx * math.cos(car_rad)) + (dy * math.sin(car_rad))
    X_rel = (dx * math.sin(car_rad)) - (dy * math.cos(car_rad))

    return (round(X_w, 1), round(Y_w, 1), round(X_rel, 1), round(Y_rel, 1))


def world_to_pixel(
    X_w: float, Y_w: float,
    car_x: float, car_y: float, heading_deg: float,
    servo_angle_deg: float = 90.0,
    img_w: int = 640, img_h: int = 480,
    cam_height_cm: float = 12.0,
    cam_pitch_deg: float = 15.0,
    fov_h_deg: float = 62.2,
    fov_v_deg: float = 48.8,
) -> Optional[Tuple[int, int, float, float]]:
    """
    Forward projection: Projects 3D world coordinate (X_w, Y_w) onto camera image (u, v)
    accounting for both vehicle pose and camera pan servo angle.
    Returns (u, v, distance_cm, bearing_deg) or None if point is behind camera optical axis.
    """
    cam_yaw_deg = heading_deg + (servo_angle_deg - 90.0)
    cam_rad = math.radians(cam_yaw_deg)
    dx = X_w - car_x
    dy = Y_w - car_y

    # Relative to camera optical axis:
    # Forward along optical axis:
    Y_cam = (dx * math.cos(cam_rad)) + (dy * math.sin(cam_rad))
    # Right lateral perpendicular to optical axis:
    X_cam = (dx * math.sin(cam_rad)) - (dy * math.cos(cam_rad))

    dist_cm = math.hypot(dx, dy)
    bearing_deg = math.degrees(math.atan2(X_cam, Y_cam))

    if Y_cam <= 2.0:
        return None

    cx = img_w / 2.0
    cy = img_h / 2.0

    psi_h = bearing_deg
    u = cx + (psi_h / (fov_h_deg / 2.0)) * cx

    alpha = math.degrees(math.atan2(cam_height_cm, Y_cam))
    phi_v = alpha - cam_pitch_deg
    v = cy + (phi_v / (fov_v_deg / 2.0)) * cy

    return (int(round(u)), int(round(v)), round(dist_cm, 1), round(bearing_deg, 1))
