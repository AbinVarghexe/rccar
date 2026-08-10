"""
Waypoint Navigation and 2D Dead-Reckoning Odometry module.
"""

import os
import sys
import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

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
    heading_deg: float = 0.0

class OdometryEstimator:
    def __init__(self, config: WaypointConfig) -> None:
        self.config: WaypointConfig = config
        self.pose: Pose2D = Pose2D()
        self._last_update: float = time.perf_counter()

    def reset(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 90.0) -> None:
        self.pose = Pose2D(x=x, y=y, heading_deg=heading_deg)
        self._last_update = time.perf_counter()

    def update(self, command: Optional[VehicleCommand], dt: Optional[float] = None) -> Pose2D:
        now = time.perf_counter()
        if dt is None:
            dt = now - self._last_update
        self._last_update = now

        if dt <= 0 or command is None or command == VehicleCommand.STOP:
            return self.pose

        rad = math.radians(self.pose.heading_deg)

        if command == VehicleCommand.FORWARD:
            dist = self.config.nominal_speed_cm_s * dt
            self.pose.x += dist * math.cos(rad)
            self.pose.y += dist * math.sin(rad)

        elif command == VehicleCommand.BACKWARD:
            dist = self.config.nominal_speed_cm_s * dt
            self.pose.x -= dist * math.cos(rad)
            self.pose.y -= dist * math.sin(rad)

        elif command == VehicleCommand.LEFT:
            self.pose.heading_deg = (self.pose.heading_deg + self.config.turn_rate_deg_s * dt) % 360.0

        elif command == VehicleCommand.RIGHT:
            self.pose.heading_deg = (self.pose.heading_deg - self.config.turn_rate_deg_s * dt) % 360.0

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

        return (angle_error, dist, False)
