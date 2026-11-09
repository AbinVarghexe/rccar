"""
slam/slam_navigation_bridge.py - Bridge connecting Visual SLAM with Navigation & Safety Pipeline.

Fulfills Step 3:
1. Fuses Visual SLAM poses with Kinematic Dead-Reckoning (WaypointNavigator).
2. Feeds visual landmarks into local navigation planner.
3. Enforces strict independence of the Safety Controller / Safety Supervisor:
   Safety supervisor preserves 100% authority to override or halt motors when
   physical obstacles (sonar < 8cm or close range hazards) are detected.
4. Exposes thread-safe telemetry and map data for dashboard visualization.
"""

import math
import time
import threading
from typing import Optional, Dict, Any, List, Tuple

import numpy as np

from .orb_visual_odometry import ORBVisualOdometry, VisualOdometryConfig
from .orb_slam_mapper import ORBSLAMMapper, SLAMTrackingResult


class SLAMNavigationBridge:
    """
    Manages the lifecycle of Visual SLAM and coordinates data exchange between
    camera perception, dead-reckoning odometry, and navigation planning.
    """

    def __init__(
        self,
        slam_mapper: Optional[ORBSLAMMapper] = None,
        fusion_weight_slam: float = 0.35,  # Complementary filter gain for visual SLAM corrections
    ):
        self.mapper = slam_mapper or ORBSLAMMapper()
        self.fusion_weight_slam = fusion_weight_slam
        self._lock = threading.RLock()

        # Fused Robot State
        self.fused_x_cm: float = 0.0
        self.fused_y_cm: float = 0.0
        self.fused_heading_deg: float = 90.0
        self.tracking_health: str = "INITIALIZING"
        self.total_frames_processed: int = 0
        self.last_update_time: float = 0.0
        self._last_result = SLAMTrackingResult()

    def update(
        self,
        bgr_frame: np.ndarray,
        kinematic_x_cm: float,
        kinematic_y_cm: float,
        kinematic_heading_deg: float,
        kinematic_speed_cm_s: float,
        dt: float,
        sonar_depth_cm: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        Updates Visual SLAM with the latest camera frame and fuses the estimated
        pose with the vehicle's kinematic odometry.
        """
        now = time.time()
        self.total_frames_processed += 1

        # Run Visual SLAM pipeline
        slam_res: SLAMTrackingResult = self.mapper.process_frame(
            bgr_frame=bgr_frame,
            timestamp=now,
            kinematic_speed_hint_cm_s=kinematic_speed_cm_s,
            dt=dt,
            sonar_depth_hint_cm=sonar_depth_cm,
        )

        with self._lock:
            self._last_result = slam_res
            self.last_update_time = now

            if slam_res.tracking_mode == "MAP_TRACKING":
                # High confidence map tracking: apply weighted fusion to correct dead-reckoning drift
                w = self.fusion_weight_slam
                self.fused_x_cm = (1.0 - w) * kinematic_x_cm + w * slam_res.pose_x_cm
                self.fused_y_cm = (1.0 - w) * kinematic_y_cm + w * slam_res.pose_y_cm

                # Angle circular fusion
                h_diff = (slam_res.heading_deg - kinematic_heading_deg + 180.0) % 360.0 - 180.0
                self.fused_heading_deg = (kinematic_heading_deg + w * h_diff) % 360.0
                self.tracking_health = "LOCKED"

            elif slam_res.tracking_mode == "VO_ODOMETRY":
                # Inter-frame VO active: moderate confidence
                w = self.fusion_weight_slam * 0.5
                self.fused_x_cm = (1.0 - w) * kinematic_x_cm + w * slam_res.pose_x_cm
                self.fused_y_cm = (1.0 - w) * kinematic_y_cm + w * slam_res.pose_y_cm
                self.fused_heading_deg = kinematic_heading_deg
                self.tracking_health = "TRACKING"

            else:
                # LOST / DEGRADED (textureless floor, dark lighting, fast motion blur):
                # Gracefully fall back 100% to kinematic dead-reckoning to guarantee uninterrupted navigation
                self.fused_x_cm = kinematic_x_cm
                self.fused_y_cm = kinematic_y_cm
                self.fused_heading_deg = kinematic_heading_deg
                self.tracking_health = "DEGRADED" if slam_res.tracking_mode == "DEGRADED" else "LOST"

            return self.get_telemetry()

    def get_fused_pose(self) -> Tuple[float, float, float]:
        """Returns the current fused robot pose (x_cm, y_cm, heading_deg)."""
        with self._lock:
            return (self.fused_x_cm, self.fused_y_cm, self.fused_heading_deg)

    def check_landmark_in_corridor(
        self,
        corridor_width_cm: float = 35.0,
        lookahead_distance_cm: float = 65.0,
    ) -> Tuple[bool, Optional[Tuple[float, float]]]:
        """
        Inspects the persistent visual SLAM map to check if any triangulated 3D
        landmarks are located directly inside the vehicle's forward driving corridor.
        """
        with self._lock:
            rx, ry, r_heading = self.fused_x_cm, self.fused_y_cm, self.fused_heading_deg
            rad = math.radians(r_heading)
            fwd_x = math.cos(rad)
            fwd_y = math.sin(rad)
            lat_x = -math.sin(rad)
            lat_y = math.cos(rad)

            for mp in self.mapper.map_points.values():
                if mp.observations_count < 2:
                    continue
                # Transform landmark to robot body frame
                dx = mp.world_pos[0] - rx
                dy = mp.world_pos[1] - ry

                dist_fwd = dx * fwd_x + dy * fwd_y
                dist_lat = abs(dx * lat_x + dy * lat_y)

                if 5.0 <= dist_fwd <= lookahead_distance_cm and dist_lat <= (corridor_width_cm / 2.0):
                    return True, (float(mp.world_pos[0]), float(mp.world_pos[1]))

            return False, None

    def get_telemetry(self) -> Dict[str, Any]:
        """Returns thread-safe SLAM telemetry for Web Dashboard and APIs."""
        with self._lock:
            res = self._last_result
            return {
                "fused_pose": {
                    "x_cm": round(float(self.fused_x_cm), 1),
                    "y_cm": round(float(self.fused_y_cm), 1),
                    "heading_deg": round(float(self.fused_heading_deg), 1),
                },
                "slam_pose": {
                    "x_cm": res.pose_x_cm,
                    "y_cm": res.pose_y_cm,
                    "heading_deg": res.heading_deg,
                },
                "tracking_health": self.tracking_health,
                "tracking_mode": res.tracking_mode,
                "map_points_count": res.num_map_points,
                "tracked_points_count": res.num_tracked_points,
                "keyframes_count": len(self.mapper.keyframes),
                "map_quality": res.map_quality,
                "landmarks_count": len(res.landmarks_2d),
                "landmarks_sample": res.landmarks_2d[:60],
            }

    def reset(self) -> None:
        """Resets the SLAM mapper and fused states."""
        with self._lock:
            self.mapper.reset_map()
            self.fused_x_cm = 0.0
            self.fused_y_cm = 0.0
            self.fused_heading_deg = 90.0
            self.tracking_health = "INITIALIZING"
            self.total_frames_processed = 0
            self._last_result = SLAMTrackingResult()
