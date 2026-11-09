"""
slam/orb_slam_mapper.py - Keyframe-Based Visual SLAM & Sparse Metric Mapping.
Inspired by ORB_SLAM3's Local Mapping & Tracking Components.

Responsibilities:
1. Keyframe Decision Logic: Spawns new keyframes based on parallax, distance, or inlier degradation.
2. 3D MapPoint Triangulation: Triangulates landmark features into metric 3D space (X, Y, Z in cm).
3. Map-Based Pose Tracking (PnP): Localizes the vehicle against the persistent visual landmark map
   using Perspective-n-Point with RANSAC (cv2.solvePnPRansac) to eliminate cumulative odometry drift.
4. 2D Navigational Occupancy Projection: Projects triangulated 3D visual landmarks onto a 2D top-down
   grid for path planning and obstacle avoidance.
5. Loop Closure / Relocalization Candidates: Maintains a database of keyframe descriptors to detect
   when the robot returns to a previously visited area.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Tuple, Set, Any

import cv2
import numpy as np

from .orb_visual_odometry import ORBVisualOdometry, VisualMotionEstimate, VisualOdometryConfig


@dataclass
class MapPoint:
    """3D Landmark point in the persistent visual SLAM map."""
    id: int
    world_pos: np.ndarray        # (3,) [X, Y, Z] in cm
    descriptor: np.ndarray       # (32,) uint8 ORB descriptor
    observed_keyframe_ids: Set[int] = field(default_factory=set)
    observations_count: int = 1
    last_seen_timestamp: float = 0.0


@dataclass
class KeyFrame:
    """Keyframe containing camera pose, visual features, and landmark associations."""
    id: int
    timestamp: float
    world_pose_x_cm: float
    world_pose_y_cm: float
    heading_deg: float
    R_world_cam: np.ndarray      # 3x3 Rotation matrix
    t_world_cam: np.ndarray      # 3x1 Translation vector in cm
    keypoints: List[cv2.KeyPoint]
    descriptors: np.ndarray
    pts2d: np.ndarray            # (N, 2)
    map_point_ids: Dict[int, int] = field(default_factory=dict)  # feature_idx -> map_point_id


@dataclass
class SLAMTrackingResult:
    """Result of full visual SLAM tracking step."""
    pose_x_cm: float = 0.0
    pose_y_cm: float = 0.0
    heading_deg: float = 90.0
    is_keyframe: bool = False
    num_map_points: int = 0
    num_tracked_points: int = 0
    tracking_mode: str = "INITIALIZING"  # "MAP_TRACKING", "VO_ODOMETRY", "LOST"
    map_quality: float = 0.0
    landmarks_2d: List[Tuple[float, float]] = field(default_factory=list)


class ORBSLAMMapper:
    """
    Lightweight Keyframe Visual SLAM & Local Mapping Engine for Raspberry Pi.
    """

    def __init__(self, vo: Optional[ORBVisualOdometry] = None):
        self.vo = vo or ORBVisualOdometry()
        self.K = self.vo.K

        # Keyframe thresholds
        self.keyframe_min_dist_cm: float = 12.0
        self.keyframe_min_angle_deg: float = 12.0
        self.keyframe_min_inlier_ratio: float = 0.65

        # Persistent Map
        self.keyframes: Dict[int, KeyFrame] = {}
        self.map_points: Dict[int, MapPoint] = {}
        self._next_kf_id: int = 0
        self._next_mp_id: int = 0

        # Current Robot Pose in World Map Frame
        self.cur_x_cm: float = 0.0
        self.cur_y_cm: float = 0.0
        self.cur_heading_deg: float = 90.0  # Initial heading facing +Y (North)

        self._last_kf_x: float = 0.0
        self._last_kf_y: float = 0.0
        self._last_kf_heading: float = 90.0

        self.last_result = SLAMTrackingResult()

    def process_frame(
        self,
        bgr_frame: np.ndarray,
        timestamp: float,
        kinematic_speed_hint_cm_s: Optional[float] = None,
        dt: Optional[float] = None,
        sonar_depth_hint_cm: Optional[float] = None,
    ) -> SLAMTrackingResult:
        """
        Processes an incoming frame through the Visual SLAM pipeline:
        1. Estimates inter-frame motion via Visual Odometry.
        2. Localizes against existing 3D map points via PnP (if map populated).
        3. Decides whether to insert a KeyFrame.
        4. Triangulates new 3D map points when a KeyFrame is created.
        """
        if bgr_frame is None or bgr_frame.size == 0:
            self.last_result = SLAMTrackingResult(
                pose_x_cm=self.cur_x_cm,
                pose_y_cm=self.cur_y_cm,
                heading_deg=self.cur_heading_deg,
                tracking_mode="LOST",
                num_map_points=len(self.map_points),
            )
            return self.last_result

        # Step 1: Inter-frame Visual Odometry
        vo_est: VisualMotionEstimate = self.vo.estimate_motion(
            curr_bgr_frame=bgr_frame,
            timestamp=timestamp,
            kinematic_speed_hint_cm_s=kinematic_speed_hint_cm_s,
            dt=dt,
            sonar_depth_hint_cm=sonar_depth_hint_cm,
        )

        tracking_mode = "VO_ODOMETRY"

        if vo_est.tracking_state == "OK":
            # Integrate relative planar motion into dead-reckoning pose
            rad = math.radians(self.cur_heading_deg)
            # Body delta forward (delta_x) and lateral (delta_y)
            dx_world = vo_est.delta_x_cm * math.cos(rad) - vo_est.delta_y_cm * math.sin(rad)
            dy_world = vo_est.delta_x_cm * math.sin(rad) + vo_est.delta_y_cm * math.cos(rad)

            self.cur_x_cm += dx_world
            self.cur_y_cm += dy_world
            self.cur_heading_deg = (self.cur_heading_deg + vo_est.delta_heading_deg) % 360.0

        elif vo_est.tracking_state in ("DEGRADED", "LOST"):
            # Fallback to kinematic dead-reckoning integration if provided
            if kinematic_speed_hint_cm_s is not None and dt is not None and dt > 0:
                dist = kinematic_speed_hint_cm_s * dt
                rad = math.radians(self.cur_heading_deg)
                self.cur_x_cm += dist * math.cos(rad)
                self.cur_y_cm += dist * math.sin(rad)
            tracking_mode = "LOST" if vo_est.tracking_state == "LOST" else "DEGRADED"

        # Step 2: Map-Based Tracking via PnP (if map has points and descriptors)
        num_tracked = 0
        pnp_success, pnp_x, pnp_y, pnp_heading, num_tracked = self._track_local_map(bgr_frame)
        if pnp_success:
            # Fused map localization corrects cumulative drift
            self.cur_x_cm = 0.85 * pnp_x + 0.15 * self.cur_x_cm
            self.cur_y_cm = 0.85 * pnp_y + 0.15 * self.cur_y_cm
            self.cur_heading_deg = (0.85 * pnp_heading + 0.15 * self.cur_heading_deg) % 360.0
            tracking_mode = "MAP_TRACKING"

        # Step 3: Keyframe Decision Logic
        is_keyframe = False
        dist_from_last_kf = math.hypot(self.cur_x_cm - self._last_kf_x, self.cur_y_cm - self._last_kf_y)
        heading_diff = abs(self.cur_heading_deg - self._last_kf_heading)
        if heading_diff > 180.0:
            heading_diff = 360.0 - heading_diff

        should_create_kf = (
            len(self.keyframes) == 0 or
            dist_from_last_kf >= self.keyframe_min_dist_cm or
            heading_diff >= self.keyframe_min_angle_deg or
            (vo_est.inlier_ratio < self.keyframe_min_inlier_ratio and len(vo_est.matched_curr_pts) >= 20)
        )

        if should_create_kf and vo_est.tracking_state in ("OK", "INITIALIZING"):
            new_kf = self._create_keyframe(bgr_frame, timestamp)
            if new_kf is not None:
                is_keyframe = True
                self._triangulate_new_map_points(new_kf, vo_est)
                self._last_kf_x = self.cur_x_cm
                self._last_kf_y = self.cur_y_cm
                self._last_kf_heading = self.cur_heading_deg

        # Step 4: Extract 2D top-down coordinates of visual map points for navigation
        landmarks_2d = [
            (round(float(mp.world_pos[0]), 1), round(float(mp.world_pos[1]), 1))
            for mp in self.map_points.values()
            if mp.observations_count >= 2
        ]

        map_quality = min(1.0, len(self.map_points) / 100.0)

        self.last_result = SLAMTrackingResult(
            pose_x_cm=round(float(self.cur_x_cm), 1),
            pose_y_cm=round(float(self.cur_y_cm), 1),
            heading_deg=round(float(self.cur_heading_deg), 1),
            is_keyframe=is_keyframe,
            num_map_points=len(self.map_points),
            num_tracked_points=num_tracked,
            tracking_mode=tracking_mode,
            map_quality=round(float(map_quality), 2),
            landmarks_2d=landmarks_2d[:150]  # Cap payload size for UI
        )
        return self.last_result

    def _create_keyframe(self, bgr_frame: np.ndarray, timestamp: float) -> Optional[KeyFrame]:
        """Extracts features and packages keyframe with world pose matrix."""
        gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY) if len(bgr_frame.shape) == 3 else bgr_frame
        features = self.vo.extract_features(gray, timestamp)
        if features.descriptors is None or len(features.keypoints) < 15:
            return None

        # Build 3D rotation and translation in world coordinates
        rad = math.radians(self.cur_heading_deg)
        R_world = np.array([
            [math.cos(rad), -math.sin(rad), 0.0],
            [math.sin(rad),  math.cos(rad), 0.0],
            [0.0,            0.0,           1.0]
        ], dtype=np.float64)

        t_world = np.array([[self.cur_x_cm], [self.cur_y_cm], [self.vo.config.camera_height_cm]], dtype=np.float64)

        kf = KeyFrame(
            id=self._next_kf_id,
            timestamp=timestamp,
            world_pose_x_cm=self.cur_x_cm,
            world_pose_y_cm=self.cur_y_cm,
            heading_deg=self.cur_heading_deg,
            R_world_cam=R_world,
            t_world_cam=t_world,
            keypoints=features.keypoints,
            descriptors=features.descriptors,
            pts2d=features.pts2d
        )
        self.keyframes[kf.id] = kf
        self._next_kf_id += 1
        return kf

    def _triangulate_new_map_points(self, curr_kf: KeyFrame, vo_est: VisualMotionEstimate) -> int:
        """
        Triangulates matched inlier features between previous and current keyframe into 3D world coordinates.
        """
        if len(self.keyframes) < 2:
            return 0

        prev_kf = list(self.keyframes.values())[-2]

        if prev_kf.descriptors is None or curr_kf.descriptors is None:
            return 0

        # Match descriptors between the two keyframes
        try:
            knn = self.vo.matcher.knnMatch(prev_kf.descriptors, curr_kf.descriptors, k=2)
        except Exception:
            return 0

        good_matches = []
        for pair in knn:
            if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance:
                good_matches.append(pair[0])

        if len(good_matches) < 10:
            return 0

        pts_prev = np.float32([prev_kf.keypoints[m.queryIdx].pt for m in good_matches])
        pts_curr = np.float32([curr_kf.keypoints[m.trainIdx].pt for m in good_matches])

        # Projection matrices: P = K [R | t]
        R1 = prev_kf.R_world_cam.T
        t1 = -R1 @ prev_kf.t_world_cam
        P1 = self.K @ np.hstack((R1, t1))

        R2 = curr_kf.R_world_cam.T
        t2 = -R2 @ curr_kf.t_world_cam
        P2 = self.K @ np.hstack((R2, t2))

        # Triangulate 3D points
        pts4d = cv2.triangulatePoints(P1, P2, pts_prev.T, pts_curr.T)
        pts3d = pts4d[:3] / np.maximum(1e-7, pts4d[3])  # Normalize homogenous coordinates

        created_count = 0
        for i in range(pts3d.shape[1]):
            wx, wy, wz = pts3d[:, i]
            # Validate 3D depth limits (10cm to 600cm from vehicle)
            dist_from_cam = math.hypot(wx - curr_kf.world_pose_x_cm, wy - curr_kf.world_pose_y_cm)
            if 10.0 <= dist_from_cam <= 600.0:
                mp_id = self._next_mp_id
                self._next_mp_id += 1

                mp = MapPoint(
                    id=mp_id,
                    world_pos=np.array([wx, wy, wz], dtype=np.float64),
                    descriptor=curr_kf.descriptors[good_matches[i].trainIdx].copy(),
                    observed_keyframe_ids={prev_kf.id, curr_kf.id},
                    observations_count=2,
                    last_seen_timestamp=curr_kf.timestamp
                )
                self.map_points[mp_id] = mp
                curr_kf.map_point_ids[good_matches[i].trainIdx] = mp_id
                prev_kf.map_point_ids[good_matches[i].queryIdx] = mp_id
                created_count += 1

        # Keep map memory bounded for Raspberry Pi (cap at 600 active map points)
        if len(self.map_points) > 600:
            self._prune_distant_map_points()

        return created_count

    def _track_local_map(self, bgr_frame: np.ndarray) -> Tuple[bool, float, float, float, int]:
        """
        Matches incoming frame against persistent 3D MapPoints and estimates pose using PnP with RANSAC.
        """
        if len(self.map_points) < 15:
            return False, 0.0, 0.0, 0.0, 0

        gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY) if len(bgr_frame.shape) == 3 else bgr_frame
        kps, descs = self.vo.orb.detectAndCompute(gray, None)
        if descs is None or len(kps) < 10:
            return False, 0.0, 0.0, 0.0, 0

        # Stack local map descriptors
        mp_list = [mp for mp in self.map_points.values() if mp.observations_count >= 2]
        if len(mp_list) < 10:
            return False, 0.0, 0.0, 0.0, 0

        map_descs = np.vstack([mp.descriptor for mp in mp_list])

        try:
            knn = self.vo.matcher.knnMatch(descs, map_descs, k=2)
        except Exception:
            return False, 0.0, 0.0, 0.0, 0

        obj_pts = []
        img_pts = []
        for pair in knn:
            if len(pair) == 2 and pair[0].distance < 0.72 * pair[1].distance:
                frame_idx = pair[0].queryIdx
                map_idx = pair[0].trainIdx
                obj_pts.append(mp_list[map_idx].world_pos)
                img_pts.append(kps[frame_idx].pt)

        if len(obj_pts) < 8:
            return False, 0.0, 0.0, 0.0, 0

        obj_pts = np.float32(obj_pts)
        img_pts = np.float32(img_pts)

        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts,
            img_pts,
            self.K,
            distCoeffs=None,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=3.0,
            confidence=0.99
        )

        if not success or inliers is None or len(inliers) < 6:
            return False, 0.0, 0.0, 0.0, 0

        R_cam, _ = cv2.Rodrigues(rvec)
        # World position from camera translation: t_world = -R_cam^T * tvec
        R_world = R_cam.T
        t_world = -R_world @ tvec

        pnp_x = float(t_world[0, 0])
        pnp_y = float(t_world[1, 0])

        yaw_rad = math.atan2(R_world[1, 0], R_world[0, 0])
        pnp_heading = math.degrees(yaw_rad) % 360.0

        return True, pnp_x, pnp_y, pnp_heading, len(inliers)

    def _prune_distant_map_points(self) -> None:
        """Removes distant or unobserved landmarks to maintain fast CPU cache performance."""
        keep_ids = set()
        for mp_id, mp in sorted(
            self.map_points.items(),
            key=lambda item: (item[1].observations_count, -math.hypot(item[1].world_pos[0] - self.cur_x_cm, item[1].world_pos[1] - self.cur_y_cm)),
            reverse=True
        )[:400]:
            keep_ids.add(mp_id)

        self.map_points = {k: v for k, v in self.map_points.items() if k in keep_ids}

    def reset_map(self) -> None:
        """Clears all keyframes and map points, resetting the SLAM state."""
        self.keyframes.clear()
        self.map_points.clear()
        self._next_kf_id = 0
        self._next_mp_id = 0
        self.cur_x_cm = 0.0
        self.cur_y_cm = 0.0
        self.cur_heading_deg = 90.0
        self._last_kf_x = 0.0
        self._last_kf_y = 0.0
        self._last_kf_heading = 90.0
        self.vo.reset()
        self.last_result = SLAMTrackingResult()
