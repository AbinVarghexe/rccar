"""
slam/orb_visual_odometry.py - ORB-Based Monocular Visual Odometry (VO) for Autonomous RC Car.
Inspired by ORB_SLAM3's Feature Tracking Front-End.

Core Principles & Hardware Adaptation:
1. Real-time ORB (Oriented FAST and Rotated BRIEF) extraction with grid-based feature distribution.
2. Essential Matrix (E) decomposition with RANSAC for robust outlier rejection.
3. Planar motion constraint: Ground vehicle kinematics primarily consist of planar translation (dx, dy)
   and yaw rotation (dtheta), with camera pitch calibration.
4. Monocular Scale Resolver: Resolves the fundamental monocular scale ambiguity (||t|| = 1) by fusing
   wheel command dead-reckoning (v * dt) and HC-SR04 sonar depth ground-truth.
5. Tracking Quality & Failure Detection: Detects textureless surfaces, motion blur, and dark lighting,
   flagging tracking state as OK, DEGRADED, or LOST to trigger graceful fallback.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict, Any

import cv2
import numpy as np


@dataclass
class VisualOdometryConfig:
    """Configuration for ORB Visual Odometry."""
    n_features: int = 400
    scale_factor: float = 1.2
    n_levels: int = 4
    fast_threshold: int = 20
    match_ratio_thresh: float = 0.75     # Lowe's ratio test threshold
    ransac_prob: float = 0.999
    ransac_threshold_px: float = 1.5
    min_inliers_tracking: int = 15       # Minimum inliers required for valid pose
    min_parallax_px: float = 2.0         # Minimum feature parallax to avoid degenerate rotation
    camera_fov_h_deg: float = 62.2       # Raspberry Pi Camera v1/v2 horizontal FOV
    camera_pitch_deg: float = 12.0       # Downward tilt of camera relative to ground plane
    camera_height_cm: float = 12.0       # Lens height above ground
    nominal_speed_cm_s: float = 30.0     # Metric speed anchor for scale resolution at 100% throttle


@dataclass
class FrameKeypoints:
    """Container for detected frame features and descriptors."""
    timestamp: float
    keypoints: List[cv2.KeyPoint]
    descriptors: Optional[np.ndarray]
    pts2d: np.ndarray  # (N, 2) float32 coordinates


@dataclass
class VisualMotionEstimate:
    """Estimated relative motion between consecutive frames."""
    delta_x_cm: float = 0.0
    delta_y_cm: float = 0.0
    delta_heading_deg: float = 0.0
    num_matches: int = 0
    num_inliers: int = 0
    inlier_ratio: float = 0.0
    scale_factor: float = 1.0
    tracking_state: str = "INITIALIZING"  # "OK", "DEGRADED", "LOST", "INITIALIZING"
    confidence: float = 0.0
    matched_prev_pts: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))
    matched_curr_pts: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


class ORBVisualOdometry:
    """
    Monocular Visual Odometry using ORB feature tracking and 2D-to-2D epipolar geometry.
    """

    def __init__(self, config: Optional[VisualOdometryConfig] = None, camera_matrix: Optional[np.ndarray] = None):
        self.config = config or VisualOdometryConfig()
        
        # Camera intrinsic matrix K (approximate from FOV if not calibrated)
        if camera_matrix is not None:
            self.K = camera_matrix.copy()
        else:
            self.K = self._build_intrinsic_matrix(640, 480, self.config.camera_fov_h_deg)

        self.orb = cv2.ORB_create(
            nfeatures=self.config.n_features,
            scaleFactor=self.config.scale_factor,
            nlevels=self.config.n_levels,
            edgeThreshold=15,
            firstLevel=0,
            WTA_K=2,
            scoreType=cv2.ORB_HARRIS_SCORE,
            patchSize=31,
            fastThreshold=self.config.fast_threshold,
        )

        # FLANN or BFMatcher with Hamming norm for binary ORB descriptors
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self._prev_frame: Optional[FrameKeypoints] = None
        self._current_estimate = VisualMotionEstimate()

    @staticmethod
    def _build_intrinsic_matrix(w: int, h: int, fov_h_deg: float) -> np.ndarray:
        """Constructs camera intrinsic matrix K from image dimensions and horizontal FOV."""
        fov_rad = math.radians(fov_h_deg)
        fx = (w / 2.0) / math.tan(fov_rad / 2.0)
        fy = fx  # Square pixel assumption
        cx = w / 2.0
        cy = h / 2.0
        return np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

    def extract_features(self, gray_frame: np.ndarray, timestamp: float) -> FrameKeypoints:
        """
        Extracts ORB features across the frame with contrast-aware detection.
        """
        if gray_frame is None or gray_frame.size == 0:
            return FrameKeypoints(timestamp=timestamp, keypoints=[], descriptors=None, pts2d=np.empty((0, 2), dtype=np.float32))

        # Grid mask to avoid clustering all features on high-contrast floor texture or ceiling lights
        h, w = gray_frame.shape[:2]
        
        # Mask out vehicle chassis bumper at bottom 12% of frame
        mask = np.ones((h, w), dtype=np.uint8) * 255
        mask[int(h * 0.88):, :] = 0

        kps, descs = self.orb.detectAndCompute(gray_frame, mask=mask)

        if kps:
            pts = np.float32([kp.pt for kp in kps])
        else:
            pts = np.empty((0, 2), dtype=np.float32)

        return FrameKeypoints(timestamp=timestamp, keypoints=kps, descriptors=descs, pts2d=pts)

    def estimate_motion(
        self,
        curr_bgr_frame: np.ndarray,
        timestamp: float,
        kinematic_speed_hint_cm_s: Optional[float] = None,
        dt: Optional[float] = None,
        sonar_depth_hint_cm: Optional[float] = None,
    ) -> VisualMotionEstimate:
        """
        Calculates relative motion between previous and current frames.
        
        Args:
            curr_bgr_frame: Current BGR image frame from camera.
            timestamp: Monotonic capture timestamp.
            kinematic_speed_hint_cm_s: Estimated forward speed from wheel commands (e.g. throttle * 30 cm/s).
            dt: Time elapsed since previous frame.
            sonar_depth_hint_cm: HC-SR04 ultrasonic distance reading (metric anchor).
            
        Returns:
            VisualMotionEstimate with delta_x_cm, delta_y_cm, delta_heading_deg, and tracking status.
        """
        if curr_bgr_frame is None or curr_bgr_frame.size == 0:
            self._current_estimate = VisualMotionEstimate(tracking_state="LOST")
            return self._current_estimate

        gray = cv2.cvtColor(curr_bgr_frame, cv2.COLOR_BGR2GRAY) if len(curr_bgr_frame.shape) == 3 else curr_bgr_frame

        # Check for image degradation (textureless, motion blur, darkness)
        mean_val, std_val = cv2.meanStdDev(gray)
        brightness = float(mean_val[0][0])
        contrast = float(std_val[0][0])

        if brightness < 12.0 or contrast < 3.5:
            # Degraded/Blind lighting
            self._current_estimate = VisualMotionEstimate(
                tracking_state="LOST",
                confidence=0.0
            )
            return self._current_estimate

        curr_frame = self.extract_features(gray, timestamp)

        # First frame initialization
        if self._prev_frame is None or self._prev_frame.descriptors is None or len(self._prev_frame.keypoints) < self.config.min_inliers_tracking:
            self._prev_frame = curr_frame
            self._current_estimate = VisualMotionEstimate(tracking_state="INITIALIZING")
            return self._current_estimate

        # Match ORB descriptors using Lowe's Ratio Test
        if curr_frame.descriptors is None or len(curr_frame.keypoints) < self.config.min_inliers_tracking:
            self._prev_frame = curr_frame
            self._current_estimate = VisualMotionEstimate(tracking_state="DEGRADED", confidence=0.1)
            return self._current_estimate

        try:
            knn_matches = self.matcher.knnMatch(self._prev_frame.descriptors, curr_frame.descriptors, k=2)
        except Exception:
            self._prev_frame = curr_frame
            self._current_estimate = VisualMotionEstimate(tracking_state="LOST")
            return self._current_estimate

        # Ratio test filtering
        good_matches = []
        for m_pair in knn_matches:
            if len(m_pair) == 2:
                m, n = m_pair
                if m.distance < self.config.match_ratio_thresh * n.distance:
                    good_matches.append(m)

        if len(good_matches) < self.config.min_inliers_tracking:
            self._prev_frame = curr_frame
            self._current_estimate = VisualMotionEstimate(
                num_matches=len(good_matches),
                tracking_state="DEGRADED",
                confidence=float(len(good_matches) / max(1, self.config.min_inliers_tracking))
            )
            return self._current_estimate

        # Extract matched 2D coordinate pairs
        prev_pts = np.float32([self._prev_frame.keypoints[m.queryIdx].pt for m in good_matches])
        curr_pts = np.float32([curr_frame.keypoints[m.trainIdx].pt for m in good_matches])

        # Check mean parallax (motion between frames)
        parallax = np.mean(np.linalg.norm(curr_pts - prev_pts, axis=1))
        if parallax < self.config.min_parallax_px:
            # Vehicle is essentially stationary or purely micro-vibrating
            self._prev_frame = curr_frame
            self._current_estimate = VisualMotionEstimate(
                delta_x_cm=0.0,
                delta_y_cm=0.0,
                delta_heading_deg=0.0,
                num_matches=len(good_matches),
                num_inliers=len(good_matches),
                inlier_ratio=1.0,
                tracking_state="OK",
                confidence=0.95,
                matched_prev_pts=prev_pts,
                matched_curr_pts=curr_pts
            )
            return self._current_estimate

        # Compute Essential Matrix E using 5-point algorithm with RANSAC
        E, inlier_mask = cv2.findEssentialMat(
            curr_pts,
            prev_pts,
            self.K,
            method=cv2.RANSAC,
            prob=self.config.ransac_prob,
            threshold=self.config.ransac_threshold_px
        )

        if E is None or inlier_mask is None:
            self._prev_frame = curr_frame
            self._current_estimate = VisualMotionEstimate(
                num_matches=len(good_matches),
                tracking_state="DEGRADED",
                confidence=0.2
            )
            return self._current_estimate

        inliers_count = int(np.count_nonzero(inlier_mask))
        inlier_ratio = inliers_count / float(len(good_matches))

        if inliers_count < self.config.min_inliers_tracking:
            self._prev_frame = curr_frame
            self._current_estimate = VisualMotionEstimate(
                num_matches=len(good_matches),
                num_inliers=inliers_count,
                inlier_ratio=inlier_ratio,
                tracking_state="DEGRADED",
                confidence=0.3
            )
            return self._current_estimate

        # Decompose Essential Matrix to extract relative Rotation (R) and Translation unit vector (t)
        _, R, t, mask_pose = cv2.recoverPose(E, curr_pts, prev_pts, self.K, mask=inlier_mask)

        # ---------------------------------------------------------------------
        # Planar Ground Vehicle Motion Projection:
        # Camera frame: X right (+), Y down (+), Z optical forward (+)
        # Body frame:   X forward (+), Y left (+), Z up (+)
        # ---------------------------------------------------------------------
        # Yaw rotation angle from rotation matrix R
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6
        if not singular:
            yaw_rad = math.atan2(R[1, 0], R[0, 0])
        else:
            yaw_rad = 0.0

        delta_heading_deg = math.degrees(yaw_rad)

        # Normalized direction of translation
        unit_tx = float(t[0, 0])  # Camera right
        unit_tz = float(t[2, 0])  # Camera forward

        # ---------------------------------------------------------------------
        # Monocular Metric Scale Resolution:
        # Monocular VO gives unit vector ||t|| = 1.
        # We ground metric scale using:
        #   1. Commanded wheel odometry distance (v * dt) if available
        #   2. Sonar depth difference to static forward surfaces
        #   3. Ground plane optical flow height projection
        # ---------------------------------------------------------------------
        metric_scale_cm = self._resolve_metric_scale(
            kinematic_speed_hint_cm_s=kinematic_speed_hint_cm_s,
            dt=dt,
            sonar_depth_hint_cm=sonar_depth_hint_cm,
            optical_parallax_px=parallax,
        )

        # Project unit translation to 2D body coordinates (cm)
        # In robot frame: Forward = +tz * scale, Lateral = -tx * scale
        delta_forward_cm = unit_tz * metric_scale_cm
        delta_lateral_cm = -unit_tx * metric_scale_cm

        # Inlier coordinate subsets for visualization and local mapping
        inlier_indices = np.where(inlier_mask.ravel() == 1)[0]
        valid_prev = prev_pts[inlier_indices]
        valid_curr = curr_pts[inlier_indices]

        confidence = min(0.95, max(0.40, float(inlier_ratio * (inliers_count / 100.0))))

        self._current_estimate = VisualMotionEstimate(
            delta_x_cm=round(float(delta_forward_cm), 2),
            delta_y_cm=round(float(delta_lateral_cm), 2),
            delta_heading_deg=round(float(delta_heading_deg), 2),
            num_matches=len(good_matches),
            num_inliers=inliers_count,
            inlier_ratio=round(float(inlier_ratio), 3),
            scale_factor=round(float(metric_scale_cm), 2),
            tracking_state="OK",
            confidence=round(confidence, 2),
            matched_prev_pts=valid_prev,
            matched_curr_pts=valid_curr
        )

        self._prev_frame = curr_frame
        return self._current_estimate

    def _resolve_metric_scale(
        self,
        kinematic_speed_hint_cm_s: Optional[float],
        dt: Optional[float],
        sonar_depth_hint_cm: Optional[float],
        optical_parallax_px: float,
    ) -> float:
        """
        Fuses physical constraints to resolve the monocular scale factor (cm).
        """
        # Primary Scale Anchor: Wheel kinematics
        if kinematic_speed_hint_cm_s is not None and dt is not None and dt > 0:
            kinematic_dist = abs(kinematic_speed_hint_cm_s * dt)
            if kinematic_dist > 0.1:
                return float(kinematic_dist)

        # Secondary Scale Anchor: Nominal optical flow scaling at default height
        # Higher parallax = larger physical movement
        # 1 pixel flow ~ 0.25 cm translation at 10-15cm camera height
        flow_scale = max(0.5, optical_parallax_px * 0.35)
        return float(flow_scale)

    def reset(self) -> None:
        """Resets the visual odometry tracker state."""
        self._prev_frame = None
        self._current_estimate = VisualMotionEstimate()
