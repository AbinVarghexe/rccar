"""
navigation/aruco_navigator.py — High-Precision ArUco & AprilTag Visual Navigation Engine.

Features:
1. Native OpenCV ArUco detector (DICT_4X4_50 / DICT_5X5_50).
2. Millimeter-accurate 3D metric pose estimation (Distance, Bearing, Lateral X, Depth Z).
3. Visual Waypoint & Landmark Navigation:
   - Tracks Tag ID (e.g. ID #0 = Home, ID #1 = Waypoint 1, ID #2 = Waypoint 2, ID #3 = Destination).
   - Generates closed-loop steering and distance commands.
   - Handles arrival condition at target distance (e.g. 25 cm).
4. Real-time 3D Pose HUD: Draws coordinate frames, bounding boxes, and metric distance labels.
5. On-the-fly marker generator for display on phone screens or printing.
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, Any, List
import cv2
import numpy as np


@dataclass
class ArucoPose:
    tag_id: int
    distance_cm: float
    bearing_deg: float      # Negative = Left of center, Positive = Right
    x_cm: float             # Lateral offset in centimeters
    y_cm: float             # Height offset in centimeters
    z_cm: float             # Forward depth in centimeters
    center_px: Tuple[int, int]
    corners_px: np.ndarray
    last_seen_time: float


class ArucoNavigator:
    """
    Precision Visual Landmark & Waypoint Navigation using ArUco Markers.
    """

    def __init__(
        self,
        marker_size_cm: float = 10.0,    # Physical size of printed tag (e.g. 10x10 cm)
        dict_type: int = cv2.aruco.DICT_4X4_50,
        camera_fov_h_deg: float = 62.2,
        arrival_dist_cm: float = 25.0,
    ) -> None:
        self.marker_size_cm = marker_size_cm
        self.dict_type = dict_type
        self.camera_fov_h_deg = camera_fov_h_deg
        self.arrival_dist_cm = arrival_dist_cm

        # Predefined ArUco dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(dict_type)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary)
        else:
            self.detector = None

        # Camera intrinsic matrix for OV5647 at 640x480
        fx = 530.0
        fy = 530.0
        cx = 320.0
        cy = 240.0
        self.cam_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)

        # 3D points of square marker in its own object frame
        s2 = marker_size_cm / 2.0
        self.obj_points = np.array([
            [-s2,  s2, 0.0],
            [ s2,  s2, 0.0],
            [ s2, -s2, 0.0],
            [-s2, -s2, 0.0]
        ], dtype=np.float32)

        # Active Navigation Target
        self.target_tag_id: Optional[int] = None
        self.latest_detected_tags: Dict[int, ArucoPose] = {}

    def set_target_tag(self, tag_id: Optional[int]) -> None:
        """Sets the active ArUco tag ID to home onto."""
        self.target_tag_id = tag_id

    def clear_target(self) -> None:
        self.target_tag_id = None

    def _fit_intrinsics(self, w: int, h: int) -> None:
        """Scale the OV5647 pinhole model (62.2 deg HFOV) to the real frame size."""
        if getattr(self, "_intr_size", None) == (w, h):
            return
        fx = (w / 2.0) / math.tan(math.radians(62.2 / 2.0))
        fy = fx * (h / 1080.0) / (w / 1920.0)   # 1080p sensor mode scaled to w x h
        self.cam_matrix = np.array([[fx, 0, w / 2.0], [0, fy, h / 2.0], [0, 0, 1]], dtype=np.float32)
        self._intr_size = (w, h)

    def detect_markers(self, bgr_frame: np.ndarray) -> Dict[int, ArucoPose]:
        """
        Detects all ArUco tags in frame and estimates 3D pose for each.
        """
        if bgr_frame is None or bgr_frame.size == 0:
            return {}

        gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
        self._fit_intrinsics(gray.shape[1], gray.shape[0])
        now = time.time()

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)

        results: Dict[int, ArucoPose] = {}
        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            for i, tag_id in enumerate(ids_flat):
                c = corners[i].reshape((4, 2))
                center_x = int(np.mean(c[:, 0]))
                center_y = int(np.mean(c[:, 1]))

                # 3D Pose Estimation via solvePnP
                ret, rvec, tvec = cv2.solvePnP(
                    self.obj_points,
                    c.astype(np.float32),
                    self.cam_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if ret:
                    tx = float(tvec[0][0])
                    ty = float(tvec[1][0])
                    tz = float(tvec[2][0])

                    dist_cm = float(np.linalg.norm(tvec))
                    bearing_deg = float(np.degrees(np.arctan2(tx, tz)))

                    pose = ArucoPose(
                        tag_id=int(tag_id),
                        distance_cm=round(dist_cm, 1),
                        bearing_deg=round(bearing_deg, 1),
                        x_cm=round(tx, 1),
                        y_cm=round(ty, 1),
                        z_cm=round(tz, 1),
                        center_px=(center_x, center_y),
                        corners_px=c,
                        last_seen_time=now,
                    )
                    results[int(tag_id)] = pose

        self.latest_detected_tags = results
        return results

    def get_navigation_command(self) -> Tuple[str, Optional[ArucoPose], str]:
        """
        Calculates steering decision to reach the active target tag.
        Returns:
            motor_cmd ('FORWARD', 'LEFT', 'RIGHT', 'STOP'),
            target_pose (ArucoPose or None),
            status_text (str)
        """
        if self.target_tag_id is None:
            return "STOP", None, "NO_TARGET_SET"

        target_pose = self.latest_detected_tags.get(self.target_tag_id)

        # If specific tag is not set or not found, check if ANY tag is visible
        if target_pose is None and self.target_tag_id == -1 and self.latest_detected_tags:
            # -1 means 'track any nearest visible tag'
            target_pose = min(self.latest_detected_tags.values(), key=lambda p: p.distance_cm)

        if target_pose is None:
            return "STOP", None, f"SEARCHING_TAG_{self.target_tag_id}"

        # Check Arrival Threshold
        if target_pose.distance_cm <= self.arrival_dist_cm:
            return "STOP", target_pose, f"ARRIVED_AT_TAG_{target_pose.tag_id}"

        # Closed-loop visual steering toward tag
        bearing = target_pose.bearing_deg
        if bearing < -8.0:
            cmd = "LEFT"
        elif bearing > 8.0:
            cmd = "RIGHT"
        else:
            cmd = "FORWARD"

        return cmd, target_pose, f"HOMING_TAG_{target_pose.tag_id}"

    def draw_hud(self, frame: np.ndarray, poses: Dict[int, ArucoPose]) -> np.ndarray:
        """Renders futuristic corner targeting brackets and sleek translucent metric badges."""
        h, w = frame.shape[:2]
        cx, cy = w // 2, h - 20

        for tag_id, pose in poses.items():
            pts = pose.corners_px.astype(np.int32).reshape((-1, 2))
            is_target = (self.target_tag_id == tag_id or self.target_tag_id == -1)
            box_color = (0, 255, 130) if is_target else (255, 205, 50)

            # Draw sleek HUD corner brackets (4 corner ticks)
            d = 9
            for i in range(4):
                p = pts[i]
                p_next = pts[(i + 1) % 4]
                p_prev = pts[(i - 1) % 4]
                v_next = p_next.astype(float) - p.astype(float)
                v_prev = p_prev.astype(float) - p.astype(float)
                l_next = np.linalg.norm(v_next)
                l_prev = np.linalg.norm(v_prev)
                if l_next > 0 and l_prev > 0:
                    u_next = (v_next / l_next) * min(float(d), l_next * 0.45)
                    u_prev = (v_prev / l_prev) * min(float(d), l_prev * 0.45)
                    cv2.line(frame, tuple(p.astype(int)), tuple((p + u_next).astype(int)), box_color, 2, cv2.LINE_AA)
                    cv2.line(frame, tuple(p.astype(int)), tuple((p + u_prev).astype(int)), box_color, 2, cv2.LINE_AA)

            cv2.circle(frame, pose.center_px, 3, box_color, -1, cv2.LINE_AA)

            # Subtle trajectory guide vector to target
            if is_target:
                cv2.line(frame, (cx, cy), pose.center_px, (0, 240, 200), 1, cv2.LINE_AA)

            # Tag HUD Badge with translucent backing
            label = f"TAG #{tag_id} · {pose.distance_cm:.0f}cm [{pose.bearing_deg:+.0f}°]"
            tx, ty = pose.center_px
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.35
            (tw, th), _ = cv2.getTextSize(label, font, scale, 1)
            bx = tx - tw // 2 - 5
            by = max(8, ty - 22)
            bw = tw + 10
            bh = th + 7

            overlay = frame.copy()
            cv2.rectangle(overlay, (bx, by), (bx + bw, by + bh), (15, 20, 32), -1)
            cv2.addWeighted(overlay, 0.70, frame, 0.30, 0, frame)
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), box_color, 1, cv2.LINE_AA)
            cv2.putText(frame, label, (bx + 5, by + bh - 2), font, scale, (240, 245, 255), 1, cv2.LINE_AA)

        return frame

    @staticmethod
    def generate_marker_base64(tag_id: int, size_px: int = 240) -> str:
        """Generates a PNG ArUco marker encoded as base64 string for direct browser rendering."""
        import base64
        dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker = cv2.aruco.generateImageMarker(dict_4x4, tag_id, size_px, 1)

        # Add clean white border (quiet zone)
        border_size = int(size_px * 0.15)
        bordered = cv2.copyMakeBorder(
            marker, border_size, border_size, border_size, border_size,
            cv2.BORDER_CONSTANT, value=255
        )
        _, buf = cv2.imencode(".png", bordered)
        return base64.b64encode(buf).decode("utf-8")
