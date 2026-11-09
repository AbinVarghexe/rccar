"""Offline ArUco marker navigation for region-to-region driving.

This module deliberately avoids global X/Y odometry.  A known-size marker
provides a fresh local direction and distance reference at every frame.  The
HC-SR04 remains the independent collision-safety layer in the driver.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

import cv2
import numpy as np


@dataclass
class MarkerObservation:
    marker_id: int
    center_x: float
    center_y: float
    distance_cm: float
    bearing_deg: float
    area_px: float


@dataclass
class MarkerRouteStatus:
    enabled: bool
    route: List[int]
    route_index: int
    active_marker_id: Optional[int]
    visible: bool
    arrived: bool
    route_complete: bool
    distance_cm: Optional[float]
    bearing_deg: Optional[float]
    steering: float
    reason: str
    detector_available: bool
    last_seen_age_sec: Optional[float]

    def as_dict(self) -> Dict[str, Any]:
        return {
            "enabled": self.enabled,
            "route": list(self.route),
            "route_index": self.route_index,
            "active_marker_id": self.active_marker_id,
            "visible": self.visible,
            "arrived": self.arrived,
            "route_complete": self.route_complete,
            "distance_cm": self.distance_cm,
            "bearing_deg": self.bearing_deg,
            "steering": self.steering,
            "reason": self.reason,
            "detector_available": self.detector_available,
            "last_seen_age_sec": self.last_seen_age_sec,
        }


class MarkerRouteNavigator:
    """Detects a configured sequence of known-size ArUco markers."""

    def __init__(
        self,
        marker_size_cm: float = 8.0,
        dictionary_name: str = "DICT_4X4_50",
        camera_fov_h_deg: float = 62.2,
        arrival_distance_cm: float = 35.0,
        throttle: float = 0.30,
    ) -> None:
        self.marker_size_cm = max(1.0, float(marker_size_cm))
        self.dictionary_name = str(dictionary_name)
        self.camera_fov_h_deg = float(camera_fov_h_deg)
        self.arrival_distance_cm = max(10.0, float(arrival_distance_cm))
        self.throttle = max(0.15, min(0.55, float(throttle)))

        self.route: List[int] = []
        self.route_index = 0
        self.enabled = False
        self.last_observation: Optional[MarkerObservation] = None
        self.last_seen_at: Optional[float] = None
        self._dictionary = None
        self._detector = None
        self._detector_available = False
        self._init_detector()

    @property
    def detector_available(self) -> bool:
        return self._detector_available

    def _init_detector(self) -> None:
        aruco = getattr(cv2, "aruco", None)
        if aruco is None:
            return
        dictionary_id = getattr(aruco, self.dictionary_name, None)
        if dictionary_id is None:
            return
        try:
            self._dictionary = aruco.getPredefinedDictionary(dictionary_id)
            if hasattr(aruco, "DetectorParameters"):
                params = aruco.DetectorParameters()
            else:
                params = aruco.DetectorParameters_create()
            if hasattr(aruco, "ArucoDetector"):
                self._detector = aruco.ArucoDetector(self._dictionary, params)
            else:
                self._detector = params
            self._detector_available = True
        except Exception:
            self._dictionary = None
            self._detector = None
            self._detector_available = False

    def set_route(self, marker_ids: Iterable[int], enabled: bool = True) -> List[int]:
        clean: List[int] = []
        for marker_id in marker_ids:
            value = int(marker_id)
            if value < 0 or value > 1023:
                raise ValueError("marker IDs must be between 0 and 1023")
            if value not in clean:
                clean.append(value)
        self.route = clean
        self.route_index = 0
        self.last_observation = None
        self.last_seen_at = None
        self.enabled = bool(enabled and clean)
        return list(self.route)

    def clear_route(self) -> None:
        self.route = []
        self.route_index = 0
        self.enabled = False
        self.last_observation = None
        self.last_seen_at = None

    def active_marker_id(self) -> Optional[int]:
        if not self.enabled or self.route_index >= len(self.route):
            return None
        return self.route[self.route_index]

    def _detect_markers(self, frame: np.ndarray):
        if not self._detector_available or frame is None or frame.size == 0:
            return [], None
        aruco = cv2.aruco
        if hasattr(aruco, "ArucoDetector"):
            return self._detector.detectMarkers(frame)[:2]
        return aruco.detectMarkers(frame, self._dictionary, parameters=self._detector)[:2]

    def detect(self, frame: np.ndarray) -> List[MarkerObservation]:
        corners, ids = self._detect_markers(frame)
        if ids is None or not corners:
            return []

        h, w = frame.shape[:2]
        fx = (w / 2.0) / math.tan(math.radians(self.camera_fov_h_deg) / 2.0)
        cx = w / 2.0
        observations: List[MarkerObservation] = []

        rvecs = tvecs = None
        estimate_pose = getattr(cv2.aruco, "estimatePoseSingleMarkers", None)
        if estimate_pose is not None:
            try:
                camera_matrix = np.array(
                    [[fx, 0.0, cx], [0.0, fx, h / 2.0], [0.0, 0.0, 1.0]],
                    dtype=np.float64,
                )
                rvecs, tvecs, _ = estimate_pose(
                    corners, self.marker_size_cm, camera_matrix, np.zeros((5, 1))
                )
            except Exception:
                rvecs = tvecs = None

        for index, marker_corners in enumerate(corners):
            marker_id = int(np.asarray(ids[index]).reshape(-1)[0])
            points = np.asarray(marker_corners, dtype=np.float32).reshape(-1, 2)
            center_x = float(np.mean(points[:, 0]))
            center_y = float(np.mean(points[:, 1]))
            area_px = float(cv2.contourArea(points))
            pixel_width = max(1.0, float(np.linalg.norm(points[0] - points[1])))

            if tvecs is not None:
                tx, _, tz = [float(value) for value in tvecs[index][0]]
                distance_cm = math.hypot(tx, tz)
                bearing_deg = math.degrees(math.atan2(tx, tz))
            else:
                distance_cm = self.marker_size_cm * fx / pixel_width
                bearing_deg = math.degrees(math.atan2(center_x - cx, fx))

            observations.append(MarkerObservation(
                marker_id=marker_id,
                center_x=round(center_x, 1),
                center_y=round(center_y, 1),
                distance_cm=round(max(0.0, distance_cm), 1),
                bearing_deg=round(bearing_deg, 1),
                area_px=round(area_px, 1),
            ))
        return observations

    def update(self, frame: np.ndarray, servo_angle_deg: float = 90.0) -> MarkerRouteStatus:
        active_id = self.active_marker_id()
        if not self.enabled or active_id is None:
            return self.status(reason="Marker route disabled")
        if not self.detector_available:
            return self.status(reason="OpenCV ArUco detector unavailable")

        observations = self.detect(frame)
        target = next((item for item in observations if item.marker_id == active_id), None)
        if target is None:
            return self.status(reason=f"Marker {active_id} not visible; holding STOP")

        # Convert camera bearing to vehicle bearing.  The route assumes the
        # camera is centered while moving, but this keeps the math correct if
        # a small servo offset remains after centering.
        vehicle_bearing = target.bearing_deg + (float(servo_angle_deg) - 90.0)
        steering = max(-0.45, min(0.45, vehicle_bearing / 55.0))
        self.last_observation = MarkerObservation(
            marker_id=target.marker_id,
            center_x=target.center_x,
            center_y=target.center_y,
            distance_cm=target.distance_cm,
            bearing_deg=round(vehicle_bearing, 1),
            area_px=target.area_px,
        )
        self.last_seen_at = time.time()

        if target.distance_cm <= self.arrival_distance_cm:
            self.route_index += 1
            completed = self.route_index >= len(self.route)
            if completed:
                self.enabled = False
            return self.status(
                arrived=True,
                route_complete=completed,
                reason=("Final marker reached; holding STOP" if completed
                        else f"Marker {active_id} reached; advancing to marker {self.active_marker_id()}"),
            )

        return self.status(
            visible=True,
            distance_cm=target.distance_cm,
            bearing_deg=round(vehicle_bearing, 1),
            steering=round(steering, 3),
            reason=f"Driving to marker {active_id}",
        )

    def status(
        self,
        *,
        visible: bool = False,
        arrived: bool = False,
        route_complete: bool = False,
        distance_cm: Optional[float] = None,
        bearing_deg: Optional[float] = None,
        steering: float = 0.0,
        reason: str = "",
    ) -> MarkerRouteStatus:
        age = None if self.last_seen_at is None else max(0.0, time.time() - self.last_seen_at)
        obs = self.last_observation
        return MarkerRouteStatus(
            enabled=self.enabled,
            route=list(self.route),
            route_index=self.route_index,
            active_marker_id=self.active_marker_id(),
            visible=visible,
            arrived=arrived,
            route_complete=route_complete,
            distance_cm=distance_cm if distance_cm is not None else (obs.distance_cm if obs else None),
            bearing_deg=bearing_deg if bearing_deg is not None else (obs.bearing_deg if obs else None),
            steering=steering,
            reason=reason,
            detector_available=self.detector_available,
            last_seen_age_sec=round(age, 2) if age is not None else None,
        )

