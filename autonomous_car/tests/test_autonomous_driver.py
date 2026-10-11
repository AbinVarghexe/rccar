"""
Automated Verification Test for Autonomous Driving and Perception Subsystem.
Tests:
  1. YOLO Object Detection & Class Filtering (bus, car, person, etc.)
  2. Bounding Box Looming & Time-to-Collision (TTC) Risk Analysis
  3. Visual Corridor Analysis & Clearance Calculation (Left vs Right)
  4. Autonomous Driving State Machine & Rule Execution (Bus Stop, Obstacle Avoidance)
  5. Live Camera HUD Annotations & Overlay Generation
  6. Dashboard Server REST Endpoints (/api/auto/*)
"""

import os
import sys
import time
import unittest
import numpy as np
import cv2

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from config import config
from ai.classes import ActionEnum, DecisionResult, DetectionResult, BoundingBox
from ai.detector import YOLODetector
from ai.tracker import ObjectTracker
from ai.risk_analyzer import RiskAnalyzer
from ai.corridor_scanner import CorridorScanner
from ai.visualization import Visualizer
from ai.server_client import ServerAIClient
from hardware.motors import MotorController
from hardware.servo import ServoController
from vision.camera import Camera
from navigation.autonomous_driver import AutonomousDriver
from dashboard.app import DashboardServer


class TestAutonomousSystem(unittest.TestCase):

    def setUp(self):
        self.motors = MotorController(auto_setup=False)
        self.servo = ServoController(auto_setup=False)
        self.camera = Camera(width=640, height=480)
        self.driver = AutonomousDriver(
            config=config.autonomous,
            motors=self.motors,
            servo=self.servo,
            camera=self.camera
        )

    def tearDown(self):
        self.driver.stop()
        self.camera.stop()

    def test_01_corridor_scanner(self):
        """Test computer vision corridor analyzer on open vs blocked paths."""
        scanner = CorridorScanner(lane_width_ratio=0.42)

        # 1. Clean open road frame
        open_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        open_res = scanner.analyze(open_frame)
        self.assertFalse(open_res["center_blocked"])
        self.assertEqual(open_res["best_direction"], "FORWARD")
        self.assertGreaterEqual(open_res["left_clearance"], 0.8)
        self.assertGreaterEqual(open_res["right_clearance"], 0.8)

        # 2. Frame with high-density obstacle in left sector
        left_obs_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        for i in range(200, 470, 5):
            cv2.line(left_obs_frame, (20, i), (180, i), (255, 255, 255), 2)
        left_res = scanner.analyze(left_obs_frame)
        self.assertGreater(left_res["right_clearance"], left_res["left_clearance"])

    def test_02_risk_analyzer_looming_ttc(self):
        """Test looming bounding box expansion correctly calculates TTC."""
        analyzer = RiskAnalyzer()
        t0 = time.time()

        # Frame 1: distant object
        b1 = BoundingBox(x1=280, y1=200, x2=320, y2=260) # width = 40
        d1 = [DetectionResult(class_name="car", confidence=0.85, bbox=b1, track_id=1)]
        r1 = analyzer.update(d1, t0)

        # Frame 2: object rapidly expanding (approaching car)
        b2 = BoundingBox(x1=240, y1=160, x2=360, y2=320) # width = 120
        d2 = [DetectionResult(class_name="car", confidence=0.88, bbox=b2, track_id=1)]
        r2 = analyzer.update(d2, t0 + 0.2)

        track_risk = r2.get(1, {})
        self.assertIn("risk_score", track_risk)
        self.assertGreater(track_risk["risk_score"], 0.5)

    def test_03_emergency_stop_on_bus(self):
        """Verify prompt requirement: Whenever you see a bus, car must immediately stop."""
        test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Inject bus detection directly
        bus_det = DetectionResult(
            class_name="bus",
            confidence=0.92,
            bbox=BoundingBox(x1=200, y1=150, x2=440, y2=450), # Large bus in driving path
            track_id=10
        )
        self.driver.tracker.track = lambda f: [bus_det]

        decision = self.driver._evaluate_environment_and_decide(test_frame, 640, 480)
        self.assertEqual(decision.action, ActionEnum.STOP)
        self.assertTrue(decision.hazard_detected)
        self.assertIn("Bus", decision.reason)

    def test_04_steering_avoidance_better_path(self):
        """Verify prompt requirement: If an obstacle is there, find a better path."""
        test_frame = np.zeros((480, 640, 3), dtype=np.uint8)

        # Obstacle on left side of corridor -> must steer RIGHT
        obs_left = DetectionResult(
            class_name="traffic cone",
            confidence=0.80,
            bbox=BoundingBox(x1=160, y1=280, x2=240, y2=420), # center_x = 200 < 320
            track_id=12
        )
        self.driver.tracker.track = lambda f: [obs_left]

        decision = self.driver._evaluate_environment_and_decide(test_frame, 640, 480)
        self.assertEqual(decision.action, ActionEnum.RIGHT)
        self.assertIn("Right", decision.reason)

        # Obstacle on right side of corridor -> must steer LEFT
        obs_right = DetectionResult(
            class_name="traffic cone",
            confidence=0.80,
            bbox=BoundingBox(x1=400, y1=280, x2=480, y2=420), # center_x = 440 > 320
            track_id=14
        )
        self.driver.tracker.track = lambda f: [obs_right]

        decision2 = self.driver._evaluate_environment_and_decide(test_frame, 640, 480)
        self.assertEqual(decision2.action, ActionEnum.LEFT)
        self.assertIn("Left", decision2.reason)

    def test_05_hud_visualizer(self):
        """Verify sleek HUD overlay renders bounding boxes, decision banners, and corridor."""
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        dets = [
            DetectionResult(
                class_name="bus",
                confidence=0.91,
                bbox=BoundingBox(x1=180, y1=140, x2=460, y2=420),
                track_id=3
            )
        ]
        risks = {3: {"risk_score": 0.95, "ttc": 1.2}}
        dec = DecisionResult(action=ActionEnum.STOP, reason="EMERGENCY STOP: Bus detected in driving path")

        annotated = Visualizer.draw(
            frame=frame,
            detections=dets,
            risks=risks,
            decision=dec,
            fps=30.0,
            latency_ms=25.0,
            lane_width_ratio=0.42,
            mode_str="HYBRID",
            auto_enabled=True
        )
        self.assertIsNotNone(annotated)
        self.assertEqual(annotated.shape, (480, 640, 3))
        # Top banner should have drawn colors
        self.assertGreater(np.count_nonzero(annotated[:50, :, :]), 0)

    def test_06_dashboard_endpoints(self):
        """Test DashboardServer Flask API routes for autonomous driving."""
        server = DashboardServer(
            config=config.dashboard,
            motor_controller=self.motors,
            servo_controller=self.servo,
            camera=self.camera,
            autonomous_driver=self.driver
        )
        client = server.app.test_client()

        # 1. /api/auto/status
        res = client.get('/api/auto/status')
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertIn("autonomous_enabled", data)
        self.assertIn("mode", data)

        # 2. /api/auto/toggle
        res = client.post('/api/auto/toggle', json={"enabled": True})
        self.assertEqual(res.status_code, 200)
        self.assertTrue(self.driver.is_enabled)

        res = client.post('/api/auto/toggle', json={"enabled": False})
        self.assertEqual(res.status_code, 200)
        self.assertFalse(self.driver.is_enabled)

        # 3. /api/auto/mode
        res = client.post('/api/auto/mode', json={"mode": "local_ai"})
        self.assertEqual(res.status_code, 200)
        self.assertEqual(self.driver.mode, "local_ai")

        # 4. /api/auto/config
        res = client.post('/api/auto/config', json={"cruise_speed": 0.70, "server_url": "http://10.0.0.99:8000"})
        self.assertEqual(res.status_code, 200)
        self.assertAlmostEqual(self.driver.cruise_speed, 0.70, places=2)

        # 5. /api/auto/overlay
        res = client.post('/api/auto/overlay', json={"enabled": False})
        self.assertEqual(res.status_code, 200)
        self.assertFalse(self.camera.overlay_enabled)

        res = client.post('/api/auto/overlay', json={"enabled": True})
        self.assertEqual(res.status_code, 200)
        self.assertTrue(self.camera.overlay_enabled)


if __name__ == "__main__":
    unittest.main()
