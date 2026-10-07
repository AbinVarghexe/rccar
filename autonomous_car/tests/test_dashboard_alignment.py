"""
Unit tests for Camera Field Alignment, Servo Pan Trim Calibration, and Dashboard API.
"""

import os
import sys
import unittest

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
for p in [curr_dir, parent_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from config import DashboardConfig
from hardware.servo import ServoController
from vision.camera import Camera
from dashboard.app import DashboardServer


class TestCameraAlignmentAndServoTrim(unittest.TestCase):
    def setUp(self):
        self.servo = ServoController(auto_setup=False)
        self.camera = Camera(width=640, height=480)
        self.server = DashboardServer(
            config=DashboardConfig(),
            servo_controller=self.servo,
            camera=self.camera,
        )
        self.client = self.server.app.test_client()

    def tearDown(self):
        if self.camera:
            self.camera.stop()

    def test_servo_trim_offset(self):
        self.assertEqual(self.servo.trim_offset, 0.0)
        self.servo.set_trim(5.0)
        self.assertEqual(self.servo.trim_offset, 5.0)
        self.servo.set_angle(90.0)
        self.assertEqual(self.servo.current_angle, 90.0)

    def test_servo_trim_clamping(self):
        self.servo.set_trim(50.0)
        self.assertEqual(self.servo.trim_offset, 30.0)
        self.servo.set_trim(-45.0)
        self.assertEqual(self.servo.trim_offset, -30.0)

    def test_camera_alignment_settings(self):
        self.assertEqual(self.camera.flip_h, False)
        self.assertEqual(self.camera.flip_v, False)
        self.assertEqual(self.camera.rotation, 0)

        self.camera.set_alignment(flip_h=True, flip_v=True, rotation=180)
        self.assertEqual(self.camera.flip_h, True)
        self.assertEqual(self.camera.flip_v, True)
        self.assertEqual(self.camera.rotation, 180)

    def test_api_camera_alignment_endpoint(self):
        res = self.client.post("/api/camera/alignment", json={
            "flip_h": True,
            "flip_v": False,
            "rotation": 90
        })
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["flip_h"], True)
        self.assertEqual(data["flip_v"], False)
        self.assertEqual(data["rotation"], 90)

        get_res = self.client.get("/api/camera/alignment")
        self.assertEqual(get_res.status_code, 200)
        get_data = get_res.get_json()
        self.assertEqual(get_data["flip_h"], True)
        self.assertEqual(get_data["rotation"], 90)

    def test_api_servo_trim_endpoint(self):
        res = self.client.post("/api/servo/trim", json={"offset": 3.5})
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["trim_offset"], 3.5)
        self.assertEqual(self.servo.trim_offset, 3.5)

        get_res = self.client.get("/api/servo/trim")
        self.assertEqual(get_res.status_code, 200)
        self.assertEqual(get_res.get_json()["trim_offset"], 3.5)

    def test_dashboard_renders_with_alignment_elements(self):
        res = self.client.get("/")
        self.assertEqual(res.status_code, 200)
        html = res.get_data(as_text=True)
        self.assertIn("cam-align-overlay", html)
        self.assertIn("Align Guide", html)
        self.assertIn("Flip H", html)
        self.assertIn("Flip V", html)
        self.assertIn("calibrateCurrentAsCenter", html)
        self.assertIn("Set Center", html)


if __name__ == "__main__":
    unittest.main()
