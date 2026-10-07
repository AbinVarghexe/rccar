"""
Unit tests for Camera Input and Footage API Endpoints via Dashboard.
"""

import os
import sys
import time
import unittest

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from config import DashboardConfig
from hardware.motors import MotorController
from hardware.servo import ServoController
from vision.camera import Camera, MEDIA_DIR
from dashboard.app import DashboardServer


class TestCameraAndFootageAPI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.camera = Camera(width=640, height=480)
        cls.camera.start()
        # Allow sensor / backend warm-up
        time.sleep(1.0)

        cls.servo = ServoController(auto_setup=False)
        cls.motors = MotorController()

        cls.server = DashboardServer(
            config=DashboardConfig(),
            motor_controller=cls.motors,
            servo_controller=cls.servo,
            camera=cls.camera,
        )
        cls.client = cls.server.app.test_client()

    @classmethod
    def tearDownClass(cls):
        cls.camera.stop()

    def test_01_api_frame_endpoint(self):
        """Test /api/frame returns valid JPEG content."""
        res = self.client.get("/api/frame")
        self.assertEqual(res.status_code, 200)
        self.assertEqual(res.content_type, "image/jpeg")
        self.assertGreater(len(res.data), 1000)
        # Check JPEG SOI (0xFFD8) and EOI (0xFFD9) markers
        self.assertTrue(res.data.startswith(b"\xff\xd8"))
        self.assertTrue(res.data.endswith(b"\xff\xd9"))

    def test_02_video_feed_stream(self):
        """Test /video_feed returns multipart MJPEG stream."""
        res = self.client.get("/video_feed")
        self.assertEqual(res.status_code, 200)
        self.assertIn("multipart/x-mixed-replace", res.content_type)

    def test_03_photo_capture_endpoint(self):
        """Test /api/capture/photo saves photo and returns valid JSON."""
        res = self.client.post("/api/capture/photo")
        self.assertEqual(res.status_code, 200)
        data = res.get_json()
        self.assertEqual(data["status"], "success")
        self.assertIn("filename", data)
        self.assertTrue(data["filename"].startswith("photo_"))

        photo_path = os.path.join(MEDIA_DIR, data["filename"])
        self.assertTrue(os.path.exists(photo_path))
        self.assertGreater(os.path.getsize(photo_path), 0)

    def test_04_video_recording_endpoints(self):
        """Test starting and stopping video recording via API."""
        start_res = self.client.post("/api/capture/video/start")
        self.assertEqual(start_res.status_code, 200)
        data = start_res.get_json()
        self.assertEqual(data["status"], "recording")
        filename = data["filename"]
        self.assertTrue(filename.startswith("video_"))

        # Verify telemetry reflects recording state
        telem_res = self.client.get("/api/telemetry")
        self.assertEqual(telem_res.status_code, 200)
        self.assertTrue(telem_res.get_json()["is_recording"])

        # Record for 1.5 seconds
        time.sleep(1.5)

        # Stop recording
        stop_res = self.client.post("/api/capture/video/stop")
        self.assertEqual(stop_res.status_code, 200)
        stop_data = stop_res.get_json()
        self.assertEqual(stop_data["status"], "saved")
        self.assertEqual(stop_data["filename"], filename)

        # Allow background disk flush
        time.sleep(0.5)

        video_path = os.path.join(MEDIA_DIR, filename)
        self.assertTrue(os.path.exists(video_path))
        self.assertGreater(os.path.getsize(video_path), 0)

    def test_05_media_list_endpoint(self):
        """Test /api/media/list returns list of captured photos and video footage."""
        res = self.client.get("/api/media/list")
        self.assertEqual(res.status_code, 200)
        media_list = res.get_json()
        self.assertIsInstance(media_list, list)
        self.assertGreater(len(media_list), 0)

        # Check structure of media entries
        for item in media_list:
            self.assertIn("name", item)
            self.assertIn("type", item)
            self.assertIn("size_kb", item)
            self.assertIn(item["type"], ["photo", "video"])

    def test_06_media_download_endpoint(self):
        """Test /api/media/<filename> can download saved photo and video footage."""
        res = self.client.get("/api/media/list")
        media_list = res.get_json()
        first_item = media_list[0]
        filename = first_item["name"]

        dl_res = self.client.get(f"/api/media/{filename}")
        self.assertEqual(dl_res.status_code, 200)
        self.assertGreater(len(dl_res.data), 0)


if __name__ == "__main__":
    unittest.main()
