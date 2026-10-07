import os
import sys
import unittest

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from config import DashboardConfig
from dashboard.app import DashboardServer


class TestDashboardLivePreview(unittest.TestCase):
    def setUp(self):
        # Initialized without pre-started camera to test auto-start
        self.server = DashboardServer(config=DashboardConfig())
        self.client = self.server.app.test_client()

    def tearDown(self):
        if self.server.camera:
            self.server.camera.stop()

    def test_camera_auto_initialized(self):
        self.assertTrue(self.server.camera.is_initialized)

    def test_index_serves_video_feed(self):
        res = self.client.get("/")
        self.assertEqual(res.status_code, 200)
        html = res.get_data(as_text=True)
        self.assertIn('src="/video_feed"', html)
        self.assertIn('id="cam"', html)

    def test_api_frame_immediate_valid_jpeg(self):
        res = self.client.get("/api/frame")
        self.assertEqual(res.status_code, 200)
        self.assertEqual(res.content_type, "image/jpeg")
        self.assertGreater(len(res.data), 1000)

    def test_video_feed_multipart_stream(self):
        res = self.client.get("/video_feed")
        self.assertEqual(res.status_code, 200)
        self.assertIn("multipart/x-mixed-replace", res.content_type)


if __name__ == "__main__":
    unittest.main()
