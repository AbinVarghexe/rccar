"""
Unit tests for vision preprocessing, lane detection, and junction detection.
"""

import os
import sys
import unittest
import numpy as np
import cv2

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from autonomous_car.config import PreprocessingConfig
    from autonomous_car.vision.preprocessing import ImagePreprocessor
    from autonomous_car.vision.lane_detector import LaneDetector, Direction
    from autonomous_car.vision.junction_detector import JunctionDetector, JunctionType
except ImportError:
    from config import PreprocessingConfig
    from vision.preprocessing import ImagePreprocessor
    from vision.lane_detector import LaneDetector, Direction
    from vision.junction_detector import JunctionDetector, JunctionType

class TestVisionPipeline(unittest.TestCase):
    def setUp(self):
        self.config = PreprocessingConfig()
        self.preprocessor = ImagePreprocessor(self.config)
        self.lane_detector = LaneDetector()
        self.junction_detector = JunctionDetector()

    def test_preprocessing_output_shape(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        # Draw a synthetic line
        cv2.line(frame, (320, 480), (320, 200), (255, 255, 255), 10)
        
        binary_img = self.preprocessor.process(frame)
        self.assertEqual(binary_img.shape, (480, 640))
        self.assertEqual(binary_img.dtype, np.uint8)

    def test_lane_detection_straight(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        binary = np.zeros((480, 640), dtype=np.uint8)
        # Draw vertical line exactly in center (x=320)
        cv2.line(binary, (320, 480), (320, 200), 255, 20)

        result = self.lane_detector.detect(binary, frame)
        self.assertTrue(result.contour_found)
        self.assertAlmostEqual(result.error, 0.0, delta=5.0)
        self.assertEqual(result.direction, Direction.STRAIGHT)

    def test_lane_detection_left_turn(self):
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        binary = np.zeros((480, 640), dtype=np.uint8)
        # Draw vertical line shifted left (x=200)
        cv2.line(binary, (200, 480), (200, 200), 255, 20)

        result = self.lane_detector.detect(binary, frame)
        self.assertTrue(result.contour_found)
        self.assertLess(result.error, -15.0)
        self.assertEqual(result.direction, Direction.LEFT)

    def test_junction_detection_dead_end(self):
        empty_binary = np.zeros((480, 640), dtype=np.uint8)
        result = self.junction_detector.detect(empty_binary)
        self.assertEqual(result.junction_type, JunctionType.DEAD_END)

if __name__ == "__main__":
    unittest.main()
