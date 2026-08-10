"""
Unit tests for ultrasonic sensor readings and threshold triggers.
"""

import os
import sys
import unittest

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from autonomous_car.config import SensorConfig
    from autonomous_car.sensors.ultrasonic import UltrasonicSensors
except ImportError:
    from config import SensorConfig
    from sensors.ultrasonic import UltrasonicSensors

class TestSensors(unittest.TestCase):
    def setUp(self):
        self.config = SensorConfig(warning_distance_cm=30.0, emergency_stop_distance_cm=15.0)
        self.sensors = UltrasonicSensors(self.config)

    def test_read_all_structure(self):
        data = self.sensors.read_all()
        self.assertIsNotNone(data.front)
        self.assertIsNotNone(data.rear)
        self.assertIsInstance(data.front.distance_cm, float)
        self.assertIsInstance(data.rear.distance_cm, float)

if __name__ == "__main__":
    unittest.main()
