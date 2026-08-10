"""
Unit tests for Navigation State Machine, Planner, Waypoint Navigator, and Camera Servo Scanning.
"""

import os
import sys
import unittest
import numpy as np

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from autonomous_car.config import AppConfig, SerialConfig
    from autonomous_car.navigation.state_machine import NavigationStateMachine, State
    from autonomous_car.navigation.waypoint import WaypointNavigator
    from autonomous_car.navigation.planner import NavigationPlanner
    from autonomous_car.control.serial_comm import SerialCommunicator
    from autonomous_car.control.controller import VehicleController
    from autonomous_car.vision.lane_detector import LaneResult, Direction
    from autonomous_car.vision.junction_detector import JunctionResult, JunctionType
    from autonomous_car.sensors.ultrasonic import DualUltrasonicData, SingleSensorData
except ImportError:
    from config import AppConfig, SerialConfig
    from navigation.state_machine import NavigationStateMachine, State
    from navigation.waypoint import WaypointNavigator
    from navigation.planner import NavigationPlanner
    from control.serial_comm import SerialCommunicator
    from control.controller import VehicleController
    from vision.lane_detector import LaneResult, Direction
    from vision.junction_detector import JunctionResult, JunctionType
    from sensors.ultrasonic import DualUltrasonicData, SingleSensorData

class TestNavigation(unittest.TestCase):
    def setUp(self):
        self.app_config = AppConfig()
        self.state_machine = NavigationStateMachine(initial_state=State.IDLE)
        self.serial_comm = SerialCommunicator(SerialConfig(mock_serial=True))
        self.controller = VehicleController(self.app_config, self.serial_comm)
        self.waypoint_nav = WaypointNavigator(self.app_config.waypoint)
        self.planner = NavigationPlanner(
            state_machine=self.state_machine,
            controller=self.controller,
            waypoint_navigator=self.waypoint_nav
        )

    def test_servo_pan_command(self):
        self.controller.pan_camera_left()
        self.assertEqual(self.serial_comm.current_servo_angle, 150)
        self.controller.pan_camera_right()
        self.assertEqual(self.serial_comm.current_servo_angle, 30)
        self.controller.pan_camera_center()
        self.assertEqual(self.serial_comm.current_servo_angle, 90)

    def test_state_machine_transition(self):
        self.assertEqual(self.state_machine.current_state, State.IDLE)
        res = self.state_machine.transition_to(State.FOLLOW_LANE)
        self.assertTrue(res)
        self.assertEqual(self.state_machine.current_state, State.FOLLOW_LANE)

    def test_planner_obstacle_triggers_scan(self):
        self.planner.enable()
        
        sensor_data = DualUltrasonicData(
            front=SingleSensorData(distance_cm=25.0, obstacle_warning=True, emergency_stop=False),
            rear=SingleSensorData(distance_cm=100.0, obstacle_warning=False, emergency_stop=False),
            emergency_stop_triggered=False
        )

        lane_res = LaneResult(
            lane_center_x=320, lane_center_y=240, image_center_x=320,
            error=0.0, direction=Direction.STRAIGHT, contour_found=True,
            largest_contour=None, annotated_frame=np.zeros((480,640,3), dtype=np.uint8)
        )

        junction_res = JunctionResult(JunctionType.NONE, 0.0, 0, False, False, False)

        # Trigger update step which executes scan
        state = self.planner.update(lane_res, junction_res, sensor_data)
        self.assertIn(state, [State.SCANNING_ENVIRONMENT, State.TURN_LEFT, State.TURN_RIGHT, State.STOP])

if __name__ == "__main__":
    unittest.main()
