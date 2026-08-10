"""
Unit tests for PID controller, vehicle commands, and motor primitives.
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
    from autonomous_car.config import PIDConfig, SerialConfig, AppConfig
    from autonomous_car.control.pid import PIDController
    from autonomous_car.control.serial_comm import SerialCommunicator, VehicleCommand
    from autonomous_car.control.controller import VehicleController
except ImportError:
    from config import PIDConfig, SerialConfig, AppConfig
    from control.pid import PIDController
    from control.serial_comm import SerialCommunicator, VehicleCommand
    from control.controller import VehicleController

class TestControlSystem(unittest.TestCase):
    def setUp(self):
        self.pid_config = PIDConfig(kp=0.5, ki=0.0, kd=0.1, setpoint=0.0)
        self.pid = PIDController(self.pid_config)

        self.serial_config = SerialConfig(mock_serial=True)
        self.serial_comm = SerialCommunicator(self.serial_config)
        self.app_config = AppConfig()
        self.controller = VehicleController(self.app_config, self.serial_comm)

    def test_pid_zero_error(self):
        output = self.pid.compute(0.0)
        self.assertAlmostEqual(output, 0.0)

    def test_pid_positive_error(self):
        output = self.pid.compute(50.0, dt=0.033)
        self.assertGreater(output, 0.0)

    def test_pid_output_clamping(self):
        output = self.pid.compute(1000.0, dt=0.033)
        self.assertLessEqual(output, self.pid_config.output_max)

    def test_serial_mock_command(self):
        res = self.serial_comm.send_command(VehicleCommand.FORWARD)
        self.assertTrue(res)
        self.assertEqual(self.serial_comm.last_command, VehicleCommand.FORWARD)

    def test_motor_primitives(self):
        self.controller.moveForward(1.0)
        self.assertEqual(self.serial_comm.last_command, VehicleCommand.FORWARD)

        self.controller.turnLeft(1.0)
        self.assertEqual(self.serial_comm.last_command, VehicleCommand.LEFT)

        self.controller.turnRight(1.0)
        self.assertEqual(self.serial_comm.last_command, VehicleCommand.RIGHT)

        self.controller.stop()
        self.assertEqual(self.serial_comm.last_command, VehicleCommand.STOP)

if __name__ == "__main__":
    unittest.main()
