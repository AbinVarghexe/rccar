"""
control/motor_controller.py - Modular export for MotorController.
Provides clean alias to hardware.motors.MotorController.
"""

import os
import sys

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

try:
    from hardware.motors import MotorController
except ImportError:
    from autonomous_car.hardware.motors import MotorController

__all__ = ["MotorController"]
