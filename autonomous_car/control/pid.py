"""
PID Controller for lane tracking steering alignment.
"""

import os
import sys
import time
from typing import Optional

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from config import PIDConfig
    from utils.logger import car_logger
except ImportError:
    from autonomous_car.config import PIDConfig
    from autonomous_car.utils.logger import car_logger

class PIDController:
    def __init__(self, config: PIDConfig) -> None:
        self.kp: float = config.kp
        self.ki: float = config.ki
        self.kd: float = config.kd
        self.setpoint: float = config.setpoint
        self.output_min: float = config.output_min
        self.output_max: float = config.output_max
        self.integral_max: float = config.integral_max

        self._integral: float = 0.0
        self._last_error: float = 0.0
        self._last_time: Optional[float] = None

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

    def compute(self, current_value: float, dt: Optional[float] = None) -> float:
        now = time.perf_counter()
        if dt is None:
            if self._last_time is None:
                dt = 0.033
            else:
                dt = now - self._last_time
        
        self._last_time = now
        if dt <= 0.0:
            dt = 0.001

        error = current_value - self.setpoint
        p_term = self.kp * error

        self._integral += error * dt
        self._integral = max(-self.integral_max, min(self.integral_max, self._integral))
        i_term = self.ki * self._integral

        d_term = self.kd * ((error - self._last_error) / dt)
        self._last_error = error

        output = p_term + i_term + d_term
        clamped_output = max(self.output_min, min(self.output_max, output))
        return clamped_output
