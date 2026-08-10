"""
Ultrasonic Sensor module supporting:
1. Arduino USB Serial distance readings (Sensors wired directly to Arduino UNO)
2. Native Raspberry Pi GPIO readings
3. Dev Mock fallback mode
"""

import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, Tuple

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from config import SensorConfig
    from utils.logger import car_logger
    from control.serial_comm import SerialCommunicator
except ImportError:
    from autonomous_car.config import SensorConfig
    from autonomous_car.utils.logger import car_logger
    from autonomous_car.control.serial_comm import SerialCommunicator

@dataclass
class SingleSensorData:
    distance_cm: float
    obstacle_warning: bool
    emergency_stop: bool

@dataclass
class DualUltrasonicData:
    front: SingleSensorData
    rear: SingleSensorData
    emergency_stop_triggered: bool

class UltrasonicSensors:
    def __init__(self, config: SensorConfig, serial_comm: Optional[SerialCommunicator] = None) -> None:
        self.config: SensorConfig = config
        self.serial_comm: Optional[SerialCommunicator] = serial_comm
        self._gpio_available: bool = False
        self._gpio_module = None

        if self.serial_comm is None or not self.serial_comm.is_connected:
            self._init_gpio()

    def _init_gpio(self) -> None:
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            GPIO.setup(self.config.front_trig_pin, GPIO.OUT)
            GPIO.setup(self.config.front_echo_pin, GPIO.IN)
            GPIO.output(self.config.front_trig_pin, False)

            GPIO.setup(self.config.rear_trig_pin, GPIO.OUT)
            GPIO.setup(self.config.rear_echo_pin, GPIO.IN)
            GPIO.output(self.config.rear_trig_pin, False)

            self._gpio_module = GPIO
            self._gpio_available = True
            car_logger.info("Initialized RPi.GPIO hardware interface for Ultrasonic Sensors.")
            time.sleep(0.1)
        except Exception as e:
            self._gpio_available = False
            car_logger.warning(f"GPIO not available ({e}). Ultrasonic Sensors using Arduino Serial / Dev Mock Mode.")

    def read_all(self) -> DualUltrasonicData:
        front_dist = 120.0
        rear_dist = 150.0

        # Option 1: Primary - Read distance sent over USB Serial from Arduino UNO
        if self.serial_comm and self.serial_comm.is_connected:
            front_dist, rear_dist = self.serial_comm.get_ultrasonic_distances()

        # Option 2: Fallback to Pi GPIO
        elif self._gpio_available:
            front_dist = self._measure_distance_gpio(self.config.front_trig_pin, self.config.front_echo_pin)
            rear_dist = self._measure_distance_gpio(self.config.rear_trig_pin, self.config.rear_echo_pin)

        front_warn = front_dist <= self.config.warning_distance_cm
        front_stop = front_dist <= self.config.emergency_stop_distance_cm

        rear_warn = rear_dist <= self.config.warning_distance_cm
        rear_stop = rear_dist <= self.config.emergency_stop_distance_cm

        front_data = SingleSensorData(distance_cm=front_dist, obstacle_warning=front_warn, emergency_stop=front_stop)
        rear_data = SingleSensorData(distance_cm=rear_dist, obstacle_warning=rear_warn, emergency_stop=rear_stop)

        return DualUltrasonicData(
            front=front_data,
            rear=rear_data,
            emergency_stop_triggered=front_stop or rear_stop
        )

    def _measure_distance_gpio(self, trig_pin: int, echo_pin: int) -> float:
        if not self._gpio_available or self._gpio_module is None:
            return 100.0

        GPIO = self._gpio_module
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)

        start_time = time.time()
        stop_time = time.time()
        timeout = self.config.sensor_timeout_sec
        init_time = time.time()

        while GPIO.input(echo_pin) == 0:
            start_time = time.time()
            if start_time - init_time > timeout:
                return 400.0

        while GPIO.input(echo_pin) == 1:
            stop_time = time.time()
            if stop_time - start_time > timeout:
                return 400.0

        elapsed = stop_time - start_time
        distance = (elapsed * 34300.0) / 2.0
        return round(distance, 2)

    def cleanup(self) -> None:
        if self._gpio_available and self._gpio_module is None:
            try:
                self._gpio_module.cleanup()
            except Exception:
                pass
