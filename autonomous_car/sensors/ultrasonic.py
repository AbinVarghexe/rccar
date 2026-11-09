"""
sensors/ultrasonic.py - HC-SR04 Ultrasonic Distance Sensor Driver.

Hardware Wiring (Raspberry Pi 4 BCM GPIO):
- TRIG Pin: BCM GPIO 24 (Physical Pin 18)
- ECHO Pin: BCM GPIO 25 (Physical Pin 22)
  * CRITICAL: ECHO is 5V TTL and MUST pass through a voltage divider
    (1k ohm series, 2k ohm to GND) to produce safe 3.3V logic for the Pi.

Reliability & Safety Principles:
1. Bounded hardware pulse measurement timeout (default 30ms ~ 5m max flight).
2. Explicit distinction between "Path Clear" and "Sensor Fault/Timeout".
   A timeout NEVER returns 400cm or is_valid=True.
3. 3-sample median spike filter for active physical reads.
4. Health metrics tracking (total reads, timeout count, error rate).
5. Mock injection support for hardware-free simulation and unit testing.
"""

import os
import sys
import time
from dataclasses import dataclass
from typing import Optional, List, Dict, Any

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from config import SensorConfig
    from utils.logger import car_logger
except ImportError:
    from autonomous_car.config import SensorConfig
    from autonomous_car.utils.logger import car_logger


@dataclass
class SingleSensorData:
    distance_cm: Optional[float]
    obstacle_warning: bool
    emergency_stop: bool
    is_valid: bool = True
    timestamp: float = 0.0
    status: str = "OK"  # "OK", "TIMEOUT", "OUT_OF_RANGE", "DISABLED", "FAULT"


@dataclass
class DualUltrasonicData:
    front: SingleSensorData
    rear: SingleSensorData
    emergency_stop_triggered: bool
    timestamp: float = 0.0


class UltrasonicSensors:
    """
    Production HC-SR04 Ultrasonic sensor driver with strict validation.
    """

    def __init__(self, config: Optional[SensorConfig] = None, serial_comm=None, mock_mode: bool = False) -> None:
        self.config: SensorConfig = config or SensorConfig()
        self.serial_comm = serial_comm
        self.mock_mode: bool = mock_mode
        self._mock_front_distance: Optional[float] = None
        self._mock_rear_distance: Optional[float] = None
        self._mock_front_valid: bool = True

        self._gpio_available: bool = False
        self._gpio_module = None
        self._enabled: bool = bool(self.config.enabled)

        # Health metrics
        self.total_reads: int = 0
        self.failed_reads: int = 0
        self.consecutive_timeouts: int = 0
        self.last_valid_time: float = 0.0

        if not self.mock_mode and self._enabled:
            self._init_gpio()
        elif not self._enabled:
            car_logger.info(
                "Ultrasonic Sensor is in Standby Mode (disabled in config). "
                "Spatial perception active via camera."
            )

    def is_enabled(self) -> bool:
        return self._enabled

    def enable(self) -> bool:
        self._enabled = True
        if self.mock_mode:
            return True
        return self._init_gpio()

    def disable(self) -> None:
        self._enabled = False
        self.cleanup()
        car_logger.info("Ultrasonic Sensor placed in standby.")

    def set_mock_distance(self, front_cm: Optional[float], rear_cm: Optional[float] = None, is_valid: bool = True) -> None:
        """Inject synthetic distances for unit testing and dry-run simulation."""
        self.mock_mode = True
        self._mock_front_distance = front_cm
        self._mock_rear_distance = rear_cm
        self._mock_front_valid = is_valid

    def _init_gpio(self) -> bool:
        if not self._enabled or self.mock_mode:
            return False

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            # Front Sensor (Primary)
            GPIO.setup(self.config.front_trig_pin, GPIO.OUT)
            GPIO.setup(self.config.front_echo_pin, GPIO.IN)
            GPIO.output(self.config.front_trig_pin, False)

            # Optional Rear Sensor
            if self.config.rear_trig_pin is not None and self.config.rear_echo_pin is not None:
                GPIO.setup(self.config.rear_trig_pin, GPIO.OUT)
                GPIO.setup(self.config.rear_echo_pin, GPIO.IN)
                GPIO.output(self.config.rear_trig_pin, False)

            self._gpio_module = GPIO
            self._gpio_available = True
            car_logger.info(
                f"Initialized HC-SR04 Ultrasonic on BCM GPIO "
                f"(TRIG: {self.config.front_trig_pin}, ECHO: {self.config.front_echo_pin})"
            )
            time.sleep(0.05)
            return True
        except Exception as e:
            self._gpio_available = False
            car_logger.warning(f"RPi.GPIO not accessible for Ultrasonic ({e}). Operating in Standby/Simulation.")
            return False

    def _measure_pulse(self, trig_pin: int, echo_pin: int) -> Optional[float]:
        """
        Executes bounded 10us trigger pulse and measures echo flight duration.
        Returns float distance in cm, or None if timed out / invalid pulse.
        """
        if not self._gpio_available or self._gpio_module is None:
            return None

        GPIO = self._gpio_module

        # Trigger 10 microsecond pulse
        GPIO.output(trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(trig_pin, False)

        init_time = time.perf_counter()
        timeout = getattr(self.config, 'sensor_timeout_sec', 0.03)

        # 1. Wait for echo to go HIGH
        pulse_start = time.perf_counter()
        while GPIO.input(echo_pin) == 0:
            pulse_start = time.perf_counter()
            if pulse_start - init_time > timeout:
                return None  # Timed out waiting for echo response

        # 2. Wait for echo to go LOW
        pulse_stop = pulse_start
        while GPIO.input(echo_pin) == 1:
            pulse_stop = time.perf_counter()
            if pulse_stop - pulse_start > timeout:
                return None  # Pulse exceeded max range timeout

        elapsed = pulse_stop - pulse_start
        # Speed of sound in dry air at 20°C: ~343 m/s = 34300 cm/s
        distance = (elapsed * 34300.0) / 2.0

        # Physical validation for HC-SR04: 2.0 cm to 400.0 cm
        if 2.0 <= distance <= 400.0:
            return round(distance, 1)
        return None

    def get_front_distance(self) -> Optional[float]:
        """
        Returns front distance in cm using 3-sample median filter.
        Returns None if sensor is disabled, failed, or timed out.
        """
        if self.mock_mode:
            if not self._mock_front_valid:
                return None
            return self._mock_front_distance

        if not self._enabled or not self._gpio_available:
            return None

        samples: List[float] = []
        for _ in range(3):
            d = self._measure_pulse(self.config.front_trig_pin, self.config.front_echo_pin)
            if d is not None:
                samples.append(d)
            time.sleep(0.002)

        if not samples:
            return None

        samples.sort()
        return samples[len(samples) // 2]

    def read_all(self) -> DualUltrasonicData:
        """
        Acquires front and rear ultrasonic measurements with complete validation.
        Missing or timed out readings NEVER masquerade as 400cm clear paths.
        """
        now = time.time()
        self.total_reads += 1

        # If sensor is explicitly disabled or hardware is unavailable in simulation
        if not self._enabled or (not self._gpio_available and not self.mock_mode):
            safe_front = SingleSensorData(
                distance_cm=None,
                obstacle_warning=False,
                emergency_stop=False,
                is_valid=False,
                timestamp=now,
                status="DISABLED"
            )
            safe_rear = SingleSensorData(
                distance_cm=None,
                obstacle_warning=False,
                emergency_stop=False,
                is_valid=False,
                timestamp=now,
                status="DISABLED"
            )
            return DualUltrasonicData(
                front=safe_front,
                rear=safe_rear,
                emergency_stop_triggered=False,
                timestamp=now
            )

        # Read Front Sensor
        front_dist = self.get_front_distance()

        if front_dist is None:
            self.failed_reads += 1
            self.consecutive_timeouts += 1
            front_data = SingleSensorData(
                distance_cm=None,
                obstacle_warning=False,
                emergency_stop=False,
                is_valid=False,
                timestamp=now,
                status="TIMEOUT"
            )
        else:
            self.consecutive_timeouts = 0
            self.last_valid_time = now
            front_warn = front_dist <= self.config.warning_distance_cm
            front_stop = front_dist <= self.config.emergency_stop_distance_cm
            front_data = SingleSensorData(
                distance_cm=front_dist,
                obstacle_warning=front_warn,
                emergency_stop=front_stop,
                is_valid=True,
                timestamp=now,
                status="OK"
            )

        # Read Optional Rear Sensor
        rear_dist = None
        if self.mock_mode:
            rear_dist = self._mock_rear_distance
        elif self._gpio_available and self.config.rear_trig_pin is not None and self.config.rear_echo_pin is not None:
            rear_dist = self._measure_pulse(self.config.rear_trig_pin, self.config.rear_echo_pin)

        if rear_dist is None:
            rear_data = SingleSensorData(
                distance_cm=None,
                obstacle_warning=False,
                emergency_stop=False,
                is_valid=False if self.config.rear_trig_pin is not None else False,
                timestamp=now,
                status="NO_SENSOR" if self.config.rear_trig_pin is None else "TIMEOUT"
            )
        else:
            rear_warn = rear_dist <= self.config.warning_distance_cm
            rear_stop = rear_dist <= self.config.emergency_stop_distance_cm
            rear_data = SingleSensorData(
                distance_cm=rear_dist,
                obstacle_warning=rear_warn,
                emergency_stop=rear_stop,
                is_valid=True,
                timestamp=now,
                status="OK"
            )

        return DualUltrasonicData(
            front=front_data,
            rear=rear_data,
            emergency_stop_triggered=front_data.emergency_stop or rear_data.emergency_stop,
            timestamp=now
        )

    def get_health(self) -> Dict[str, Any]:
        """Returns diagnostic health metrics for the ultrasonic subsystem."""
        error_rate = (self.failed_reads / self.total_reads) if self.total_reads > 0 else 0.0
        return {
            "enabled": self._enabled,
            "gpio_available": self._gpio_available or self.mock_mode,
            "mock_mode": self.mock_mode,
            "total_reads": self.total_reads,
            "failed_reads": self.failed_reads,
            "consecutive_timeouts": self.consecutive_timeouts,
            "error_rate": round(error_rate, 3),
            "last_valid_time": self.last_valid_time,
            "is_healthy": (self.consecutive_timeouts < 5) and (error_rate < 0.40 or self.total_reads < 10)
        }

    def cleanup(self) -> None:
        if self._gpio_available and self._gpio_module is not None:
            try:
                pins = [self.config.front_trig_pin, self.config.front_echo_pin]
                if self.config.rear_trig_pin is not None and self.config.rear_echo_pin is not None:
                    pins.extend([self.config.rear_trig_pin, self.config.rear_echo_pin])
                self._gpio_module.cleanup(pins)
            except Exception:
                pass
            self._gpio_available = False
