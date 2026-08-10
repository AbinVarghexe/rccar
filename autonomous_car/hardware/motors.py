"""
Proportional Motor Controller for Raspberry Pi 3.
Provides true 360° proportional speed & steering control via RPi.GPIO PWM.

Pin Allocations (BCM):
  IN1 → GPIO 17  (Left  motors reverse)
  IN2 → GPIO 18  (Left  motors forward)
  IN3 → GPIO 22  (Right motors reverse)
  IN4 → GPIO 23  (Right motors forward)
"""

import time
import logging
import threading
from typing import Optional

logger = logging.getLogger(__name__)

_GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    pass


class MotorController:
    """
    4-wheel RC car motor controller with inverted steering orientation
    matching physical wheel wiring.
    """

    PWM_FREQ: int = 100  # 100 Hz software PWM
    WATCHDOG_TIMEOUT_SEC: float = 0.6

    def __init__(
        self,
        in1_pin: int = 17,
        in2_pin: int = 18,
        in3_pin: int = 22,
        in4_pin: int = 23,
        auto_setup: bool = True,
    ) -> None:
        self.in1 = in1_pin
        self.in2 = in2_pin
        self.in3 = in3_pin
        self.in4 = in4_pin
        self.is_initialized = False
        self._lock = threading.Lock()
        self._watchdog_timer: Optional[threading.Timer] = None

        self._pwm_in1 = None
        self._pwm_in2 = None
        self._pwm_in3 = None
        self._pwm_in4 = None

        if auto_setup:
            self.setup_gpio()

    def setup_gpio(self) -> bool:
        if not _GPIO_AVAILABLE:
            print("[SIMULATION] RPi.GPIO not available.")
            return False
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            for pin in (self.in1, self.in2, self.in3, self.in4):
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

            # Initialize software PWM at 100 Hz
            self._pwm_in1 = GPIO.PWM(self.in1, self.PWM_FREQ)
            self._pwm_in2 = GPIO.PWM(self.in2, self.PWM_FREQ)
            self._pwm_in3 = GPIO.PWM(self.in3, self.PWM_FREQ)
            self._pwm_in4 = GPIO.PWM(self.in4, self.PWM_FREQ)

            for p in (self._pwm_in1, self._pwm_in2, self._pwm_in3, self._pwm_in4):
                p.start(0)

            self.is_initialized = True
            print(f"[HARDWARE READY - RPi.GPIO 100Hz PWM] BCM: IN1={self.in1}, IN2={self.in2}, IN3={self.in3}, IN4={self.in4}")
            return True
        except Exception as e:
            logger.error(f"GPIO setup failed: {e}")
            return False

    def _reset_watchdog(self) -> None:
        """Reset auto-stop watchdog timer."""
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
        self._watchdog_timer = threading.Timer(self.WATCHDOG_TIMEOUT_SEC, self.stop)
        self._watchdog_timer.daemon = True
        self._watchdog_timer.start()

    def _set_duties(self, d1: float, d2: float, d3: float, d4: float) -> None:
        """Set duty cycles (0.0 to 100.0) for each motor pin."""
        if not self.is_initialized or not _GPIO_AVAILABLE:
            return
        d1 = max(0.0, min(100.0, d1))
        d2 = max(0.0, min(100.0, d2))
        d3 = max(0.0, min(100.0, d3))
        d4 = max(0.0, min(100.0, d4))

        with self._lock:
            if self._pwm_in1: self._pwm_in1.ChangeDutyCycle(d1)
            if self._pwm_in2: self._pwm_in2.ChangeDutyCycle(d2)
            if self._pwm_in3: self._pwm_in3.ChangeDutyCycle(d3)
            if self._pwm_in4: self._pwm_in4.ChangeDutyCycle(d4)

    def forward(self, speed: float = 100.0) -> None:
        spd = max(0.0, min(100.0, speed))
        print(f">>> MOTOR: FORWARD {spd:.0f}%")
        self._set_duties(0.0, spd, 0.0, spd)
        self._reset_watchdog()

    def backward(self, speed: float = 100.0) -> None:
        spd = max(0.0, min(100.0, speed))
        print(f">>> MOTOR: BACKWARD {spd:.0f}%")
        self._set_duties(spd, 0.0, spd, 0.0)
        self._reset_watchdog()

    def left(self, speed: float = 80.0) -> None:
        spd = max(0.0, min(100.0, speed))
        print(f">>> MOTOR: LEFT {spd:.0f}%")
        # Left side forward (IN2), Right side reverse (IN3) -> Turn LEFT
        self._set_duties(0.0, spd, spd, 0.0)
        self._reset_watchdog()

    def right(self, speed: float = 80.0) -> None:
        spd = max(0.0, min(100.0, speed))
        print(f">>> MOTOR: RIGHT {spd:.0f}%")
        # Left side reverse (IN1), Right side forward (IN4) -> Turn RIGHT
        self._set_duties(spd, 0.0, 0.0, spd)
        self._reset_watchdog()

    def stop(self) -> None:
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None
        self._set_duties(0.0, 0.0, 0.0, 0.0)

    def drive(self, throttle: float, steering: float) -> str:
        """
        True 360° proportional tank-mix drive with inverted steering orientation.
        throttle: -1.0 (full reverse) to +1.0 (full forward)
        steering: -1.0 (full left) to +1.0 (full right)
        """
        throttle = max(-1.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))

        # Deadzone check
        if abs(throttle) < 0.12 and abs(steering) < 0.12:
            self.stop()
            return "STOPPED"

        # Invert steering sign to match physical wiring orientation
        effective_steering = -steering

        # Tank-mix formula:
        # Pushing joystick LEFT (-x) -> effective_steering > 0 -> left_speed > right_speed -> Left wheel forward, Right wheel reverse -> Turns LEFT
        # Pushing joystick RIGHT (+x) -> effective_steering < 0 -> right_speed > left_speed -> Right wheel forward, Left wheel reverse -> Turns RIGHT
        left_speed = throttle + effective_steering
        right_speed = throttle - effective_steering

        # Normalize so neither exceeds ±1.0
        max_val = max(abs(left_speed), abs(right_speed), 1.0)
        left_speed /= max_val
        right_speed /= max_val

        def get_duties(val: float):
            duty = abs(val) * 100.0
            if val > 0:
                return 0.0, duty   # forward: (rev=0, fwd=duty)
            elif val < 0:
                return duty, 0.0   # reverse: (rev=duty, fwd=0)
            else:
                return 0.0, 0.0

        left_rev, left_fwd = get_duties(left_speed)
        right_rev, right_fwd = get_duties(right_speed)

        # IN1=left_rev, IN2=left_fwd, IN3=right_rev, IN4=right_fwd
        self._set_duties(left_rev, left_fwd, right_rev, right_fwd)
        self._reset_watchdog()

        if abs(throttle) >= abs(steering):
            return "FORWARD" if throttle > 0 else "REVERSE"
        else:
            return "TURN-LEFT" if steering < 0 else "TURN-RIGHT"

    def cleanup(self) -> None:
        self.stop()
        time.sleep(0.1)
        if self.is_initialized and _GPIO_AVAILABLE:
            for p in (self._pwm_in1, self._pwm_in2, self._pwm_in3, self._pwm_in4):
                if p:
                    try: p.stop()
                    except Exception: pass
            GPIO.cleanup([self.in1, self.in2, self.in3, self.in4])
            self.is_initialized = False
            print("[MOTORS] GPIO released.")
