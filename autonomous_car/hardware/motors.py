"""
Motor Controller for Raspberry Pi 3 — RPi.GPIO with Software PWM Speed Control.

Pin Allocations (BCM):
  IN1 → GPIO 17  (Left  motors direction A)
  IN2 → GPIO 18  (Left  motors direction B)
  IN3 → GPIO 22  (Right motors direction A)
  IN4 → GPIO 23  (Right motors direction B)

Speed control is done via RPi.GPIO software PWM on the direction pins,
giving proportional 0–100% duty for smooth graduated motor response.
"""

import time
import logging
import threading

logger = logging.getLogger(__name__)

_GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    pass


class MotorController:
    """
    4-wheel RC car motor controller with proportional speed PWM.
    Supports both discrete commands (forward/backward/left/right/stop)
    and proportional mixed commands via drive(speed, turn).
    """

    PWM_FREQ : int = 1000   # Hz for software PWM on direction pins

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

        # PWM objects (created after setup)
        self._pwm_in1 = None
        self._pwm_in2 = None
        self._pwm_in3 = None
        self._pwm_in4 = None

        if auto_setup:
            self.setup_gpio()

    # ------------------------------------------------------------------
    def setup_gpio(self) -> bool:
        if not _GPIO_AVAILABLE:
            print("[SIMULATION] RPi.GPIO not available.")
            return False
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            for pin in (self.in1, self.in2, self.in3, self.in4):
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

            # Create software PWM on each pin (1 kHz)
            self._pwm_in1 = GPIO.PWM(self.in1, self.PWM_FREQ)
            self._pwm_in2 = GPIO.PWM(self.in2, self.PWM_FREQ)
            self._pwm_in3 = GPIO.PWM(self.in3, self.PWM_FREQ)
            self._pwm_in4 = GPIO.PWM(self.in4, self.PWM_FREQ)

            for p in (self._pwm_in1, self._pwm_in2, self._pwm_in3, self._pwm_in4):
                p.start(0)

            self.is_initialized = True
            print(f"[HARDWARE READY - RPi.GPIO PWM] BCM: IN1={self.in1}, IN2={self.in2}, IN3={self.in3}, IN4={self.in4}")
            return True
        except Exception as e:
            logger.error(f"GPIO setup failed: {e}")
            return False

    # ------------------------------------------------------------------
    def _set_pwm(self, d1: float, d2: float, d3: float, d4: float) -> None:
        """Set duty cycles (0–100) for each direction pin."""
        if not self.is_initialized:
            return
        with self._lock:
            self._pwm_in1.ChangeDutyCycle(max(0, min(100, d1)))
            self._pwm_in2.ChangeDutyCycle(max(0, min(100, d2)))
            self._pwm_in3.ChangeDutyCycle(max(0, min(100, d3)))
            self._pwm_in4.ChangeDutyCycle(max(0, min(100, d4)))

    # ------------------------------------------------------------------
    # Discrete commands (100% speed)
    # ------------------------------------------------------------------
    def forward(self, speed: float = 100.0) -> None:
        print(f">>> MOTOR: FORWARD {speed:.0f}%")
        self._set_pwm(0, speed, 0, speed)

    def backward(self, speed: float = 100.0) -> None:
        print(f">>> MOTOR: BACKWARD {speed:.0f}%")
        self._set_pwm(speed, 0, speed, 0)

    def left(self, speed: float = 80.0) -> None:
        """Left motors reverse, right motors forward."""
        print(f">>> MOTOR: LEFT {speed:.0f}%")
        self._set_pwm(speed, 0, 0, speed)

    def right(self, speed: float = 80.0) -> None:
        """Right motors reverse, left motors forward."""
        print(f">>> MOTOR: RIGHT {speed:.0f}%")
        self._set_pwm(0, speed, speed, 0)

    def stop(self) -> None:
        print(">>> MOTOR: STOP")
        self._set_pwm(0, 0, 0, 0)

    # ------------------------------------------------------------------
    # Proportional drive — used by joystick
    # ------------------------------------------------------------------
    def drive(self, throttle: float, steering: float) -> str:
        """
        Smooth joystick/proportional drive.

        throttle : -1.0 (full reverse) to +1.0 (full forward)
        steering : -1.0 (full left)    to +1.0 (full right)

        Returns a status string describing the motion.
        """
        throttle = max(-1.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))

        # Tank-mix: left_speed and right_speed in [-1, +1]
        left_speed  = throttle - steering
        right_speed = throttle + steering

        # Normalise so neither exceeds ±1
        max_val = max(abs(left_speed), abs(right_speed), 1.0)
        left_speed  /= max_val
        right_speed /= max_val

        # Convert to duty cycles for each side
        def _duties(spd: float):
            """Returns (fwd_duty, rev_duty) for a given speed [-1..+1]."""
            duty = abs(spd) * 100.0
            if spd > 0:
                return duty, 0.0    # forward
            elif spd < 0:
                return 0.0, duty    # reverse
            else:
                return 0.0, 0.0    # stop

        lf, lr = _duties(left_speed)
        rf, rr = _duties(right_speed)

        # IN1/IN2 → left side  |  IN3/IN4 → right side
        # Forward:  IN2=HIGH, IN1=LOW  (matches motors.forward() logic)
        # Reverse:  IN1=HIGH, IN2=LOW
        self._set_pwm(lr, lf, rr, rf)

        # Build human-readable status
        if abs(throttle) < 0.12 and abs(steering) < 0.12:
            return "STOPPED"
        if abs(throttle) >= abs(steering):
            return "FORWARD" if throttle > 0 else "REVERSE"
        return "TURN-LEFT" if steering < 0 else "TURN-RIGHT"

    # ------------------------------------------------------------------
    def cleanup(self) -> None:
        self.stop()
        time.sleep(0.1)
        if self.is_initialized:
            for p in (self._pwm_in1, self._pwm_in2, self._pwm_in3, self._pwm_in4):
                if p:
                    p.stop()
            GPIO.cleanup([self.in1, self.in2, self.in3, self.in4])
            self.is_initialized = False
            print("[MOTORS] GPIO released.")
