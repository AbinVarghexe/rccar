"""
Servo Motor Control — Raspberry Pi 3.
Uses RPi.GPIO hardware PWM on GPIO 12 (Pin 32, PWM0).
Hardware PWM = ZERO jitter. No gpiozero needed.

Servo timing (standard):
  50 Hz  →  20 ms period
  0°   = 0.5 ms pulse  = 2.5%  duty
  90°  = 1.5 ms pulse  = 7.5%  duty
  180° = 2.5 ms pulse  = 12.5% duty
"""

import time
import logging
from typing import Optional

logger = logging.getLogger(__name__)


class ServoController:
    """
    Jitter-free Pan Servo using RPi.GPIO Hardware PWM.
    GPIO 12 (Physical Pin 32) — Hardware PWM0 on Pi 3.
    Angle range: 0° to 180°.  Center: 90°.
    """

    SERVO_PIN    : int   = 12
    PWM_FREQ     : int   = 50       # Hz — standard servo frequency
    DUTY_MIN     : float = 2.5      # % — 0.5ms pulse  → 0°
    DUTY_MAX     : float = 12.5     # % — 2.5ms pulse  → 180°
    MIN_ANGLE    : float = 0.0
    MAX_ANGLE    : float = 180.0
    CENTER_ANGLE : float = 90.0

    def __init__(self, servo_pin: int = 12, auto_setup: bool = True) -> None:
        self.servo_pin       : int            = servo_pin
        self._pwm            : Optional[object] = None   # RPi.GPIO PWM instance
        self._current_angle  : float          = self.CENTER_ANGLE
        self.is_initialized  : bool           = False
        self._use_gpio       : bool           = False

        try:
            import RPi.GPIO as GPIO
            self._use_gpio = True
        except ImportError:
            logger.warning("[SERVO] RPi.GPIO not available — servo in simulation mode.")

        if auto_setup:
            self.setup()

    # ------------------------------------------------------------------
    def setup(self) -> bool:
        if not self._use_gpio:
            logger.warning("[SERVO] Simulation mode — no hardware PWM.")
            return False
        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.servo_pin, GPIO.OUT)

            self._pwm = GPIO.PWM(self.servo_pin, self.PWM_FREQ)
            # Start at center (90°)
            self._pwm.start(self._angle_to_duty(self.CENTER_ANGLE))
            time.sleep(0.3)

            self.is_initialized  = True
            self._current_angle  = self.CENTER_ANGLE
            print(f"[SERVO READY] GPIO {self.servo_pin} (Pin 32) — Hardware PWM {self.PWM_FREQ}Hz — Center 90°")
            return True
        except Exception as e:
            logger.error(f"[SERVO] Init failed on GPIO {self.servo_pin}: {e}")
            return False

    # ------------------------------------------------------------------
    def _angle_to_duty(self, angle: float) -> float:
        """Convert angle (0–180) to duty cycle (2.5–12.5)."""
        angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, float(angle)))
        return self.DUTY_MIN + (angle / self.MAX_ANGLE) * (self.DUTY_MAX - self.DUTY_MIN)

    # ------------------------------------------------------------------
    def set_angle(self, angle: float) -> None:
        """Move servo to given angle (0–180°)."""
        angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, float(angle)))
        self._current_angle = angle
        duty = self._angle_to_duty(angle)

        print(f">>> SERVO: {angle:.1f}° → duty {duty:.2f}%")
        if self.is_initialized and self._pwm:
            self._pwm.ChangeDutyCycle(duty)
        else:
            print(f"[SIMULATION] Servo → {angle:.1f}°")

    def center(self) -> None:
        self.set_angle(self.CENTER_ANGLE)

    def pan_left(self) -> None:
        self.set_angle(self.MIN_ANGLE)

    def pan_right(self) -> None:
        self.set_angle(self.MAX_ANGLE)

    @property
    def current_angle(self) -> float:
        return self._current_angle

    # ------------------------------------------------------------------
    def cleanup(self) -> None:
        """Center servo then release GPIO PWM."""
        self.center()
        time.sleep(0.3)
        if self.is_initialized and self._pwm:
            self._pwm.stop()
            self.is_initialized = False
            print("[SERVO] GPIO PWM released.")
