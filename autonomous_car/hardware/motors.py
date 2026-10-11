"""
Proportional Motor Controller for Raspberry Pi 3.
Provides true 360° proportional speed & steering control via RPi.GPIO PWM.

Hardware Pin Allocations (BCM):
  IN1 → GPIO 17  (Physical Pin 11, Left Motor)
  IN2 → GPIO 27  (Physical Pin 13, Left Motor)
  IN2_ALT → GPIO 18 (Physical Pin 12, Left Motor Auto-Mirror fallback)
  IN3 → GPIO 22  (Physical Pin 15, Right Motor)
  IN4 → GPIO 23  (Physical Pin 16, Right Motor)

Features:
  - Dual IN2 Drive: Drives BOTH GPIO 27 and GPIO 18 so left motor works
    regardless of whether wired to Pin 13 (GPIO 27) or Pin 12 (GPIO 18).
  - Configurable / Invertible Polarity: left_polarity (+1/-1), right_polarity (+1/-1)
  - Persistent Settings: Saves polarity and trim calibrations to motor_settings.json
  - Independent wheel test routines: left_fwd, left_rev, right_fwd, right_rev
"""

import os
import json
import time
import logging
import threading
from typing import Optional, Dict, Any, Tuple

logger = logging.getLogger(__name__)

_GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    pass

SETTINGS_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "motor_settings.json"
)


class MotorController:
    """
    4-wheel RC car motor controller with natural 360° proportional tank-mix steering.
    """

    PWM_FREQ: int = 100  # 100 Hz software PWM
    WATCHDOG_TIMEOUT_SEC: float = 0.6

    def __init__(
        self,
        in1_pin: int = 17,
        in2_pin: int = 27,
        in3_pin: int = 22,
        in4_pin: int = 23,
        in2_alt_pin: int = 18,
        auto_setup: bool = True,
    ) -> None:
        self.in1 = in1_pin
        self.in2 = in2_pin
        self.in3 = in3_pin
        self.in4 = in4_pin
        self.in2_alt = in2_alt_pin

        self.left_polarity: int = 1   # +1 or -1
        self.right_polarity: int = 1  # +1 or -1
        self.left_trim: float = 1.0   # 0.5 to 1.0
        self.right_trim: float = 1.0  # 0.5 to 1.0

        self.is_initialized = False
        self._lock = threading.Lock()
        self._watchdog_timer: Optional[threading.Timer] = None

        self._pwm_in1 = None
        self._pwm_in2 = None
        self._pwm_in2_alt = None
        self._pwm_in3 = None
        self._pwm_in4 = None

        self.load_settings()

        if auto_setup:
            self.setup_gpio()

    def load_settings(self) -> None:
        """Load persistent calibration and polarity settings from disk."""
        if os.path.exists(SETTINGS_FILE):
            try:
                with open(SETTINGS_FILE, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    self.left_polarity = int(data.get("left_polarity", 1))
                    self.right_polarity = int(data.get("right_polarity", 1))
                    self.left_trim = float(data.get("left_trim", 1.0))
                    self.right_trim = float(data.get("right_trim", 1.0))
            except Exception as e:
                logger.warning(f"Could not load motor settings: {e}")

    def save_settings(self) -> None:
        """Save persistent polarity and trim settings to disk."""
        try:
            with open(SETTINGS_FILE, "w", encoding="utf-8") as f:
                json.dump({
                    "left_polarity": self.left_polarity,
                    "right_polarity": self.right_polarity,
                    "left_trim": self.left_trim,
                    "right_trim": self.right_trim,
                    "in1_pin": self.in1,
                    "in2_pin": self.in2,
                    "in3_pin": self.in3,
                    "in4_pin": self.in4,
                    "in2_alt_pin": self.in2_alt,
                }, f, indent=2)
        except Exception as e:
            logger.error(f"Could not save motor settings: {e}")

    def setup_gpio(self) -> bool:
        if not _GPIO_AVAILABLE:
            print("[SIMULATION] RPi.GPIO not available.")
            return False
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)

            pins_to_setup = [self.in1, self.in2, self.in3, self.in4]
            if self.in2_alt and self.in2_alt not in pins_to_setup:
                pins_to_setup.append(self.in2_alt)

            for pin in pins_to_setup:
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

            # Initialize software PWM at 100 Hz
            self._pwm_in1 = GPIO.PWM(self.in1, self.PWM_FREQ)
            self._pwm_in2 = GPIO.PWM(self.in2, self.PWM_FREQ)
            self._pwm_in3 = GPIO.PWM(self.in3, self.PWM_FREQ)
            self._pwm_in4 = GPIO.PWM(self.in4, self.PWM_FREQ)

            self._pwm_in1.start(0)
            self._pwm_in2.start(0)
            self._pwm_in3.start(0)
            self._pwm_in4.start(0)

            if self.in2_alt:
                try:
                    self._pwm_in2_alt = GPIO.PWM(self.in2_alt, self.PWM_FREQ)
                    self._pwm_in2_alt.start(0)
                except Exception:
                    self._pwm_in2_alt = None

            self.is_initialized = True
            print(f"[HARDWARE READY - RPi.GPIO PWM] IN1={self.in1}, IN2={self.in2} (alt {self.in2_alt}), IN3={self.in3}, IN4={self.in4}")
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
            if self._pwm_in2_alt: self._pwm_in2_alt.ChangeDutyCycle(d2)  # Mirror to Pin 18 as well
            if self._pwm_in3: self._pwm_in3.ChangeDutyCycle(d3)
            if self._pwm_in4: self._pwm_in4.ChangeDutyCycle(d4)

    def _get_left_duties(self, speed: float) -> Tuple[float, float]:
        """Convert speed (-100 to +100) to left motor pin duties (d1, d2)."""
        duty = max(0.0, min(100.0, abs(speed))) * self.left_trim
        if speed > 0:
            return (0.0, duty) if self.left_polarity == 1 else (duty, 0.0)
        elif speed < 0:
            return (duty, 0.0) if self.left_polarity == 1 else (0.0, duty)
        return (0.0, 0.0)

    def _get_right_duties(self, speed: float) -> Tuple[float, float]:
        """Convert speed (-100 to +100) to right motor pin duties (d3, d4)."""
        duty = max(0.0, min(100.0, abs(speed))) * self.right_trim
        if speed > 0:
            return (0.0, duty) if self.right_polarity == 1 else (duty, 0.0)
        elif speed < 0:
            return (duty, 0.0) if self.right_polarity == 1 else (0.0, duty)
        return (0.0, 0.0)

    def forward(self, speed: float = 100.0) -> None:
        spd = max(0.0, min(100.0, speed))
        print(f">>> MOTOR: FORWARD {spd:.0f}% (Left Pol: {self.left_polarity}, Right Pol: {self.right_polarity})")
        d1, d2 = self._get_left_duties(spd)
        d3, d4 = self._get_right_duties(spd)
        self._set_duties(d1, d2, d3, d4)
        self._reset_watchdog()

    def backward(self, speed: float = 100.0) -> None:
        spd = max(0.0, min(100.0, speed))
        print(f">>> MOTOR: BACKWARD {spd:.0f}%")
        d1, d2 = self._get_left_duties(-spd)
        d3, d4 = self._get_right_duties(-spd)
        self._set_duties(d1, d2, d3, d4)
        self._reset_watchdog()

    def left(self, speed: float = 80.0) -> None:
        spd = max(0.0, min(100.0, speed))
        print(f">>> MOTOR: LEFT {spd:.0f}%")
        # Turn left: Left side reverse (-spd), Right side forward (+spd)
        d1, d2 = self._get_left_duties(-spd)
        d3, d4 = self._get_right_duties(spd)
        self._set_duties(d1, d2, d3, d4)
        self._reset_watchdog()

    def right(self, speed: float = 80.0) -> None:
        spd = max(0.0, min(100.0, speed))
        print(f">>> MOTOR: RIGHT {spd:.0f}%")
        # Turn right: Left side forward (+spd), Right side reverse (-spd)
        d1, d2 = self._get_left_duties(spd)
        d3, d4 = self._get_right_duties(-spd)
        self._set_duties(d1, d2, d3, d4)
        self._reset_watchdog()

    def stop(self) -> None:
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None
        self._set_duties(0.0, 0.0, 0.0, 0.0)

    def drive(self, throttle: float, steering: float) -> str:
        """
        True 360° proportional tank-mix drive.
        throttle: -1.0 (full reverse) to +1.0 (full forward)
        steering: -1.0 (full left) to +1.0 (full right)
        """
        throttle = max(-1.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))

        # Deadzone check
        if abs(throttle) < 0.10 and abs(steering) < 0.10:
            self.stop()
            return "STOPPED"

        # Differential Steering:
        # Steering LEFT (-x): Right motor speeds up forward, Left motor slows down/reverses -> Turns LEFT
        # Steering RIGHT (+x): Left motor speeds up forward, Right motor slows down/reverses -> Turns RIGHT
        left_speed = throttle + steering
        right_speed = throttle - steering

        max_val = max(abs(left_speed), abs(right_speed), 1.0)
        left_speed = (left_speed / max_val) * 100.0
        right_speed = (right_speed / max_val) * 100.0

        d1, d2 = self._get_left_duties(left_speed)
        d3, d4 = self._get_right_duties(right_speed)
        self._set_duties(d1, d2, d3, d4)
        self._reset_watchdog()

        if abs(throttle) >= abs(steering):
            return "FORWARD" if throttle > 0 else "REVERSE"
        else:
            return "TURN-LEFT" if steering < 0 else "TURN-RIGHT"

    # Calibration & Diagnostic Routines
    def flip_left_polarity(self) -> int:
        self.left_polarity = -1 if self.left_polarity == 1 else 1
        self.save_settings()
        print(f"[MOTOR CALIBRATION] Left polarity flipped to: {self.left_polarity}")
        return self.left_polarity

    def flip_right_polarity(self) -> int:
        self.right_polarity = -1 if self.right_polarity == 1 else 1
        self.save_settings()
        print(f"[MOTOR CALIBRATION] Right polarity flipped to: {self.right_polarity}")
        return self.right_polarity

    def set_trim(self, left_trim: Optional[float] = None, right_trim: Optional[float] = None) -> None:
        if left_trim is not None:
            self.left_trim = max(0.1, min(1.0, float(left_trim)))
        if right_trim is not None:
            self.right_trim = max(0.1, min(1.0, float(right_trim)))
        self.save_settings()

    def get_config(self) -> Dict[str, Any]:
        return {
            "left_polarity": self.left_polarity,
            "right_polarity": self.right_polarity,
            "left_trim": self.left_trim,
            "right_trim": self.right_trim,
            "in1_pin": self.in1,
            "in2_pin": self.in2,
            "in3_pin": self.in3,
            "in4_pin": self.in4,
            "in2_alt_pin": self.in2_alt,
        }

    def test_pulse(self, action: str, duration: float = 1.2, speed: float = 90.0) -> None:
        """Run a test motion pulse for diagnosing individual wheel direction."""
        action = action.lower()
        if action == "left_fwd":
            d1, d2 = self._get_left_duties(speed)
            self._set_duties(d1, d2, 0.0, 0.0)
        elif action == "left_rev":
            d1, d2 = self._get_left_duties(-speed)
            self._set_duties(d1, d2, 0.0, 0.0)
        elif action == "right_fwd":
            d3, d4 = self._get_right_duties(speed)
            self._set_duties(0.0, 0.0, d3, d4)
        elif action == "right_rev":
            d3, d4 = self._get_right_duties(-speed)
            self._set_duties(0.0, 0.0, d3, d4)
        elif action == "both_fwd":
            self.forward(speed)
        elif action == "both_rev":
            self.backward(speed)

        def _auto_stop():
            time.sleep(duration)
            self.stop()

        t = threading.Thread(target=_auto_stop, daemon=True)
        t.start()

    def cleanup(self) -> None:
        self.stop()
        time.sleep(0.1)
        if self.is_initialized and _GPIO_AVAILABLE:
            for p in (self._pwm_in1, self._pwm_in2, self._pwm_in2_alt, self._pwm_in3, self._pwm_in4):
                if p:
                    try: p.stop()
                    except Exception: pass
            pins = [self.in1, self.in2, self.in3, self.in4]
            if self.in2_alt: pins.append(self.in2_alt)
            GPIO.cleanup(pins)
            self.is_initialized = False
            print("[MOTORS] GPIO released.")
