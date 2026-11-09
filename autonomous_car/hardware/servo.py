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
import threading
from typing import Optional

logger = logging.getLogger(__name__)


class ServoController:
    """
    Jitter-free Pan Servo using RPi.GPIO Hardware PWM.
    GPIO 12 (Physical Pin 32) — Hardware PWM0 on Pi 3 / Pi 4.
    Angle range: 0° to 180°.  Center: 90°.
    """

    SERVO_PIN    : int   = 12
    PWM_FREQ     : int   = 50       # Hz — standard servo frequency
    DUTY_MIN     : float = 2.5      # % — 0.5ms pulse  → 0°
    DUTY_MAX     : float = 12.5     # % — 2.5ms pulse  → 180°
    MIN_ANGLE    : float = 25.0     # Safe travel limit to prevent mechanical stall & burnout
    MAX_ANGLE    : float = 155.0    # Safe travel limit to prevent mechanical stall & burnout
    CENTER_ANGLE : float = 90.0

    def __init__(
        self,
        servo_pin: int = 12,
        auto_setup: bool = True,
        center_angle: float = CENTER_ANGLE,
    ) -> None:
        self.servo_pin       : int            = servo_pin
        self._pwm            : Optional[object] = None   # RPi.GPIO PWM instance
        self.center_angle    : float          = max(self.MIN_ANGLE, min(self.MAX_ANGLE, float(center_angle)))
        self._current_angle  : float          = self.center_angle
        self.trim_offset     : float          = 0.0
        self.is_initialized  : bool           = False
        self._use_gpio       : bool           = False
        self._lock           : threading.Lock = threading.Lock()
        self._relax_timer    : Optional[threading.Timer] = None

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
        with self._lock:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(self.servo_pin, GPIO.OUT)

                self._pwm = GPIO.PWM(self.servo_pin, self.PWM_FREQ)
                # Start at center (90°) with trim
                effective_center = max(self.MIN_ANGLE, min(self.MAX_ANGLE, self.center_angle + self.trim_offset))
                self._pwm.start(self._angle_to_duty(effective_center))
                self.is_initialized  = True
                self._current_angle  = self.center_angle
                logger.info(f"[SERVO READY] GPIO {self.servo_pin} (Pin 32) — Hardware PWM {self.PWM_FREQ}Hz — Center {self.center_angle:.1f}° (Trim {self.trim_offset:+.1f}°)")

                # Thermal protection: Auto-relax PWM duty cycle to 0 after settling at center
                self._relax_timer = threading.Timer(0.6, self._relax_pwm)
                self._relax_timer.daemon = True
                self._relax_timer.start()
                return True
            except Exception as e:
                logger.error(f"[SERVO] Init failed on GPIO {self.servo_pin}: {e}")
                return False

    # ------------------------------------------------------------------
    def _angle_to_duty(self, angle: float) -> float:
        """Convert angle (0–180) to duty cycle (2.5–12.5)."""
        clamped = max(0.0, min(180.0, float(angle)))
        return self.DUTY_MIN + (clamped / 180.0) * (self.DUTY_MAX - self.DUTY_MIN)

    # ------------------------------------------------------------------
    def set_trim(self, offset: float) -> None:
        """Set calibration trim offset in degrees (e.g. -30° to +30°)."""
        self.trim_offset = max(-30.0, min(30.0, float(offset)))
        logger.info(f"[SERVO] Trim offset set: {self.trim_offset:+.1f} deg")
        self.set_angle(self._current_angle, auto_relax=True)

    def _relax_pwm(self) -> None:
        """Internal callback to stop pulse generation after transit delay."""
        with self._lock:
            if self.is_initialized and self._pwm:
                try:
                    self._pwm.ChangeDutyCycle(0)
                except Exception:
                    pass

    def relax(self) -> None:
        """
        Immediately stops PWM pulse generation, de-energizing motor coils.
        Prevents motor heating, buzzing, and current draw while holding position.
        """
        with self._lock:
            if self._relax_timer is not None:
                self._relax_timer.cancel()
                self._relax_timer = None
        self._relax_pwm()
        logger.debug("[SERVO] PWM duty cycle relaxed to 0%. Coils de-energized.")

    def set_angle(self, angle: float, auto_relax: bool = True) -> None:
        """
        Move servo to given angle, factoring in trim offset and safe mechanical limits.
        auto_relax=True relaxes PWM duty to 0% after physical transit to completely eliminate
        coil heating, hum, and high stall current.
        """
        # Clamp to safe limits to prevent physically jamming against gear stops
        angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, float(angle)))
        effective_angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, angle + self.trim_offset))
        duty = self._angle_to_duty(effective_angle)

        with self._lock:
            self._current_angle = angle
            if self._relax_timer is not None:
                self._relax_timer.cancel()
                self._relax_timer = None

            if self.is_initialized and self._pwm:
                self._pwm.ChangeDutyCycle(duty)
                if auto_relax:
                    # Non-blocking background relaxation after physical movement completes
                    self._relax_timer = threading.Timer(0.6, self._relax_pwm)
                    self._relax_timer.daemon = True
                    self._relax_timer.start()
            else:
                logger.debug(f"[SIMULATION] Servo -> target {angle:.1f} deg (effective {effective_angle:.1f} deg)")

    @property
    def current_angle(self) -> float:
        return self._current_angle

    def get_angle(self) -> float:
        return self._current_angle

    def set_angle_smooth(
        self,
        target_angle: float,
        step_deg: float = 2.0,
        step_delay_sec: float = 0.02,
        auto_relax: bool = True
    ) -> None:
        """
        Gradually slews the pan servo to target_angle in controlled angular increments.
        Eliminates abrupt mechanical whip, high current spikes on the 5V rail, camera shake,
        and PWM oscillations / servo jitter ('glitters').
        """
        target_angle = max(self.MIN_ANGLE, min(self.MAX_ANGLE, float(target_angle)))
        start_angle = self._current_angle
        delta = target_angle - start_angle
        if abs(delta) < 0.5:
            self.set_angle(target_angle, auto_relax=auto_relax)
            return

        steps = max(1, int(abs(delta) / max(0.5, step_deg)))
        step_size = delta / steps

        curr = start_angle
        for _ in range(steps):
            curr += step_size
            self.set_angle(curr, auto_relax=False)
            if step_delay_sec > 0:
                time.sleep(step_delay_sec)

        # Latch final destination angle with auto_relax to cut heating
        self.set_angle(target_angle, auto_relax=auto_relax)

    def center(self) -> None:
        self.set_angle(self.center_angle, auto_relax=True)

    def pan_left(self) -> None:
        self.set_angle(self.MIN_ANGLE, auto_relax=True)

    def pan_right(self) -> None:
        self.set_angle(self.MAX_ANGLE, auto_relax=True)

    def survey_surroundings(self, camera=None, corridor_scanner=None, detector=None) -> dict:
        """
        Executes a panoramic survey across Left, Center (90 deg), and Right within safe limits.
        Captures snapshots at each angle and calculates sector clearance and hazards.
        Returns to Center (90 deg) upon completion and relaxes PWM.
        """
        waypoints = [
            ("left", self.MIN_ANGLE),
            ("center", self.center_angle),
            ("right", self.MAX_ANGLE)
        ]
        results = {}

        for sector, angle in waypoints:
            self.set_angle(angle, auto_relax=True)
            time.sleep(0.15)  # Physical settling buffer

            frame = None
            if camera is not None:
                try:
                    frame = camera.capture_frame()
                except Exception as e:
                    logger.warning(f"[SURVEY] Camera capture error at {angle}°: {e}")

            clearance = 1.0
            if frame is not None and corridor_scanner is not None:
                try:
                    analysis = corridor_scanner.analyze(frame)
                    clearance = analysis.get("center_clearance", 1.0)
                except Exception as e:
                    logger.warning(f"[SURVEY] Corridor analysis error at {angle}°: {e}")

            detected_objects = []
            if frame is not None and detector is not None:
                try:
                    dets = detector.detect(frame)
                    detected_objects = [f"{d.class_name} ({d.confidence*100:.0f}%)" for d in dets]
                except Exception as e:
                    logger.warning(f"[SURVEY] Detector error at {angle}°: {e}")

            results[sector] = {
                "angle": angle,
                "clearance": round(clearance, 2),
                "objects": detected_objects,
                "is_clear": clearance > 0.60 and len(detected_objects) == 0
            }

        # Always return servo to center after survey
        self.center()

        # Decide best heading
        best_heading = "center"
        if results.get("center", {}).get("is_clear"):
            best_heading = "center"
        elif results.get("left", {}).get("clearance", 0) > results.get("right", {}).get("clearance", 0):
            best_heading = "left"
        elif results.get("right", {}).get("clearance", 0) > 0.4:
            best_heading = "right"
        else:
            best_heading = "blocked"

        results["best_heading"] = best_heading
        return results

    # ------------------------------------------------------------------
    def cleanup(self) -> None:
        """Center servo then release GPIO PWM."""
        self.center()
        time.sleep(0.3)
        if self.is_initialized and self._pwm:
            self._pwm.stop()
            self.is_initialized = False
            print("[SERVO] GPIO PWM released.")
