"""
Vehicle Controller Interface.
Coordinates PID steering, vehicle motion primitives, and Camera Pan Servo motor commands
supporting BOTH Arduino Serial AND Raspberry Pi GPIO 18 PWM.
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
    from control.pid import PIDController
    from control.serial_comm import SerialCommunicator, VehicleCommand
    from config import AppConfig
    from utils.logger import car_logger
except ImportError:
    from autonomous_car.control.pid import PIDController
    from autonomous_car.control.serial_comm import SerialCommunicator, VehicleCommand
    from autonomous_car.config import AppConfig
    from autonomous_car.utils.logger import car_logger

# Optional Raspberry Pi GPIO Servo support
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

class VehicleController:
    def __init__(self, config: AppConfig, serial_comm: SerialCommunicator) -> None:
        self.config: AppConfig = config
        self.serial_comm: SerialCommunicator = serial_comm
        self.pid: PIDController = PIDController(config.pid)
        self._current_steering: float = 0.0

        self._gpio_pwm = None
        self._init_pi_gpio_servo()

    def _init_pi_gpio_servo(self) -> None:
        """Initializes Raspberry Pi GPIO 18 PWM for direct Servo connection."""
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                GPIO.setup(18, GPIO.OUT)
                self._gpio_pwm = GPIO.PWM(18, 50) # 50Hz PWM for SG90 Servo
                self._gpio_pwm.start(7.5) # Center 90 degrees (7.5% duty cycle)
                car_logger.info("Initialized Camera Servo on Raspberry Pi GPIO 18 (Header Pin 12).")
            except Exception as e:
                car_logger.debug(f"Pi GPIO 18 Servo init info: {e}")
                self._gpio_pwm = None

    # =========================================================================
    # CAMERA SERVO PAN CONTROLS (Arduino Serial + Pi GPIO 18)
    # =========================================================================

    def pan_camera(self, angle: int) -> None:
        """
        Pans camera servo to specified angle (0 to 180 degrees).
        Sends to BOTH Arduino Serial AND Raspberry Pi GPIO 18.
        """
        angle = max(0, min(180, angle))
        
        # 1. Send via Arduino Serial
        self.serial_comm.send_servo_angle(angle)

        # 2. Send via Raspberry Pi GPIO 18 PWM (if plugged into Pi GPIO)
        if self._gpio_pwm is not None:
            try:
                duty_cycle = 2.5 + (angle / 180.0) * 10.0
                self._gpio_pwm.ChangeDutyCycle(duty_cycle)
                car_logger.info(f"[PI GPIO 18 -> SERVO]: {angle} degrees (Duty Cycle {duty_cycle:.1f}%)")
                time.sleep(0.05)
            except Exception as e:
                car_logger.error(f"Error changing Pi GPIO servo duty cycle: {e}")

    def pan_camera_left(self) -> None:
        self.pan_camera(self.config.servo.left_angle)

    def pan_camera_right(self) -> None:
        self.pan_camera(self.config.servo.right_angle)

    def pan_camera_center(self) -> None:
        self.pan_camera(self.config.servo.center_angle)

    # =========================================================================
    # MOTOR DRIVER PRIMITIVES & DIFFERENTIAL DRIVE
    # =========================================================================

    def moveForward(self, speed: float = 1.0) -> None:
        car_logger.debug(f"Motor Primitive: moveForward(speed={speed:.2f})")
        self.serial_comm.send_command(VehicleCommand.FORWARD)

    def turnLeft(self, speed: float = 1.0) -> None:
        car_logger.debug(f"Motor Primitive: turnLeft(speed={speed:.2f})")
        self.serial_comm.send_command(VehicleCommand.LEFT)

    def turnRight(self, speed: float = 1.0) -> None:
        car_logger.debug(f"Motor Primitive: turnRight(speed={speed:.2f})")
        self.serial_comm.send_command(VehicleCommand.RIGHT)

    def turnBackward(self, speed: float = 1.0) -> None:
        car_logger.debug(f"Motor Primitive: turnBackward(speed={speed:.2f})")
        self.serial_comm.send_command(VehicleCommand.BACKWARD)

    def stop(self) -> None:
        car_logger.debug("Motor Primitive: stop()")
        self.serial_comm.send_command(VehicleCommand.STOP)

    # =========================================================================
    # HIGH LEVEL CONTROL COMPUTATION
    # =========================================================================

    def compute_steering(self, lane_error: float) -> float:
        correction = self.pid.compute(lane_error)
        self._current_steering = correction
        return correction

    def apply_steering_control(self, steering_correction: float) -> None:
        """
        Applies proportional differential speed control to BOTH left & right motor pairs simultaneously.
        Ensures both sides drive smoothly together without locking up one side.
        """
        base_speed = 210
        sensitivity = 110.0

        left_spd = int(base_speed - (steering_correction * sensitivity))
        right_spd = int(base_speed + (steering_correction * sensitivity))

        left_spd = max(-255, min(255, left_spd))
        right_spd = max(-255, min(255, right_spd))

        drive_cmd = f"DRIVE:{left_spd},{right_spd}\n"
        
        if self.serial_comm.is_connected and self.serial_comm._serial and self.serial_comm._serial.is_open:
            try:
                self.serial_comm._serial.write(drive_cmd.encode("utf-8"))
                self.serial_comm._serial.flush()
                car_logger.info(f"[SERIAL OUT -> DIFFERENTIAL DRIVE]: Left={left_spd}, Right={right_spd}")
            except Exception as e:
                car_logger.error(f"Error sending DRIVE command: {e}")

    @property
    def current_steering(self) -> float:
        return self._current_steering
