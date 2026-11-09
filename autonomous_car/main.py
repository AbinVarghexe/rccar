"""
Main Entry Point for Autonomous RC Car System.
Raspberry Pi 3 GPIO Motor + Servo Camera Pan + Live CSI Camera Stream.
"""

import os
import sys
import signal
import logging

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] [%(name)s] - %(message)s",
    datefmt="%H:%M:%S"
)

curr_dir = os.path.dirname(os.path.abspath(__file__))
if curr_dir not in sys.path:
    sys.path.insert(0, curr_dir)

from config import config
from hardware.motors import MotorController
from hardware.servo import ServoController
from vision.camera import Camera
from behavioral_cloning import BehavioralCloningDriver
from sensors.ultrasonic import UltrasonicSensors
from dashboard.app import DashboardServer


class AutonomousCarSystem:
    def __init__(self) -> None:
        print("[INIT] Starting Autonomous RC Car (Perception + Driving + Camera System)...")

        # Motor Driver — BCM pins IN1=17, IN2=18, IN3=22, IN4=23
        m_cfg = config.motor
        self.motors = MotorController(
            in1_pin=m_cfg.in1_pin,
            in2_pin=m_cfg.in2_pin,
            in3_pin=m_cfg.in3_pin,
            in4_pin=m_cfg.in4_pin,
            in2_alt_pin=m_cfg.in2_alt_pin,
            left_polarity=m_cfg.left_polarity,
            right_polarity=m_cfg.right_polarity,
        )

        # Camera Survey Servo — GPIO 12 (Physical Pin 32), Hardware PWM0
        s_cfg = config.servo
        # The installed camera is physically straight ahead at servo 90°.
        # Keep the hardware center configurable, while retaining 90° as the
        # coordinate-frame reference used by navigation/projection.
        self.servo = ServoController(servo_pin=s_cfg.pin, center_angle=s_cfg.center_angle)
        self.servo.center()  # Start at 90° center

        # CSI Camera — OV5647 640x480
        self.camera = Camera(width=640, height=480)
        self.camera.start()

        # HC-SR04 is active for both manual forward guards and autonomous
        # safety validation. It remains harmless while the driver is disabled.
        self.ultrasonic_sensors = UltrasonicSensors(config.sensor)

        # DeepPiCar-style behavioral-cloning driver. Manual dashboard control
        # can collect labeled frames; a trained pilot is enabled explicitly
        # through /api/control/mode or /api/auto/toggle.
        self.auto_driver = BehavioralCloningDriver(
            config=config.behavioral_cloning,
            motors=self.motors,
            camera=self.camera,
            ultrasonic_sensors=self.ultrasonic_sensors,
            safety_config=config.safety,
        )
        self.auto_driver.start()

        # Flask Dashboard — passes all subsystems
        self.dashboard = DashboardServer(
            config=config.dashboard,
            motor_controller=self.motors,
            servo_controller=self.servo,
            camera=self.camera,
            autonomous_driver=self.auto_driver,
        )

    def run(self) -> None:
        def shutdown(sig, frame):
            print("\n[SHUTDOWN] Stopping autonomous driver, camera, motors, and servo...")
            self.auto_driver.stop()
            self.camera.stop()
            if self.ultrasonic_sensors is not None:
                self.ultrasonic_sensors.cleanup()
            self.motors.stop()
            self.motors.cleanup()
            self.servo.center()
            self.servo.cleanup()
            sys.exit(0)

        signal.signal(signal.SIGINT, shutdown)
        signal.signal(signal.SIGTERM, shutdown)

        self.dashboard.start()


def main():
    system = AutonomousCarSystem()
    system.run()


if __name__ == "__main__":
    main()
