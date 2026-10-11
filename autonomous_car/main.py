"""
Main Entry Point for Autonomous RC Car System.
Raspberry Pi 3 GPIO Motor + Servo Camera Pan + Live CSI Camera Stream.
"""

import os
import sys
import signal

curr_dir = os.path.dirname(os.path.abspath(__file__))
if curr_dir not in sys.path:
    sys.path.insert(0, curr_dir)

from config import config
from hardware.motors import MotorController
from hardware.servo import ServoController
from vision.camera import Camera
from navigation.autonomous_driver import AutonomousDriver
from dashboard.app import DashboardServer


class AutonomousCarSystem:
    def __init__(self) -> None:
        print("[INIT] Starting Autonomous RC Car (Perception + Driving + Camera System)...")

        # Motor Driver — BCM pins IN1=17, IN2=27 (alt 18), IN3=22, IN4=23
        m_cfg = config.motor
        self.motors = MotorController(
            in1_pin=m_cfg.in1_pin,
            in2_pin=m_cfg.in2_pin,
            in3_pin=m_cfg.in3_pin,
            in4_pin=m_cfg.in4_pin,
            in2_alt_pin=m_cfg.in2_alt_pin,
        )

        # Camera Survey Servo — GPIO 12 (Physical Pin 32), Hardware PWM0
        s_cfg = config.servo
        self.servo = ServoController(servo_pin=s_cfg.pin)
        self.servo.center()  # Start at 90° center

        # CSI Camera — OV5647 640x480
        self.camera = Camera(width=640, height=480)
        self.camera.start()

        # Autonomous Driving System
        self.auto_driver = AutonomousDriver(
            config=config.autonomous,
            motors=self.motors,
            servo=self.servo,
            camera=self.camera,
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
