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
from dashboard.app import DashboardServer


class AutonomousCarSystem:
    def __init__(self) -> None:
        print("[INIT] Starting Autonomous RC Car (Camera + Motor + Servo System)...")

        # Motor Driver — BCM pins IN1=17, IN2=18, IN3=22, IN4=23
        self.motors = MotorController(in1_pin=17, in2_pin=18, in3_pin=22, in4_pin=23)

        # Camera Survey Servo — GPIO 12 (Physical Pin 32), Hardware PWM0
        self.servo = ServoController(servo_pin=12)
        self.servo.center()  # Start at 90° center

        # CSI Camera — OV5647 640x480
        self.camera = Camera(width=640, height=480)
        self.camera.start()

        # Flask Dashboard — passes all subsystems
        self.dashboard = DashboardServer(
            config=config.dashboard,
            motor_controller=self.motors,
            servo_controller=self.servo,
            camera=self.camera
        )

    def run(self) -> None:
        def shutdown(sig, frame):
            print("\n[SHUTDOWN] Stopping camera, motors, and servo...")
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
