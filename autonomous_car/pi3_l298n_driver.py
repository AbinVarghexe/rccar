#!/usr/bin/env python3
"""
===============================================================================
🏎️ RASPBERRY PI 3 L298N DIRECT MOTOR CONTROLLER
===============================================================================

Raspberry Pi 3 Header Pin Map:
┌─────────────────────────┬──────────────────────────┬────────────────────────┐
│ L298N Module Pin        │ Pi 3 BCM GPIO Number     │ Physical Header Pin    │
├─────────────────────────┼──────────────────────────┼────────────────────────┤
│ IN1 (Left Dir 1)        │ GPIO 17                  │ Pin 11                 │
│ IN2 (Left Dir 2)        │ GPIO 27                  │ Pin 13                 │
│ IN3 (Right Dir 1)       │ GPIO 22                  │ Pin 15                 │
│ IN4 (Right Dir 2)       │ GPIO 23                  │ Pin 16                 │
│ GND (Shared Ground)     │ Pi 3 GND                 │ Pin 6 or Pin 9         │
│ ENA & ENB               │ Black Jumper Caps ON     │ 5V Power Enable        │
└─────────────────────────┴──────────────────────────┴────────────────────────┘
"""

import time
import sys

# Try RPi.GPIO (Standard on Raspberry Pi 3 OS)
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Notice: RPi.GPIO not installed on local machine. Install on Pi 3 with: sudo apt-get install python3-rpi.gpio")

# Pin Allocation (BCM Numbers for Pi 3)
IN1_PIN = 17 # Pin 11
IN2_PIN = 27 # Pin 13
IN3_PIN = 22 # Pin 15
IN4_PIN = 23 # Pin 16

class RaspberryPi3MotorController:
    def __init__(self):
        self.is_ready = False
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
                
                GPIO.setup(IN1_PIN, GPIO.OUT)
                GPIO.setup(IN2_PIN, GPIO.OUT)
                GPIO.setup(IN3_PIN, GPIO.OUT)
                GPIO.setup(IN4_PIN, GPIO.OUT)

                self.stop_all()
                self.is_ready = True
                print("Raspberry Pi 3 GPIO Motor Driver Initialized Successfully!")
            except Exception as e:
                print(f"Error initializing Pi 3 GPIO: {e}")

    def move_forward(self):
        print(">>> RASPBERRY PI 3: MOVING ALL MOTORS FORWARD...")
        if self.is_ready:
            GPIO.output(IN1_PIN, GPIO.HIGH)
            GPIO.output(IN2_PIN, GPIO.LOW)
            GPIO.output(IN3_PIN, GPIO.HIGH)
            GPIO.output(IN4_PIN, GPIO.LOW)

    def move_backward(self):
        print(">>> RASPBERRY PI 3: MOVING ALL MOTORS REVERSE...")
        if self.is_ready:
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.HIGH)
            GPIO.output(IN3_PIN, GPIO.LOW)
            GPIO.output(IN4_PIN, GPIO.HIGH)

    def turn_left(self):
        print(">>> RASPBERRY PI 3: TURNING LEFT...")
        if self.is_ready:
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.HIGH)
            GPIO.output(IN3_PIN, GPIO.HIGH)
            GPIO.output(IN4_PIN, GPIO.LOW)

    def turn_right(self):
        print(">>> RASPBERRY PI 3: TURNING RIGHT...")
        if self.is_ready:
            GPIO.output(IN1_PIN, GPIO.HIGH)
            GPIO.output(IN2_PIN, GPIO.LOW)
            GPIO.output(IN3_PIN, GPIO.LOW)
            GPIO.output(IN4_PIN, GPIO.HIGH)

    def stop_all(self):
        print(">>> RASPBERRY PI 3: STOPPING ALL MOTORS...")
        if self.is_ready:
            GPIO.output(IN1_PIN, GPIO.LOW)
            GPIO.output(IN2_PIN, GPIO.LOW)
            GPIO.output(IN3_PIN, GPIO.LOW)
            GPIO.output(IN4_PIN, GPIO.LOW)

    def cleanup(self):
        self.stop_all()
        if GPIO_AVAILABLE:
            GPIO.cleanup()

if __name__ == "__main__":
    driver = RaspberryPi3MotorController()
    if driver.is_ready:
        try:
            while True:
                driver.move_forward()
                time.sleep(3)
                driver.stop_all()
                time.sleep(1.5)
                driver.move_backward()
                time.sleep(3)
                driver.stop_all()
                time.sleep(2)
        except KeyboardInterrupt:
            driver.cleanup()
