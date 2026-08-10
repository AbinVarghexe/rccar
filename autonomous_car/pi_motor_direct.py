#!/usr/bin/env python3
"""
DIRECT RASPBERRY PI L298N MOTOR DRIVER (NO ARDUINO NEEDED)

Wiring:
- IN1 (L298N) -> Pi GPIO 17 (Physical Pin 11)
- IN2 (L298N) -> Pi GPIO 27 (Physical Pin 13)
- IN3 (L298N) -> Pi GPIO 22 (Physical Pin 15)
- IN4 (L298N) -> Pi GPIO 23 (Physical Pin 16)
- GND (L298N) -> Pi GND (Physical Pin 6 or 9)
- ENA & ENB jumper caps ON
"""

import time
try:
    import RPi.GPIO as GPIO
except ImportError:
    import gpiozero

# Pin Definitions (BCM GPIO Numbers)
IN1_PIN = 17
IN2_PIN = 27
IN3_PIN = 22
IN4_PIN = 23

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(IN1_PIN, GPIO.OUT)
    GPIO.setup(IN2_PIN, GPIO.OUT)
    GPIO.setup(IN3_PIN, GPIO.OUT)
    GPIO.setup(IN4_PIN, GPIO.OUT)

def move_forward():
    print(">>> RASPBERRY PI: SPINNING ALL MOTORS FORWARD...")
    GPIO.output(IN1_PIN, GPIO.HIGH)
    GPIO.output(IN2_PIN, GPIO.LOW)
    GPIO.output(IN3_PIN, GPIO.HIGH)
    GPIO.output(IN4_PIN, GPIO.LOW)

def move_backward():
    print(">>> RASPBERRY PI: SPINNING ALL MOTORS REVERSE...")
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.HIGH)
    GPIO.output(IN3_PIN, GPIO.LOW)
    GPIO.output(IN4_PIN, GPIO.HIGH)

def stop_motors():
    print(">>> RASPBERRY PI: STOPPING ALL MOTORS...")
    GPIO.output(IN1_PIN, GPIO.LOW)
    GPIO.output(IN2_PIN, GPIO.LOW)
    GPIO.output(IN3_PIN, GPIO.LOW)
    GPIO.output(IN4_PIN, GPIO.LOW)

if __name__ == "__main__":
    setup_gpio()
    try:
        while True:
            move_forward()
            time.sleep(3)
            stop_motors()
            time.sleep(1.5)
            move_backward()
            time.sleep(3)
            stop_motors()
            time.sleep(2)
    except KeyboardInterrupt:
        stop_motors()
        GPIO.cleanup()
