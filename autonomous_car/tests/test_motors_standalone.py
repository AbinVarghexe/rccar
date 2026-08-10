#!/usr/bin/env python3
"""
STANDALONE MOTOR FUNCTIONALITY TESTER (Phase 1)
Directly tests Raspberry Pi 3 GPIO control of the Motor Driver.

Execution Sequence:
1. FORWARD (3 seconds)
2. STOP (1.5 seconds)
3. BACKWARD (3 seconds)
4. STOP (1.5 seconds)
5. LEFT (3 seconds)
6. STOP (1.5 seconds)
7. RIGHT (3 seconds)
8. STOP (Clean shutdown)

Safety Note:
- Lift the RC car chassis off the ground during this test!
- Ensure common ground between Raspberry Pi GND and Motor Driver GND.
"""

import sys
import os
import time

# Ensure project root is in python path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from hardware.motors import MotorController

def run_motor_test():
    print("=======================================================================")
    print("PHASE 1: RASPBERRY PI 3 STANDALONE MOTOR TEST")
    print("=======================================================================")
    print("SAFETY NOTICE: Ensure car wheels are lifted off the ground!")
    print("Ensure Motor Driver GND is connected to Raspberry Pi GND!")
    print("=======================================================================\n")

    # Initialize Motor Controller on BCM GPIO 17, 18, 22, 23
    motors = MotorController(in1_pin=17, in2_pin=18, in3_pin=22, in4_pin=23)

    try:
        # STEP 1: FORWARD
        print("[TEST 1/4] Testing FORWARD Motion (3 Seconds)...")
        motors.forward()
        time.sleep(3.0)

        print("[PAUSE] Stopping Motors...")
        motors.stop()
        time.sleep(1.5)

        # STEP 2: BACKWARD
        print("[TEST 2/4] Testing BACKWARD Motion (3 Seconds)...")
        motors.backward()
        time.sleep(3.0)

        print("[PAUSE] Stopping Motors...")
        motors.stop()
        time.sleep(1.5)

        # STEP 3: LEFT
        print("[TEST 3/4] Testing Differential Turn LEFT (3 Seconds)...")
        motors.left()
        time.sleep(3.0)

        print("[PAUSE] Stopping Motors...")
        motors.stop()
        time.sleep(1.5)

        # STEP 4: RIGHT
        print("[TEST 4/4] Testing Differential Turn RIGHT (3 Seconds)...")
        motors.right()
        time.sleep(3.0)

        print("[PAUSE] Stopping Motors...")
        motors.stop()
        time.sleep(1.5)

        print("\n=======================================================")
        print("SUCCESS: MOTOR TEST SEQUENCE COMPLETED SUCCESSFULLY!")
        print("=======================================================")

    except KeyboardInterrupt:
        print("\nUser interrupted test sequence (CTRL+C).")
    finally:
        print("Ensuring failsafe motor shutdown and GPIO cleanup...")
        motors.cleanup()

if __name__ == "__main__":
    run_motor_test()
