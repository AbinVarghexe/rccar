"""
pi_servo_test.py -- Drive servo directly from Raspberry Pi GPIO (no Arduino).
Connect servo signal wire to Pi GPIO 12 (Physical Pin 32) for this test.

Wiring for this test:
  Servo VCC    (red)          -> Pi Pin 2 (5V)   OR external 5V
  Servo GND    (brown/black)  -> Pi Pin 6 (GND)
  Servo signal (orange/white) -> Pi Pin 32 (GPIO 12, Hardware PWM0)

Run: python3 /home/alfin/autonomous_car/scripts/pi_servo_test.py
"""
import time

try:
    from gpiozero import Servo, AngularServo
    USE_GPIOZERO = True
except ImportError:
    USE_GPIOZERO = False

try:
    import RPi.GPIO as GPIO
    USE_RPIGPIO = True
except ImportError:
    USE_RPIGPIO = False

SERVO_GPIO = 12  # BCM GPIO 12 = Physical Pin 32

def test_with_gpiozero():
    print("Testing with gpiozero.AngularServo on GPIO 12...")
    servo = AngularServo(
        SERVO_GPIO,
        min_angle=0,
        max_angle=180,
        min_pulse_width=0.5/1000,
        max_pulse_width=2.5/1000,
    )
    for angle in [0, 90, 180, 90, 45, 135, 90]:
        print(f"  Moving to {angle}°")
        servo.angle = angle - 90  # AngularServo: -90..+90
        time.sleep(1.5)
    servo.detach()
    print("gpiozero test done.")

def test_with_rpigpio():
    print("Testing with RPi.GPIO software PWM on GPIO 12...")
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_GPIO, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_GPIO, 50)  # 50 Hz
    pwm.start(0)

    def angle_to_duty(angle):
        # SG90: 0° = 2.5%, 90° = 7.5%, 180° = 12.5%
        return 2.5 + (angle / 180.0) * 10.0

    for angle in [0, 90, 180, 90, 45, 135, 90]:
        dc = angle_to_duty(angle)
        print(f"  Moving to {angle}° (duty={dc:.1f}%)")
        pwm.ChangeDutyCycle(dc)
        time.sleep(1.5)
        pwm.ChangeDutyCycle(0)  # stop jitter between moves
        time.sleep(0.2)

    pwm.stop()
    GPIO.cleanup()
    print("RPi.GPIO test done.")

if __name__ == "__main__":
    print(f"gpiozero available: {USE_GPIOZERO}")
    print(f"RPi.GPIO available: {USE_RPIGPIO}")
    print()

    if USE_GPIOZERO:
        try:
            test_with_gpiozero()
        except Exception as e:
            print(f"gpiozero failed: {e} -- trying RPi.GPIO")
            if USE_RPIGPIO:
                test_with_rpigpio()
    elif USE_RPIGPIO:
        test_with_rpigpio()
    else:
        print("ERROR: Neither gpiozero nor RPi.GPIO is available.")
        print("Install: pip install gpiozero RPi.GPIO")
