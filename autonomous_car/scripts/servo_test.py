"""
servo_test.py -- Direct servo diagnostic.
Run on the Pi: python3 /home/alfin/autonomous_car/scripts/servo_test.py
"""
import serial
import time
import subprocess

# Kill anything holding the port
for p in ['/dev/ttyACM1', '/dev/ttyACM0']:
    subprocess.run(['fuser', '-k', p], capture_output=True)
time.sleep(1)

# Find port
port = None
import os
for p in ['/dev/ttyACM1', '/dev/ttyACM0', '/dev/ttyUSB0']:
    if os.path.exists(p):
        port = p
        break

if not port:
    print("ERROR: No Arduino port found")
    exit(1)

print(f"Connecting to {port} at 115200...")
ser = serial.Serial(port, 115200, timeout=2)
time.sleep(2.5)
ser.reset_input_buffer()

# Drain 3 DIST lines to confirm sketch is alive
print("\n=== Sketch alive check ===")
for _ in range(3):
    line = ser.readline().decode(errors='replace').strip()
    print(f"  {line}")

# Sweep servo and confirm ACK
print("\n=== Servo sweep: 0 -> 90 -> 180 -> 90 ===")
for angle in [0, 90, 180, 90]:
    ser.write(f"SERVO:{angle}\n".encode())
    print(f"  Sent SERVO:{angle} -- waiting for ACK...")
    ack = ""
    deadline = time.time() + 3
    while time.time() < deadline:
        if ser.in_waiting:
            line = ser.readline().decode(errors='replace').strip()
            if line.startswith("ACK:SERVO"):
                ack = line
                break
            # skip DIST lines
    print(f"  Got: {ack if ack else 'NO ACK (timeout)'}")
    time.sleep(1.5)

ser.close()
print("\nDone. If ACKs were received but servo did not move:")
print("  1. Check signal wire is on D6 (not D5, D9, etc.)")
print("  2. Check servo VCC is 5V (Arduino 5V pin)")
print("  3. Check servo GND shares Arduino GND")
print("  4. Try a different servo -- some are dead on arrival")
