"""
run_autonomous_line_follower.py — Headless Autonomous Line Follower Runner
Runs directly on the Raspberry Pi without requiring the Web Dashboard.

Features:
- Connects to USB serial Arduino co-processor (/dev/ttyACM0, /dev/ttyACM1)
- Captures Picamera2 640x480 video feed
- Runs closed-loop Lane Centering PD control
- Continuous ultrasonic safety collision avoidance (< 20 cm emergency stop)
- Clean SIGINT / SIGTERM shutdown (guarantees motors stop)
"""

import os
import sys
import time
import signal
import threading
import cv2

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
for p in [curr_dir, parent_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from control.line_follower import LineFollower, LineFollowerState

try:
    import serial
except ImportError:
    serial = None

# ── Serial Setup ──────────────────────────────────────────────────────────────
ser = None
latest_dist = None
running = True

def auto_connect_serial():
    global ser
    ports = ["/dev/ttyACM1", "/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    for p in ports:
        if os.path.exists(p):
            try:
                s = serial.Serial(p, 115200, timeout=1)
                time.sleep(2)
                print(f"[serial] Connected to Arduino on {p}")
                ser = s
                return True
            except Exception as e:
                print(f"[serial] Failed {p}: {e}")
    print("[serial] Could not connect to any Arduino serial port!")
    return False

def serial_reader():
    global latest_dist, running, ser
    while running:
        if ser and ser.is_open:
            try:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line.startswith("DIST:"):
                    try:
                        latest_dist = int(float(line.split(":")[1]))
                    except Exception:
                        pass
            except Exception:
                time.sleep(0.5)
        else:
            time.sleep(1)

def send_cmd(cmd: str):
    global ser
    if ser and ser.is_open:
        try:
            ser.write(f"{cmd}\n".encode("ascii"))
            ser.flush()
        except Exception:
            pass

def main():
    global running
    print("=" * 60)
    print("🚗 Autonomous RC Car — Headless Line Follower")
    print("=" * 60)

    def shutdown(sig, frame):
        global running
        print("\n[STOP] Shutting down autonomous driver...")
        running = False
        send_cmd("STOP")
        time.sleep(0.5)
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    if not auto_connect_serial():
        print("[WARN] Running in camera-only / dry-run mode (no serial).")

    reader = threading.Thread(target=serial_reader, daemon=True)
    reader.start()

    # Camera Init
    from picamera2 import Picamera2
    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    ))
    cam.start()
    time.sleep(1)
    print("[camera] Picamera2 stream started.")

    follower = LineFollower(deadzone_px=22, safe_distance_cm=20.0)
    follower.start()
    print("[auto] Line follower started. Press Ctrl+C to stop.")

    last_cmd = None
    last_print = time.time()

    while running:
        rgb_frame = cam.capture_array()
        bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)

        cmd, servo_cmd, annotated, telem = follower.process_frame(bgr_frame, latest_dist)

        if cmd != last_cmd:
            send_cmd(cmd)
            last_cmd = cmd

        now = time.time()
        if now - last_print > 0.5:
            dist_str = f"{latest_dist} cm" if latest_dist is not None else "N/A"
            print(f"[STATUS] Cmd: {cmd:<8} | State: {telem['state']:<12} | "
                  f"Err: {telem['error_px']:>5.1f}px | Dir: {telem['direction']:<8} | Dist: {dist_str}")
            last_print = now

        time.sleep(0.04)

if __name__ == "__main__":
    main()
