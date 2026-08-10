# Modular Autonomous RC Car Software Stack

A complete, production-grade autonomous RC car software system built from scratch in Python 3.11 for the Raspberry Pi 4.

---

## Hardware Architecture

- **SBC**: Raspberry Pi 4 (4GB / 8GB RAM recommended)
- **Camera**: Raspberry Pi CSI Camera Module (driven natively via `Picamera2`)
- **Microcontroller**: Arduino UNO (Serial UART link over USB)
- **Distance Sensors**: Dual HC-SR04 Ultrasonic Sensors (Front & Rear)
- **Chassis**: 4WD Robot Chassis with 4 TT Motors
- **Power**: Separate Power Bank for Raspberry Pi and dedicated battery pack for motors

---

## Directory Structure

```
autonomous_car/
│
├── main.py                     # Main orchestrator & thread controller
├── config.py                   # Centralized configuration & parameters
├── requirements.txt            # Python dependencies
│
├── vision/
│   ├── __init__.py
│   ├── camera.py              # Picamera2 driver (with desktop/cv2 fallbacks)
│   ├── preprocessing.py       # Grayscale, Gaussian Blur, Threshold, Morph, ROI
│   ├── lane_detector.py       # Contour centroid lane tracking & HUD overlay
│   ├── junction_detector.py   # Left, Right, T-Junction, Crossroad & Dead-end detection
│   └── object_detector.py     # Ultralytics YOLOv8 object detection
│
├── sensors/
│   ├── __init__.py
│   └── ultrasonic.py          # Dual HC-SR04 distance measurement (front & rear)
│
├── control/
│   ├── __init__.py
│   ├── pid.py                 # Discrete PID controller with anti-windup
│   ├── serial_comm.py         # PySerial Arduino communication (FORWARD, LEFT, etc.)
│   └── controller.py          # Vehicle Controller & Motor Driver Primitives
│
├── navigation/
│   ├── __init__.py
│   ├── state_machine.py       # Finite State Machine (IDLE, FOLLOW_LANE, TURN, etc.)
│   └── planner.py             # Sensor fusion & navigation planner
│
├── dashboard/
│   ├── __init__.py
│   └── app.py                 # Flask server with live MJPEG streams & telemetry UI
│
├── utils/
│   ├── __init__.py
│   ├── fps.py                 # Smooth FPS calculation utility
│   └── logger.py              # Console & file logger
│
├── tests/                     # Comprehensive Unit Test Suite
│   ├── __init__.py
│   ├── test_vision.py
│   ├── test_control.py
│   ├── test_sensors.py
│   └── test_navigation.py
│
└── README.md                  # System Documentation & Wiring Guide
```

---

## Future Motor Driver Integration Guide

The software stack is fully operational without the motor driver connected. When your motor driver hardware arrives:

1. Open `autonomous_car/control/controller.py`.
2. Locate the following 5 motor primitive functions:
   - `moveForward(speed: float)`
   - `turnLeft(speed: float)`
   - `turnRight(speed: float)`
   - `turnBackward(speed: float)`
   - `stop()`
3. Update these 5 methods to interface with your specific motor driver (e.g., L298N, L293D, or PCA9685 via `RPi.GPIO` PWM or Arduino commands).

Example update in `control/controller.py`:
```python
def moveForward(self, speed: float = 1.0) -> None:
    # Add your motor driver hardware pins here:
    # E.g.: GPIO.output(IN1, GPIO.HIGH); pwm_A.ChangeDutyCycle(speed * 100)
    self.serial_comm.send_command(VehicleCommand.FORWARD)
```

---

## Installation & Setup

### 1. Raspberry Pi Setup (Raspberry Pi OS - Bookworm / Bullseye)

Install system dependencies and native `picamera2`:
```bash
sudo apt update
sudo apt install -y python3-picamera2 python3-pip python3-opencv
```

Clone or copy the repository to your Raspberry Pi, then install requirements:
```bash
cd autonomous_car
pip3 install -r requirements.txt
```

### 2. Desktop / Windows Development Mode

The codebase automatically detects whether it is running on a Raspberry Pi or Windows machine. On Windows / Desktop systems without a CSI camera or serial port:
- Camera falls back to USB webcam or a synthetic test track generator.
- GPIO falls back to mock sensor distances.
- PySerial falls back to mock console output.

Install dependencies on Windows:
```bash
pip install -r requirements.txt
```

---

## How to Run

### 1. Launch the Main Autonomous Stack & Web Dashboard
```bash
python main.py
```
Or on Raspberry Pi:
```bash
python3 main.py
```

### 2. Access the Live Dashboard
Open your web browser and navigate to:
```
http://<RASPBERRY_PI_IP>:5000
```
or on local dev machine:
```
http://localhost:5000
```

The Web Dashboard displays:
- **Camera HUD Feed**: Live video stream with center line, lane contours, error lines, direction, and FPS.
- **Threshold ROI Feed**: Binary preprocessed image stream.
- **Real-Time Telemetry**: Navigation State (`IDLE`, `FOLLOW_LANE`, `OBSTACLE`, `STOP`), Lane Error (px), Front/Rear Distances (cm), and active Motor Command.

---

## Running Automated Tests

Run the test suite to verify code integrity:
```bash
python -m unittest discover -s tests
```

---

## Navigation State Machine

```
              +-------------------+
              |       IDLE        |
              +---------+---------+
                        |
                        v
              +-------------------+
        +---->|    FOLLOW_LANE    |<----+
        |     +----+----+----+----+     |
        |          |    |    |          |
        |   Left   |    |    | Right    |
        |   Turn   |    |    | Turn     |
        |          v    |    v          |
  +-----+-----+       |       +-----+-----+
  | TURN_LEFT |       |       | TURN_RIGHT|
  +-----------+       |       +-----------+
                      v
             Obstacle / Stop Sign /
             Emergency Distance
                      |
                      v
              +-------------------+
              | OBSTACLE / STOP   |
              +-------------------+
```

- **IDLE**: Car initialized and waiting.
- **FOLLOW_LANE**: PID controller aligns vehicle with lane center.
- **TURN_LEFT / TURN_RIGHT**: Executed on junction detection.
- **OBSTACLE**: Triggered by front ultrasonic sensor (<30cm) or YOLO object detection (Person, Car, Cone, Bottle).
- **STOP**: Emergency stop triggered when front/rear sensor reads critical distance (<15cm), YOLO detects Stop Sign, or dead-end reached.
