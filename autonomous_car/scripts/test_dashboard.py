"""
test_dashboard.py — Next-Gen Autonomous Mission Cockpit.
ArUco Visual Waypoints, Sanoguzhan LKA Lane Assistant, ONNX Neural Pilot & Thermal Control.
Runs on the Raspberry Pi (http://<pi-ip>:5555).

Design Theme:
- Clean Light-Mode SaaS Architecture (Pure white cards, soft #F4F6FA canvas, royal purple gradients)
- Left Column: Wide Camera Feed, 3D HUD, Sanoguzhan Bird's Eye Perspective, 3 Metrics Cards
- Right Column: Vehicle Dynamics & Steering Dial, Ultrasonic Vertical Bar Chart, Modern D-Pad,
                Servo Slider, ArUco Destination Tag List, Autonomy Mode Switcher
- Zero-Latency MJPEG Streaming with socket TCP_NODELAY and Condition-based frame deduplication
"""

import threading
import time
import io
import os
import sys
import socket
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    serial = None
    SERIAL_AVAILABLE = False
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
import json
import urllib.parse
import cv2
import numpy as np

# Ensure project modules are importable
curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
for p in [curr_dir, parent_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from control.line_follower import LineFollower, LineFollowerState, SurveyMode, NavigationMode
from control.obstacle_avoider import trusted_reading
from navigation.aruco_navigator import ArucoNavigator
from ai.detector import YOLODetector

# ── Global State & Synchronization ────────────────────────────────────────────
CAMERA_RAW_JPEG = None
CAMERA_HUD_JPEG = None
CAMERA_BIRDSEYE_JPEG = None
LATEST_FRAME_TS = 0.0
# Only views somebody is watching get JPEG-encoded; each view keeps its own frame time.
VIEW_NAMES = ("hud", "raw", "birdseye")
VIEW_WANTED = {v: 0.0 for v in VIEW_NAMES}   # last time a client asked for the view
VIEW_TS = {v: 0.0 for v in VIEW_NAMES}       # time of the JPEG currently stored
VIEW_WANTED_FOR_S = 3.0
STREAM_MAX_FPS = 15.0
LIVE_FRAMES = {}   # newest numpy frame per view, read by the WebRTC track
CAMERA_LOCK = threading.Lock()
FRAME_CONDITION = threading.Condition(CAMERA_LOCK)
CAMERA_OK = False
CAMERA_ERROR = ""

PORT = "/dev/ttyACM1"
BAUD = 115200
ser = None
ser_lock = threading.Lock()
serial_status = "disconnected"
latest_dist = None
latest_dist_ts = None  # time.time() of the last real DIST line (None = never/standby)
# Desk testing without an Arduino only: RCCAR_SIM_SONAR=1 feeds a wobbling fake
# distance. Off by default - on the car a dead serial link must mean "no sonar".
SIM_SONAR = os.environ.get("RCCAR_SIM_SONAR") == "1"
last_motion_cmd = "STOP"
REFLEX_STOP_CM = 15  # reader-thread emergency stop, independent of vision-loop latency
last_ack = ""
servo_angle = 90      # last angle the camera loop SENT to the Arduino
servo_acked = None    # last angle the Arduino confirmed with ACK:SERVO:<angle>
latest_detections = []
object_detector = None
object_detection_error = ""
object_detection_last_ts = 0.0

# Autonomous Controller & Navigation Engine
line_follower = LineFollower(
    deadzone_px=15,
    safe_distance_cm=35.0,
    enable_auto_avoidance=True,
    model_path="models/SidewalkPilot-v3.1b.onnx",
    marker_size_cm=10.0,
    lane_layout="three_line",   # solid outer tapes + dashed centre tape (two lanes)
    servo_search=False,         # the camera never sweeps to hunt for a tag or a path
)
try:
    object_detector = YOLODetector(
        model_path="models/yolov8n_int8.tflite",
        confidence=0.30,
        iou=0.45,
    )
    if not object_detector.is_initialized:
        object_detection_error = "YOLO detector failed to initialize"
except Exception as detector_error:
    object_detection_error = str(detector_error)
    print(f"[object-detection] initialization failed: {detector_error}")
auto_running = False
auto_lock = threading.Lock()

# Standby Mode & Thermal Relief State
standby_mode = False
standby_lock = threading.Lock()
standby_wake_event = threading.Event()
standby_wake_event.set()

# Camera Color & Channel Alignment State
swap_rb = False


def get_cpu_temp() -> float:
    """Reads Raspberry Pi SoC temperature in degrees Celsius."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return round(float(f.read().strip()) / 1000.0, 1)
    except Exception:
        return 0.0


def draw_object_detections(frame, detections):
    """Draw lightweight detector output on the live camera HUD."""
    if frame is None:
        return frame
    height, width = frame.shape[:2]
    for detection in detections:
        box = detection.bbox
        x1 = max(0, min(width - 1, int(box.x1)))
        y1 = max(0, min(height - 1, int(box.y1)))
        x2 = max(x1 + 1, min(width - 1, int(box.x2)))
        y2 = max(y1 + 1, min(height - 1, int(box.y2)))
        color = (0, 0, 255) if detection.class_name == "person" else (0, 190, 255)
        label = f"{detection.class_name.upper()} {detection.confidence:.0%}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        (label_width, label_height), _ = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1
        )
        label_top = max(0, y1 - label_height - 6)
        cv2.rectangle(
            frame,
            (x1, label_top),
            (min(width - 1, x1 + label_width + 6), y1),
            color,
            -1,
        )
        cv2.putText(
            frame,
            label,
            (x1 + 3, max(label_height + 1, y1 - 3)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.38,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return frame


# ── Serial Connection Management ──────────────────────────────────────────────
def _try_connect():
    if not SERIAL_AVAILABLE or serial is None:
        return None, None
    target_ports = [PORT, "/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyUSB1"]
    for p in target_ports:
        try:
            if not os.path.exists(p):
                continue
            s = serial.Serial(p, BAUD, timeout=1)
            time.sleep(2)
            print(f"[serial] Connected on {p}")
            return s, p
        except Exception as e:
            print(f"[serial] {p} connection failed: {e}")
    return None, None


def connect_serial():
    global ser, serial_status
    s, p = _try_connect()
    if s:
        with ser_lock:
            ser = s
            serial_status = f"connected:{p}"
    else:
        serial_status = "disconnected"


def _reconnect():
    global ser, serial_status
    serial_status = "reconnecting"
    with ser_lock:
        if ser:
            try:
                ser.close()
            except Exception:
                pass
            ser = None
    time.sleep(2)
    s, p = _try_connect()
    if s:
        with ser_lock:
            ser = s
            serial_status = f"connected:{p}"
    else:
        serial_status = "disconnected"


_last_reconnect_ts = 0.0
_reconnect_lock = threading.Lock()


def _trigger_reconnect():
    global _last_reconnect_ts
    with _reconnect_lock:
        now = time.time()
        if now - _last_reconnect_ts < 2.0:
            return
        _last_reconnect_ts = now
        threading.Thread(target=_reconnect, daemon=True).start()


MOTION_CMDS = frozenset({"FORWARD", "BACKWARD", "LEFT", "RIGHT", "HARD_LEFT", "HARD_RIGHT",
                         "SLIDE_LEFT", "SLIDE_RIGHT", "STOP"})


# Commands the dashboard may send to the car. Anything else (a typo, a stray request,
# AUTO:1/0 which only the follower arms) is refused instead of reaching the Arduino.
ALLOWED_CMDS = MOTION_CMDS | frozenset({"STANDBY", "RESUME", "PING"})
# Commands that drive the car toward whatever is in front of it.
FORWARD_MOTION = frozenset({"FORWARD", "SLIDE_LEFT", "SLIDE_RIGHT"})


def send_cmd(cmd: str):
    global last_ack, last_motion_cmd
    with ser_lock:
        s = ser
    if s and s.is_open:
        try:
            s.write(f"{cmd}\n".encode())
            s.flush()
            last_ack = f"sent: {cmd}"
            if cmd in MOTION_CMDS:
                last_motion_cmd = cmd
        except Exception:
            last_ack = "write error — reconnecting"
            _trigger_reconnect()
    else:
        last_ack = "not connected"
        _trigger_reconnect()


def _on_dist(value: int) -> None:
    """Record a real sonar reading and brake immediately if driving into something."""
    global latest_dist, latest_dist_ts
    latest_dist = value
    latest_dist_ts = time.time()
    if auto_running and last_motion_cmd in FORWARD_MOTION and 2 <= value <= REFLEX_STOP_CM:
        send_cmd("STOP")


def _note_ack(line: str) -> None:
    """Record an Arduino ACK; ACK:SERVO:<angle> is the proof the servo command arrived."""
    global last_ack, servo_acked
    last_ack = line
    if line.startswith("ACK:SERVO:"):
        try:
            servo_acked = int(line.split(":")[2])
        except (IndexError, ValueError):
            pass


def _offline_sonar() -> None:
    """The serial link is down, so there is no sonar data.

    The old code invented a distance here (68 +/- 14 cm) with no timestamp,
    which the obstacle avoider stamped "now" and treated as a fresh, clear
    reading; it also fed the camera range calibration. Clear the reading
    instead so every consumer sees SENSOR_FAULT / STOP.
    """
    global latest_dist, latest_dist_ts
    if SIM_SONAR:
        latest_dist = int(68 + 14 * np.sin(time.time() * 0.8))
        latest_dist_ts = time.time()
    else:
        latest_dist = None
        latest_dist_ts = None


def reader_thread():
    global latest_dist, latest_dist_ts, last_ack, serial_status
    while True:
        try:
            with standby_lock:
                is_sb = standby_mode
            if is_sb:
                # In Standby mode: stop ultrasonic sensor reading and flush serial backlog
                latest_dist = None
                latest_dist_ts = None
                with ser_lock:
                    s = ser
                if s and s.is_open and s.in_waiting > 0:
                    try:
                        s.reset_input_buffer()
                    except Exception:
                        pass
                time.sleep(0.2)
                continue

            with ser_lock:
                s = ser
            if s and s.is_open:
                in_waiting = s.in_waiting
                if in_waiting > 64:
                    raw = s.read(in_waiting).decode("utf-8", errors="replace")
                    lines = [ln.strip() for ln in raw.split("\n") if ln.strip()]
                    for line in reversed(lines):
                        if line.startswith("DIST:"):
                            try:
                                _on_dist(int(float(line.split(":")[1])))
                                break
                            except Exception:
                                pass
                        elif line.startswith("ACK:"):
                            _note_ack(line)
                else:
                    line = s.readline().decode("utf-8", errors="replace").strip()
                    if line.startswith("DIST:"):
                        try:
                            _on_dist(int(float(line.split(":")[1])))
                        except Exception:
                            pass
                    elif line.startswith("ACK:"):
                        _note_ack(line)
            else:
                _offline_sonar()
                time.sleep(1.0)
                _trigger_reconnect()
        except Exception:
            serial_status = "error — reconnecting"
            _offline_sonar()
            time.sleep(1.0)
            _trigger_reconnect()


# ── Camera, AI & Autonomous Navigation Thread ────────────────────────────────
def camera_vision_thread():
    global CAMERA_RAW_JPEG, CAMERA_HUD_JPEG, CAMERA_BIRDSEYE_JPEG, LATEST_FRAME_TS, VIEW_TS
    global CAMERA_OK, CAMERA_ERROR, auto_running, servo_angle
    global latest_detections, object_detection_last_ts, object_detection_error
    cam = None
    cap = None
    use_picam = True
    camera_active = False

    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        # Native 400x240 low-latency configuration matches Sanoguzhan pipeline
        config = cam.create_preview_configuration(
            main={"size": (400, 240), "format": "RGB888"}  # numpy order = B,G,R (OpenCV native)
        )
        cam.configure(config)
        cam.start()
        camera_active = True
        time.sleep(0.5)
        CAMERA_OK = True
        print("[camera] Picamera2 initialized in native 400x240 low-latency RGB888 (BGR array) mode.")
    except Exception as e:
        print(f"[camera] Picamera2 init failed ({e}), falling back to cv2.VideoCapture(0)...")
        use_picam = False
        try:
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
            if not cap.isOpened():
                CAMERA_OK = False
                CAMERA_ERROR = "No camera found (Picamera2 & /dev/video0 both failed)"
                return
            CAMERA_OK = True
            camera_active = True
            CAMERA_ERROR = "OpenCV fallback (/dev/video0)"
        except Exception as e2:
            CAMERA_OK = False
            CAMERA_ERROR = str(e2)
            return

    last_auto_cmd = None
    last_cmd_time = 0.0
    last_servo_time = 0.0
    last_detection_time = 0.0
    standby_banner_emitted = False

    while True:
        try:
            with standby_lock:
                is_standby = standby_mode

            if is_standby:
                # ── COMPLETE STANDBY CUT OFF ─────────────────────────────────
                # Autonomy must be disarmed
                latest_detections = []
                with auto_lock:
                    if auto_running:
                        auto_running = False
                try:
                    line_follower.stop()
                except Exception:
                    pass

                # Cut off physical camera hardware, sensor pipeline, and ISP
                if camera_active:
                    try:
                        if use_picam and cam:
                            cam.stop()
                            print("[camera] Picamera2 capture stopped (standby cooling active)")
                        elif not use_picam and cap:
                            cap.release()
                            print("[camera] VideoCapture released (standby cooling active)")
                    except Exception as cam_stop_err:
                        print(f"[camera] Error pausing camera: {cam_stop_err}")
                    camera_active = False

                # Generate and emit standby display banner ONCE
                if not standby_banner_emitted:
                    cpu_temp = get_cpu_temp()
                    standby_frame = np.zeros((240, 400, 3), dtype=np.uint8)
                    cv2.rectangle(standby_frame, (8, 8), (392, 232), (30, 41, 59), 1)
                    cv2.putText(standby_frame, "STANDBY COOLING ACTIVE", (55, 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (56, 189, 248), 2, cv2.LINE_AA)

                    temp_color = (74, 222, 128) if cpu_temp < 60.0 else ((251, 191, 36) if cpu_temp < 70.0 else (248, 113, 113))
                    cv2.putText(standby_frame, f"Raspberry Pi SoC: {cpu_temp} C", (95, 115),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.52, temp_color, 2, cv2.LINE_AA)
                    cv2.putText(standby_frame, "Camera & Hardware Completely Cut Off", (40, 150),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (148, 163, 184), 1, cv2.LINE_AA)
                    cv2.putText(standby_frame, "Low-Power Standby Signal Sent to Board", (36, 175),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (148, 163, 184), 1, cv2.LINE_AA)
                    cv2.putText(standby_frame, "Click [STANDBY: ACTIVE] on Cockpit to Wake", (42, 205),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (251, 191, 36), 1, cv2.LINE_AA)

                    _, s_buf = cv2.imencode('.jpg', standby_frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    s_bytes = bytes(s_buf)
                    with FRAME_CONDITION:
                        CAMERA_RAW_JPEG = s_bytes
                        CAMERA_HUD_JPEG = s_bytes
                        CAMERA_BIRDSEYE_JPEG = s_bytes
                        LATEST_FRAME_TS = time.time()
                        for _v in VIEW_NAMES:
                            VIEW_TS[_v] = LATEST_FRAME_TS
                        FRAME_CONDITION.notify_all()
                    standby_banner_emitted = True

                # Put vision thread completely to sleep (zero CPU cycles used)
                standby_wake_event.wait(timeout=1.0)
                continue

            # ── RESUME FROM STANDBY ──────────────────────────────────────────
            if not camera_active:
                try:
                    if use_picam and cam:
                        cam.start()
                        print("[camera] Picamera2 restarted from standby")
                    elif not use_picam:
                        cap = cv2.VideoCapture(0)
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
                        print("[camera] VideoCapture reopened from standby")
                    time.sleep(0.3)
                except Exception as cam_start_err:
                    print(f"[camera] Error restarting camera: {cam_start_err}")
                camera_active = True
                standby_banner_emitted = False

            if use_picam:
                raw_frame = cam.capture_array()
            else:
                ret, raw_frame = cap.read()
                if not ret:
                    time.sleep(0.05)
                    continue

            # Ensure frame is 400x240
            if raw_frame.shape[1] != 400 or raw_frame.shape[0] != 240:
                raw_frame = cv2.resize(raw_frame, (400, 240), interpolation=cv2.INTER_NEAREST)

            # Live Red/Blue Channel Swap compensation if requested
            if swap_rb and raw_frame is not None:
                bgr_frame = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2RGB)
            else:
                bgr_frame = raw_frame

            # Process frame through Autonomous Navigation Engine
            with auto_lock:
                is_active = auto_running
                if is_active and not line_follower.is_active:
                    line_follower.start()
                    send_cmd("AUTO:1")  # arm firmware watchdog/hard-stop (ignored by v1 firmware)
                elif not is_active and line_follower.is_active:
                    line_follower.stop()
                    send_cmd("AUTO:0")

            sonar_cm, sonar_ts = trusted_reading(latest_dist, latest_dist_ts)
            cmd, servo_cmd, hud_frame, telem = line_follower.process_frame(bgr_frame, sonar_cm, sonar_ts)
            bev_frame = line_follower.latest_birdseye if line_follower.latest_birdseye is not None else hud_frame
            now = time.time()

            # Run the lightweight INT8 detector at a bounded cadence. The
            # SidewalkPilot ONNX model above is steering-only and does not
            # produce object classes or bounding boxes.
            detect_interval = 1.5 if is_active else 0.35  # display-only; keep CPU for control
            if object_detector is not None and object_detector.is_initialized and (
                now - last_detection_time >= detect_interval
            ):
                try:
                    detections = object_detector.detect(bgr_frame)
                    latest_detections = detections
                    object_detection_last_ts = now
                    last_detection_time = now
                except Exception as detector_error:
                    object_detection_error = str(detector_error)
                    last_detection_time = now

            hud_frame = draw_object_detections(hud_frame, latest_detections)

            # Dispatch Servo Angle changes
            if servo_cmd is not None and servo_cmd != servo_angle and (now - last_servo_time) > 0.05:
                send_cmd(f"SERVO:{servo_cmd}")
                servo_angle = servo_cmd
                last_servo_time = now

            # In Autonomous mode, dispatch steering commands
            if is_active:
                if cmd == "STOP" and last_auto_cmd != "STOP":
                    send_cmd("STOP")
                    last_auto_cmd = "STOP"
                    last_cmd_time = now
                elif cmd != last_auto_cmd or (now - last_cmd_time) > 0.25:
                    send_cmd(cmd)
                    last_auto_cmd = cmd
                    last_cmd_time = now

            LIVE_FRAMES["hud"], LIVE_FRAMES["raw"], LIVE_FRAMES["birdseye"] = hud_frame, bgr_frame, bev_frame
            # JPEG-encode only the views a client is watching (at most one in practice).
            wanted = {v: (now - VIEW_WANTED[v]) < VIEW_WANTED_FOR_S for v in VIEW_NAMES}
            sources = {"hud": (hud_frame, 60), "raw": (bgr_frame, 55), "birdseye": (bev_frame, 55)}
            encoded = {}
            for view, (img, quality) in sources.items():
                if wanted[view]:
                    ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
                    if ok:
                        encoded[view] = bytes(buf)

            with FRAME_CONDITION:
                if "hud" in encoded:
                    CAMERA_HUD_JPEG = encoded["hud"]
                if "raw" in encoded:
                    CAMERA_RAW_JPEG = encoded["raw"]
                if "birdseye" in encoded:
                    CAMERA_BIRDSEYE_JPEG = encoded["birdseye"]
                for view in encoded:
                    VIEW_TS[view] = now
                LATEST_FRAME_TS = now
                FRAME_CONDITION.notify_all()

        except Exception as e:
            CAMERA_ERROR = str(e)
            time.sleep(0.04)


# ── SaaS Light-Mode HTML Cockpit Template ─────────────────────────────────────
HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>RoboPilot • Autonomous Mission Cockpit</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;500;600;700;800&family=JetBrains+Mono:wght@500;600;700&display=swap" rel="stylesheet">
<style>
/* ── EDUPLEX DESIGN SYSTEM TOKENS ── */
*, *::before, *::after {
  box-sizing: border-box;
  margin: 0;
  padding: 0;
  user-select: none;
  -webkit-user-select: none;
  -webkit-tap-highlight-color: transparent;
}

:root {
  --canvas-bg     : #C2A5F9;           /* Wallpaper soft lavender */
  --app-bg        : #F8F9FC;           /* Inner container off-white */
  --card-bg       : #FFFFFF;           /* Pure white cards */
  --card-border   : #EEF0F5;           /* Hairline card border */
  --dark-accent   : #201F31;           /* Deep obsidian charcoal */
  --lime-accent   : #D4F973;           /* Signature neon lime */
  --lime-hover    : #C4EB60;
  --text-main     : #1E1B2E;           /* Primary text */
  --text-muted    : #8C8A9E;           /* Secondary muted text */
  --pastel-peach  : #FFEDD5;           /* Soft orange */
  --pastel-peach-txt: #EA580C;
  --pastel-mint   : #DCFCE7;           /* Soft green */
  --pastel-mint-txt : #16A34A;
  --pastel-blue   : #E0F2FE;           /* Soft sky blue */
  --pastel-blue-txt : #0284C7;
  --pastel-purple : #EDE9FE;           /* Soft lavender */
  --pastel-purple-txt: #7C3AED;
  --r-container   : 32px;
  --r-card        : 22px;
  --r-pill        : 14px;
  --r-sm          : 10px;
  --shadow-card   : 0 4px 20px rgba(30, 27, 46, 0.04);
  --shadow-float  : 0 20px 60px rgba(30, 27, 46, 0.12);
  --font-sans     : 'Plus Jakarta Sans', -apple-system, BlinkMacSystemFont, sans-serif;
  --font-mono     : 'JetBrains Mono', monospace;
}

/* ── STATIONARY VIEWPORT ARCHITECTURE ── */
html, body {
  height: 100vh;
  margin: 0;
  padding: 0;
  overflow: hidden; /* Prevent full-page outer bounce/scroll */
  background-color: var(--canvas-bg);
  font-family: var(--font-sans);
  color: var(--text-main);
  display: flex;
  justify-content: center;
  align-items: center;
}

/* Floating App Container fits viewport cleanly */
.app-container {
  width: calc(100vw - 36px);
  max-width: 1540px;
  height: calc(100vh - 36px);
  background: var(--app-bg);
  border-radius: var(--r-container);
  box-shadow: var(--shadow-float);
  padding: 22px 28px 24px;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}
@media (max-width: 1024px) {
  html, body { height: auto; overflow: auto; }
  .app-container { height: auto; padding: 18px 14px; border-radius: 20px; }
}

/* ── TOP HEADER (STATIONARY) ── */
.top-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 18px;
  flex-shrink: 0;
  gap: 14px;
}
.header-brand {
  display: flex;
  align-items: center;
  gap: 12px;
}
.brand-circle {
  width: 38px;
  height: 38px;
  border-radius: 50%;
  background: var(--dark-accent);
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0 4px 12px rgba(32, 31, 49, 0.2);
}
.brand-dot {
  width: 13px;
  height: 13px;
  border-radius: 50%;
  background: var(--lime-accent);
}
.brand-title {
  font-size: 19px;
  font-weight: 800;
  color: var(--text-main);
  letter-spacing: -0.03em;
}
.brand-sub {
  font-size: 11px;
  color: var(--text-muted);
  font-weight: 600;
}

.top-actions {
  display: flex;
  align-items: center;
  gap: 8px;
  flex-wrap: wrap;
}
.chip {
  padding: 6px 14px;
  border-radius: 30px;
  font-size: 11px;
  font-weight: 700;
  font-family: var(--font-mono);
  display: inline-flex;
  align-items: center;
  gap: 6px;
}
.chip-mint   { background: var(--pastel-mint); color: var(--pastel-mint-txt); }
.chip-peach  { background: var(--pastel-peach); color: var(--pastel-peach-txt); }
.chip-blue   { background: var(--pastel-blue); color: var(--pastel-blue-txt); }
.chip-purple { background: var(--pastel-purple); color: var(--pastel-purple-txt); }
.chip-btn {
  background: #FFFFFF;
  border: 1px solid var(--card-border);
  color: var(--text-main);
  cursor: pointer;
  box-shadow: 0 2px 6px rgba(0,0,0,0.03);
  transition: all 0.15s ease;
}
.chip-btn:hover { background: #F1F2F6; transform: translateY(-1px); }

/* ── 50/50 WORKSPACE: LEFT STATIONARY, RIGHT MOVEABLE ── */
.workspace-grid-5050 {
  display: grid;
  grid-template-columns: 1.18fr 1fr;
  gap: 24px;
  flex: 1;
  min-height: 0; /* Critical for inner scrolling */
  height: 100%;
}
@media (max-width: 1024px) {
  .workspace-grid-5050 {
    grid-template-columns: 1fr;
    height: auto;
    overflow: visible;
  }
}

/* ═════════════════════════════════════════════════════════════════
   LEFT SIDE: 100% STATIONARY (LOCKED, PINNED, ZERO SCROLL)
═════════════════════════════════════════════════════════════════ */
.col-camera-stationary {
  height: 100%;
  display: flex;
  flex-direction: column;
  overflow: hidden; /* Absolutely stationary */
}
.camera-card-stationary {
  background: var(--card-bg);
  border: 1px solid var(--card-border);
  border-radius: var(--r-card);
  padding: 20px;
  box-shadow: var(--shadow-card);
  height: 100%;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.card-head-compact {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
  flex-shrink: 0;
}
.card-heading {
  font-size: 15px;
  font-weight: 800;
  color: var(--text-main);
  letter-spacing: -0.02em;
}

/* Video Stage stretches to fill available stationary vertical height */
.camera-stage {
  position: relative;
  width: 100%;
  flex: 1;
  min-height: 280px;
  background: #111318;
  border-radius: 18px;
  overflow: hidden;
  cursor: crosshair;
  box-shadow: 0 8px 24px rgba(30, 27, 46, 0.08);
  display: flex;
  align-items: center;
  justify-content: center;
}
.camera-feed {
  width: 100%;
  height: 100%;
  object-fit: contain;
  display: block;
}
.camera-overlay-bar {
  position: absolute;
  top: 10px;
  left: 12px;
  right: 12px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  pointer-events: none;
}
.cam-pill-dark {
  background: rgba(32, 31, 49, 0.85);
  backdrop-filter: blur(8px);
  color: #FFFFFF;
  padding: 4px 10px;
  border-radius: 20px;
  font-size: 10px;
  font-family: var(--font-mono);
  font-weight: 700;
}
.cam-view-tabs {
  display: flex;
  gap: 4px;
  pointer-events: auto;
}
.cam-tab-btn {
  background: rgba(32, 31, 49, 0.85);
  border: none;
  color: rgba(255, 255, 255, 0.7);
  padding: 5px 11px;
  border-radius: 20px;
  font-size: 11px;
  font-weight: 700;
  font-family: var(--font-sans);
  cursor: pointer;
  transition: all 0.15s ease;
}
.cam-tab-btn.active {
  background: var(--lime-accent);
  color: var(--dark-accent);
  font-weight: 800;
}

.cam-toolbar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-top: 10px;
  margin-bottom: 12px;
  flex-wrap: wrap;
  gap: 8px;
  flex-shrink: 0;
}
.cam-tip {
  font-size: 11px;
  color: var(--text-muted);
  font-weight: 600;
}
.btn-clean {
  padding: 6px 12px;
  border-radius: 12px;
  background: #F4F5F8;
  border: 1px solid var(--card-border);
  color: var(--text-main);
  font-size: 11px;
  font-weight: 700;
  cursor: pointer;
  transition: all 0.15s ease;
}
.btn-clean:hover { background: #EAEBED; }

/* 4 Pastel Dynamics Tiles fit cleanly at bottom of stationary column */
.dynamics-row-stationary {
  display: grid;
  grid-template-columns: repeat(5, 1fr);
  gap: 8px;
  flex-shrink: 0;
}
@media (max-width: 900px) {
  .dynamics-row-stationary { grid-template-columns: repeat(3, 1fr); }
}
@media (max-width: 600px) {
  .dynamics-row-stationary { grid-template-columns: repeat(2, 1fr); }
}
.dyn-tile {
  background: #F8F9FC;
  border: 1px solid var(--card-border);
  border-radius: 16px;
  padding: 10px 12px;
  display: flex;
  flex-direction: column;
}
.dyn-icon-box {
  width: 28px;
  height: 28px;
  border-radius: 8px;
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 6px;
}
.dyn-val {
  font-size: 15px;
  font-weight: 800;
  font-family: var(--font-mono);
  color: var(--text-main);
}
.dyn-lbl {
  font-size: 9px;
  color: var(--text-muted);
  font-weight: 700;
  text-transform: uppercase;
  margin-top: 2px;
}
.dyn-sub {
  font-size: 9px;
  color: var(--text-muted);
  margin-top: 2px;
}

/* ═════════════════════════════════════════════════════════════════
   RIGHT SIDE: ONLY SIDE MOVEABLE / SCROLLABLE
═════════════════════════════════════════════════════════════════ */
.col-controllers-scrollable {
  height: 100%;
  overflow-y: auto;
  overflow-x: hidden;
  padding-right: 8px;
}
/* Sleek custom scrollbar */
.col-controllers-scrollable::-webkit-scrollbar {
  width: 6px;
}
.col-controllers-scrollable::-webkit-scrollbar-track {
  background: transparent;
}
.col-controllers-scrollable::-webkit-scrollbar-thumb {
  background: #CBD5E1;
  border-radius: 6px;
}
.col-controllers-scrollable::-webkit-scrollbar-thumb:hover {
  background: #94A3B8;
}

.edu-card {
  background: var(--card-bg);
  border: 1px solid var(--card-border);
  border-radius: var(--r-card);
  padding: 20px;
  box-shadow: var(--shadow-card);
  margin-bottom: 16px;
}

/* Hero Button */
.hero-action-btn {
  width: 100%;
  padding: 15px;
  border-radius: 16px;
  border: none;
  font-size: 14px;
  font-weight: 800;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 10px;
  box-shadow: 0 4px 14px rgba(0, 0, 0, 0.06);
  transition: all 0.2s ease;
}
.hero-btn-lime {
  background: var(--lime-accent);
  color: var(--dark-accent);
}
.hero-btn-lime:hover { background: var(--lime-hover); transform: translateY(-1px); }
.hero-btn-stop {
  background: #F87171;
  color: #FFFFFF;
}

/* Strategy Grid */
.strategy-grid {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 8px;
  margin-top: 12px;
}
.strategy-btn {
  padding: 9px 10px;
  border-radius: 12px;
  background: #F4F5F8;
  border: 1px solid var(--card-border);
  font-size: 11px;
  font-weight: 700;
  color: var(--text-muted);
  cursor: pointer;
  transition: all 0.15s ease;
}
.strategy-btn.active {
  background: var(--dark-accent);
  color: var(--lime-accent);
  font-weight: 800;
}

/* ── CAMERA HUD RETICLE CORNERS ── */
.hud-corner {
  position: absolute;
  width: 14px;
  height: 14px;
  pointer-events: none;
  z-index: 4;
}
.hud-corner.top-left {
  top: 10px;
  left: 10px;
  border-top: 2px solid rgba(212, 249, 115, 0.75);
  border-left: 2px solid rgba(212, 249, 115, 0.75);
}
.hud-corner.top-right {
  top: 10px;
  right: 10px;
  border-top: 2px solid rgba(212, 249, 115, 0.75);
  border-right: 2px solid rgba(212, 249, 115, 0.75);
}
.hud-corner.bottom-left {
  bottom: 10px;
  left: 10px;
  border-bottom: 2px solid rgba(212, 249, 115, 0.75);
  border-left: 2px solid rgba(212, 249, 115, 0.75);
}
.hud-corner.bottom-right {
  bottom: 10px;
  right: 10px;
  border-bottom: 2px solid rgba(212, 249, 115, 0.75);
  border-right: 2px solid rgba(212, 249, 115, 0.75);
}

/* ── ULTRASONIC PROXIMITY SLIDER (Replaces Network Bars) ── */
.sonar-activity-box {
  background: #F8F9FC;
  border-radius: 16px;
  padding: 14px 16px;
  margin-top: 10px;
  border: 1px solid var(--card-border);
}
.sonar-slider-wrap {
  margin-top: 12px;
  margin-bottom: 6px;
  position: relative;
  padding: 16px 6px 14px 6px;
}
.sonar-slider-track-box {
  position: relative;
  height: 24px;
  display: flex;
  align-items: center;
}
.sonar-slider-track {
  position: relative;
  width: 100%;
  height: 12px;
  background: #E2E8F0;
  border-radius: 999px;
  display: flex;
  overflow: visible;
  border: 1px solid rgba(0, 0, 0, 0.06);
}
.sonar-zone-segment {
  height: 100%;
}
.sonar-zone-segment.zone-danger {
  width: 23.3%;
  background: #FEE2E2;
  border-top-left-radius: 999px;
  border-bottom-left-radius: 999px;
}
.sonar-zone-segment.zone-caution {
  width: 20%;
  background: #FEF3C7;
}
.sonar-zone-segment.zone-clear {
  width: 56.7%;
  background: #DCFCE7;
  border-top-right-radius: 999px;
  border-bottom-right-radius: 999px;
}
.sonar-active-fill {
  position: absolute;
  left: 0;
  top: 0;
  height: 100%;
  border-radius: 999px;
  background: linear-gradient(90deg, #10B981, #D4F973);
  opacity: 0.9;
  transition: width 0.15s ease-out, background 0.2s ease;
  pointer-events: none;
}
.sonar-threshold-pin {
  position: absolute;
  top: -4px;
  bottom: -4px;
  width: 2px;
  background: #EF4444;
  transform: translateX(-50%);
  z-index: 2;
  pointer-events: none;
}
.threshold-flag {
  position: absolute;
  top: -20px;
  left: 50%;
  transform: translateX(-50%);
  font-size: 8px;
  font-weight: 800;
  color: #DC2626;
  background: #FEE2E2;
  border: 1px solid #FCA5A5;
  padding: 1px 5px;
  border-radius: 4px;
  white-space: nowrap;
}
.sonar-slider-thumb {
  position: absolute;
  top: 50%;
  width: 22px;
  height: 22px;
  transform: translate(-50%, -50%);
  background: #FFFFFF;
  border: 3px solid #10B981;
  border-radius: 50%;
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.2), 0 0 10px rgba(16, 185, 129, 0.4);
  transition: left 0.15s cubic-bezier(0.4, 0, 0.2, 1), border-color 0.2s, box-shadow 0.2s;
  z-index: 5;
  display: flex;
  align-items: center;
  justify-content: center;
}
.sonar-thumb-pin {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  background: #10B981;
  transition: background 0.2s;
}
.sonar-thumb-bubble {
  position: absolute;
  bottom: 26px;
  left: 50%;
  transform: translateX(-50%);
  background: #201F31;
  color: #FFFFFF;
  font-size: 11px;
  font-weight: 800;
  font-family: var(--font-mono);
  padding: 3px 8px;
  border-radius: 8px;
  white-space: nowrap;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.25);
}
.sonar-thumb-bubble::after {
  content: '';
  position: absolute;
  bottom: -4px;
  left: 50%;
  transform: translateX(-50%);
  border-left: 4px solid transparent;
  border-right: 4px solid transparent;
  border-top: 4px solid #201F31;
}
.sonar-scale-ticks {
  position: relative;
  width: 100%;
  height: 22px;
  margin-top: 8px;
}
.tick-mark {
  position: absolute;
  transform: translateX(-50%);
  font-size: 10px;
  font-weight: 700;
  color: var(--text-muted);
  text-align: center;
  line-height: 1.1;
  white-space: nowrap;
}
.tick-mark small {
  font-size: 8px;
  font-weight: 600;
  display: block;
}
.tick-danger small { color: #DC2626; }
.tick-detour small { color: #DC2626; font-weight: 800; }
.tick-caution small { color: #D97706; }
.tick-clear small { color: #16A34A; }


/* 3-Sector Radar */
.sector-pills-row {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 8px;
  margin-top: 10px;
}
.sec-pill-card {
  background: #F8F9FC;
  border: 1px solid var(--card-border);
  border-radius: 14px;
  padding: 9px;
  text-align: center;
}
.sec-pct {
  font-size: 15px;
  font-weight: 800;
  font-family: var(--font-mono);
  color: var(--text-main);
}
.sec-label {
  font-size: 10px;
  color: var(--text-muted);
  font-weight: 600;
  margin-top: 2px;
}

/* Tactical WASD Deck */
.wasd-deck {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 8px;
  margin-top: 10px;
}
.wasd-row {
  display: flex;
  gap: 8px;
}
.wasd-btn {
  width: 54px;
  height: 46px;
  border-radius: 14px;
  background: #F4F5F8;
  border: 1px solid var(--card-border);
  font-size: 13px;
  font-weight: 800;
  color: var(--text-main);
  cursor: pointer;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.03);
  transition: all 0.1s ease;
}
.wasd-btn:active, .wasd-btn.active {
  background: var(--dark-accent);
  color: var(--lime-accent);
  transform: scale(0.96);
}
.wasd-btn-stop {
  width: 178px;
  height: 38px;
  background: #FEE2E2;
  border-color: #FECACA;
  color: #DC2626;
  font-size: 11px;
}
.wasd-btn-stop:active, .wasd-btn-stop.active {
  background: #DC2626;
  color: #FFFFFF;
}

/* Camera Servo Slider */
.slider-container {
  margin-top: 12px;
}
.range-clean {
  width: 100%;
  height: 6px;
  background: #E2E8F0;
  border-radius: 4px;
  accent-color: var(--dark-accent);
  outline: none;
}

/* ArUco Landmarks & Route */
.assignment-item {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 9px 12px;
  border-radius: 14px;
  background: #F8F9FC;
  border: 1px solid var(--card-border);
  margin-bottom: 7px;
  cursor: pointer;
  transition: all 0.15s ease;
}
.assignment-item.active {
  background: #FFFFFF;
  border-color: #CBD5E1;
  box-shadow: var(--shadow-card);
}
.assign-title {
  font-size: 12px;
  font-weight: 700;
  color: var(--text-main);
}
.assign-sub {
  font-size: 10px;
  color: var(--text-muted);
}
.status-pill-badge {
  padding: 4px 10px;
  border-radius: 12px;
  font-size: 10px;
  font-weight: 700;
}

/* Sidebar Options Integrator Deck */
.options-deck-card {
  display: flex;
  align-items: center;
  justify-content: space-between;
  background: var(--lime-accent);
  border-radius: 18px;
  padding: 13px 16px;
  margin-top: 12px;
}
.deck-title {
  font-size: 13px;
  font-weight: 800;
  color: var(--dark-accent);
}
.deck-sub {
  font-size: 10px;
  color: rgba(32, 31, 49, 0.7);
  margin-top: 2px;
}
.deck-btn-circle {
  width: 32px;
  height: 32px;
  background: #FFFFFF;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  color: var(--dark-accent);
  font-size: 13px;
  font-weight: 800;
  box-shadow: 0 4px 10px rgba(0, 0, 0, 0.08);
  cursor: pointer;
}

/* ── MODALS ── */
.modal-overlay {
  display: none;
  position: fixed;
  top: 0; left: 0; right: 0; bottom: 0;
  background: rgba(30, 27, 46, 0.6);
  backdrop-filter: blur(6px);
  z-index: 9999;
  align-items: center;
  justify-content: center;
  padding: 20px;
}
.modal-window {
  background: #FFFFFF;
  border-radius: 28px;
  box-shadow: var(--shadow-float);
  max-width: 800px;
  width: 100%;
  max-height: 90vh;
  overflow: hidden;
  display: flex;
  flex-direction: column;
}
.modal-head {
  padding: 18px 24px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid var(--card-border);
}
.modal-body {
  padding: 24px;
  overflow-y: auto;
}
</style>
</head>
<body>

<div class="app-container">

  <!-- ── TOP HEADER (STATIONARY) ── -->
  <header class="top-header">
    <div class="header-brand">
      <div class="brand-circle">
        <div class="brand-dot"></div>
      </div>
      <div>
        <div class="brand-title">RoboPilot • Autonomous Mission Cockpit</div>
        <div class="brand-sub">Stationary Horizon Deck · Neural Pilot & Computer Vision GCS</div>
      </div>
    </div>

    <div class="top-actions">
      <div class="chip chip-mint" id="chip-mode">● Mode: MANUAL</div>
      <div class="chip chip-purple" id="chip-serial">Serial: /dev/ttyACM0</div>
      <div class="chip chip-blue" id="chip-ai">AI: SidewalkPilot ONNX</div>
      <div class="chip chip-mint" id="chip-sonar" style="font-weight:700">
        <span style="display:inline-block;width:7px;height:7px;border-radius:50%;background:#16A34A;margin-right:4px"></span>
        Ultrasonic: <span id="top-sonar-dist">-- cm</span>
      </div>
      <div class="chip chip-peach" id="chip-temp">CPU: 42°C</div>
      <button class="chip chip-btn" onclick="openMarkerModal()">🏷️ ArUco PDF</button>
      <button class="chip chip-btn" id="btn-standby-top" onclick="toggleStandby()">❄️ Standby: OFF</button>
    </div>
  </header>

  <!-- ── 50/50 DUAL DECK: LEFT STATIONARY, RIGHT MOVEABLE ── -->
  <div class="workspace-grid-5050">

    <!-- ═══════════════════════════════════════════════════════════════
         LEFT 1/2: 100% STATIONARY FULL CAMERA VISION (NO SCROLL)
    ═══════════════════════════════════════════════════════════════ -->
    <div class="col-camera-stationary">

      <div class="camera-card-stationary">
        <div class="card-head-compact">
          <div class="card-heading">Live Camera Feed & Perception Horizon</div>
          <div class="cam-view-tabs">
            <button class="cam-tab-btn active" id="tab-hud" onclick="switchView('hud')">🎯 3D HUD</button>
            <button class="cam-tab-btn" id="tab-birdseye" onclick="switchView('birdseye')">🦅 Bird's-Eye</button>
            <button class="cam-tab-btn" id="tab-raw" onclick="switchView('raw')">📷 Raw</button>
          </div>
        </div>

        <!-- Video Canvas (Flexible Fill) -->
        <div class="camera-stage" onclick="onCamClick(event)">
          <img id="cam-img" class="camera-feed" src="/stream?view=hud" alt="Camera Stream">
          <video id="cam-video" class="camera-feed" style="display:none" autoplay muted playsinline></video>
          <!-- Sleek Glassmorphic HUD Reticle Corners -->
          <div class="hud-corner top-left"></div>
          <div class="hud-corner top-right"></div>
          <div class="hud-corner bottom-left"></div>
          <div class="hud-corner bottom-right"></div>
          <div class="camera-overlay-bar">
            <div class="cam-pill-dark" id="cam-badge-text">LIVE · 3D PERCEPTION HUD</div>
            <div class="cam-pill-dark" id="cam-sonar-badge" style="background:rgba(15,23,42,0.85);color:#86EFAC;font-weight:700;border:1px solid rgba(134,239,172,0.3)">📡 US: <span id="cam-sonar-val">-- cm</span></div>
            <div class="cam-pill-dark" id="cam-fps-badge">30.0 FPS</div>
          </div>
        </div>

        <!-- Video Toolbar -->
        <div class="cam-toolbar">
          <div class="cam-tip" id="cam-status">
            💡 Click on camera stream to set a visual waypoint target.
          </div>
          <div style="display:flex;gap:6px">
            <button class="btn-clean" id="btn-swap-rb" onclick="toggleSwapRB()">Swap Colors</button>
            <button class="btn-clean" onclick="clearDest()">Clear Goal</button>
            <a href="/snapshot" target="_blank" style="text-decoration:none">
              <button class="btn-clean">📸 Snapshot</button>
            </a>
          </div>
        </div>

        <!-- 5 Pastel Dynamics Tiles (Fitted Stationarily at Bottom) -->
        <div class="dynamics-row-stationary">
          <div class="dyn-tile">
            <div class="dyn-icon-box" style="background:var(--pastel-peach)">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--pastel-peach-txt)" stroke-width="2.5">
                <circle cx="12" cy="12" r="10"/><polygon points="12 8 8 12 12 16 16 12 12 8"/>
              </svg>
            </div>
            <div class="dyn-val" id="val-fused-steer">0.00</div>
            <div class="dyn-lbl">Fused Steering</div>
            <div class="dyn-sub" id="val-fused-angle">Angle: 0°</div>
          </div>

          <div class="dyn-tile">
            <div class="dyn-icon-box" style="background:var(--pastel-mint)">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--pastel-mint-txt)" stroke-width="2.5">
                <path d="M18 10h-1.26A8 8 0 1 0 9 20h9a5 5 0 0 0 0-10z"/>
              </svg>
            </div>
            <div class="dyn-val" id="val-lateral-err">0.0 px</div>
            <div class="dyn-lbl">Lateral Offset</div>
            <div class="dyn-sub" id="val-lane-dir">Dir: STRAIGHT</div>
          </div>

          <div class="dyn-tile" id="tile-sonar">
            <div class="dyn-icon-box" id="tile-sonar-icon" style="background:var(--pastel-mint)">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--pastel-mint-txt)" stroke-width="2.5">
                <path d="M12 2a10 10 0 0 1 10 10c0 4.5-3 8.3-7.2 9.5"/>
                <path d="M12 6a6 6 0 0 1 6 6c0 2.7-1.8 5-4.3 5.7"/>
                <circle cx="12" cy="12" r="2" fill="var(--pastel-mint-txt)"/>
              </svg>
            </div>
            <div class="dyn-val" id="val-sonar-dist">-- cm</div>
            <div class="dyn-lbl">Ultrasonic Sensor</div>
            <div class="dyn-sub" id="val-sonar-status">Path Clear</div>
          </div>

          <div class="dyn-tile">
            <div class="dyn-icon-box" style="background:var(--pastel-blue)">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--pastel-blue-txt)" stroke-width="2.5">
                <path d="M21 10c0 7-9 13-9 13s-9-6-9-13a9 9 0 0 1 18 0z"/><circle cx="12" cy="10" r="3"/>
              </svg>
            </div>
            <div class="dyn-val" id="val-target-name">Tag #0</div>
            <div class="dyn-lbl">Target Landmark</div>
            <div class="dyn-sub" id="val-target-dist">Searching...</div>
          </div>

          <div class="dyn-tile">
            <div class="dyn-icon-box" style="background:var(--pastel-purple)">
              <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="var(--pastel-purple-txt)" stroke-width="2.5">
                <polygon points="13 2 3 14 12 14 11 22 21 10 12 10 13 2"/>
              </svg>
            </div>
            <div class="dyn-val" id="val-veh-state">IDLE</div>
            <div class="dyn-lbl">Vehicle State</div>
            <div class="dyn-sub" id="val-veh-sub">Ready to Drive</div>
          </div>
        </div>

      </div>

    </div>

    <!-- ═══════════════════════════════════════════════════════════════
         RIGHT 1/2: ONLY SIDE MOVEABLE & SCROLLABLE (CONTROLLERS)
    ═══════════════════════════════════════════════════════════════ -->
    <div class="col-controllers-scrollable">

      <!-- 1. Mission Control Hero Button & Strategy -->
      <div class="edu-card">
        <div class="card-head-compact">
          <div class="card-heading">Mission Control & Strategy</div>
          <span style="font-size:11px;font-weight:700;color:var(--text-muted)">Master Switch</span>
        </div>

        <button class="hero-action-btn hero-btn-lime" id="btn-hero-auto" onclick="toggleAuto()">
          ▶ Start Autonomous Mission
        </button>

        <div style="margin-top:14px">
          <div style="font-size:11px;font-weight:700;color:var(--text-muted);margin-bottom:6px">
            Navigation Strategy Mode
          </div>
          <div class="strategy-grid">
            <button class="strategy-btn active" id="mode-hybrid" onclick="setNavMode('HYBRID_FUSION')">⚡ Hybrid Fusion</button>
            <button class="strategy-btn" id="mode-lane" onclick="setNavMode('LANE_FOLLOWING')">🛣️ Lane Centering</button>
            <button class="strategy-btn" id="mode-classic" onclick="setNavMode('SANOGUZHAN_CLASSIC')">🛞 Sanoguzhan Classic</button>
            <button class="strategy-btn" id="mode-aruco" onclick="setNavMode('ARUCO_WAYPOINT')">🏷️ ArUco Waypoint</button>
            <button class="strategy-btn" id="mode-ai" onclick="setNavMode('AI_ONNX_PILOT')">🧠 ONNX AI Pilot</button>
          </div>
        </div>
      </div>

      <!-- 2. Ultrasonic Sensor Proximity & Spatial Radar -->
      <div class="edu-card">
        <div class="card-head-compact">
          <div class="card-heading">Ultrasonic Sensor & Spatial Radar</div>
          <span class="chip chip-mint" style="font-size:10px;font-weight:700">STATIC HC-SR04</span>
        </div>

        <div class="sonar-activity-box">
          <div style="display:flex;justify-content:space-between;align-items:flex-end;margin-bottom:12px">
            <div>
              <div style="font-size:11px;font-weight:700;color:var(--text-muted);text-transform:uppercase;letter-spacing:0.5px">Stationary Front Bumper Reading</div>
              <div style="font-size:32px;font-weight:900;font-family:var(--font-mono);color:var(--text-main);line-height:1.1;margin-top:2px" id="sonar-dist-text">-- cm</div>
              <div style="font-size:11px;color:var(--text-muted);margin-top:2px">Detour Threshold: <b>35.0 cm</b> · Front Static Mount</div>
            </div>
            <div style="text-align:right">
              <div class="chip chip-mint" id="sonar-status-chip" style="font-size:12px;font-weight:800;padding:6px 14px">CLEAR PATH</div>
            </div>
          </div>

          <!-- Ultrasonic Distance Proximity Slider (Replaces Network Bars) -->
          <div class="sonar-slider-wrap">
            <div class="sonar-slider-track-box">
              <div class="sonar-slider-track">
                <!-- Zone Color Segments: Danger (0-35cm), Caution (35-65cm), Clear (65-150cm) -->
                <div class="sonar-zone-segment zone-danger"></div>
                <div class="sonar-zone-segment zone-caution"></div>
                <div class="sonar-zone-segment zone-clear"></div>

                <!-- Dynamic Active Progress Fill -->
                <div class="sonar-active-fill" id="sonar-active-fill" style="width: 45%;"></div>

                <!-- 35.0 cm Autonomous Detour Threshold Pin -->
                <div class="sonar-threshold-pin" style="left: 23.3%;" title="35.0 cm Autonomous Detour Threshold">
                  <span class="threshold-flag">35cm DETOUR</span>
                </div>

                <!-- Real-Time Sliding Cursor / Thumb -->
                <div class="sonar-slider-thumb" id="sonar-slider-thumb" style="left: 45%;">
                  <div class="sonar-thumb-pin" id="sonar-thumb-pin"></div>
                  <div class="sonar-thumb-bubble" id="sonar-thumb-bubble">68 cm</div>
                </div>
              </div>
            </div>

            <!-- Distance Ruler & Zone Scale Ticks -->
            <div class="sonar-scale-ticks">
              <span class="tick-mark tick-danger" style="left: 0%">0 cm<small>Bumper</small></span>
              <span class="tick-mark tick-detour" style="left: 23.3%">35 cm<small>Detour</small></span>
              <span class="tick-mark tick-caution" style="left: 43.3%">65 cm<small>Caution</small></span>
              <span class="tick-mark" style="left: 66.6%">100 cm<small>Safe</small></span>
              <span class="tick-mark tick-clear" style="left: 100%">150+ cm<small>Clear</small></span>
            </div>
          </div>
        </div>

        <!-- 3-Sector Radar -->
        <div class="sector-pills-row">
          <div class="sec-pill-card">
            <div class="sec-pct" id="clr-left">100%</div>
            <div class="sec-label">Left (125°)</div>
          </div>
          <div class="sec-pill-card">
            <div class="sec-pct" id="clr-center" style="color:var(--pastel-mint-txt)">100%</div>
            <div class="sec-label">Center (90°)</div>
          </div>
          <div class="sec-pill-card">
            <div class="sec-pct" id="clr-right">100%</div>
            <div class="sec-label">Right (55°)</div>
          </div>
        </div>
      </div>

      <!-- 3. Tactical Actuation Deck (WASD) -->
      <div class="edu-card">
        <div class="card-head-compact">
          <div class="card-heading">Manual Driving (WASD)</div>
          <span style="font-size:11px;color:var(--text-muted);font-weight:600">Keyboard Active</span>
        </div>

        <div class="wasd-deck">
          <button class="wasd-btn" id="btn-w"
                  onmousedown="pressCmd('FORWARD')" onmouseup="releaseCmd()"
                  ontouchstart="pressCmd('FORWARD')" ontouchend="releaseCmd()">
            ▲<span style="font-size:9px">W</span>
          </button>
          <div class="wasd-row">
            <button class="wasd-btn" id="btn-a"
                    onmousedown="pressCmd('LEFT')" onmouseup="releaseCmd()"
                    ontouchstart="pressCmd('LEFT')" ontouchend="releaseCmd()">
              ◀<span style="font-size:9px">A</span>
            </button>
            <button class="wasd-btn" id="btn-s"
                    onmousedown="pressCmd('BACKWARD')" onmouseup="releaseCmd()"
                    ontouchstart="pressCmd('BACKWARD')" ontouchend="releaseCmd()">
              ▼<span style="font-size:9px">S</span>
            </button>
            <button class="wasd-btn" id="btn-d"
                    onmousedown="pressCmd('RIGHT')" onmouseup="releaseCmd()"
                    ontouchstart="pressCmd('RIGHT')" ontouchend="releaseCmd()">
              ▶<span style="font-size:9px">D</span>
            </button>
          </div>
          <div style="display:grid;grid-template-columns:repeat(2,1fr);gap:6px;margin-top:8px;max-width:260px">
            <button class="btn-clean" onmousedown="pressCmd('SLIDE_LEFT')" onmouseup="releaseCmd()"
                    ontouchstart="pressCmd('SLIDE_LEFT')" ontouchend="releaseCmd()">↙ Slide Left</button>
            <button class="btn-clean" onmousedown="pressCmd('SLIDE_RIGHT')" onmouseup="releaseCmd()"
                    ontouchstart="pressCmd('SLIDE_RIGHT')" ontouchend="releaseCmd()">↘ Slide Right</button>
            <button class="btn-clean" onmousedown="pressCmd('HARD_LEFT')" onmouseup="releaseCmd()"
                    ontouchstart="pressCmd('HARD_LEFT')" ontouchend="releaseCmd()">⟲ Hard Left</button>
            <button class="btn-clean" onmousedown="pressCmd('HARD_RIGHT')" onmouseup="releaseCmd()"
                    ontouchstart="pressCmd('HARD_RIGHT')" ontouchend="releaseCmd()">⟳ Hard Right</button>
          </div>
          <div style="font-size:10px;color:var(--text-muted);margin-top:5px">
            Hold W (or &uarr;) and add A/D (or &larr;/&rarr;) to arc while moving. A/D alone pivots in place.
            Slide = gentle forward arc; Hard = in-place pivot. Arrow keys on the camera slider move the camera only.
          </div>
          <button class="wasd-btn wasd-btn-stop" id="btn-space" onclick="sendStop()">
            ⏹ EMERGENCY STOP (SPACE)
          </button>
        </div>

        <!-- Camera Tilt / Pan Slider -->
        <div class="slider-container">
          <div style="display:flex;justify-content:space-between;font-size:11px;font-weight:700;color:var(--text-muted);margin-bottom:6px">
            <span>Camera Tilt / Pan</span>
            <span id="servo-display" style="color:var(--text-main)">90°</span>
          </div>
          <input type="range" class="range-clean" id="servo-slider" min="30" max="150" value="90" oninput="sendServo(this.value)">
          <div style="display:flex;gap:6px;margin-top:8px">
            <button class="btn-clean" style="flex:1" onclick="sendServo(50)">50° Right</button>
            <button class="btn-clean" style="flex:1" onclick="sendServo(90)">90° Center</button>
            <button class="btn-clean" style="flex:1" onclick="sendServo(130)">130° Left</button>
          </div>
          <!-- Fixed side looks: turn to a set angle, check for an obstacle, warn, return to centre -->
          <div style="display:flex;gap:6px;margin-top:8px">
            <button class="btn-clean" style="flex:1" onclick="lookSide('left')">◀ Look left</button>
            <button class="btn-clean" style="flex:1" onclick="lookSide('check')">↔ Check sides</button>
            <button class="btn-clean" style="flex:1" onclick="lookSide('right')">Look right ▶</button>
          </div>
          <div class="chip chip-mint" id="side-look-chip" style="margin-top:8px;font-size:11px;font-weight:700;text-align:center">Sides: not checked</div>
        </div>
      </div>

      <!-- 4. ArUco Landmarks & Route -->
      <div class="edu-card">
        <div class="card-head-compact">
          <div class="card-heading">ArUco Landmark Waypoints</div>
          <button class="btn-clean" onclick="openMarkerModal()">🖨️ Tags</button>
        </div>

        <div class="assignment-item active" id="tag-item-0" onclick="setArucoTarget(0)">
          <div>
            <div class="assign-title">Tag #0 (Dock / Home)</div>
            <div class="assign-sub">Base landmark station</div>
          </div>
          <span class="status-pill-badge" style="background:var(--pastel-mint);color:var(--pastel-mint-txt)" id="tag-badge-0">Active</span>
        </div>

        <div class="assignment-item" id="tag-item-1" onclick="setArucoTarget(1)">
          <div>
            <div class="assign-title">Tag #1 (Waypoint Alpha)</div>
            <div class="assign-sub">First corridor intersection</div>
          </div>
          <span class="status-pill-badge" style="background:#F1F2F6;color:var(--text-muted)" id="tag-badge-1">Standby</span>
        </div>

        <div class="assignment-item" id="tag-item-2" onclick="setArucoTarget(2)">
          <div>
            <div class="assign-title">Tag #2 (Waypoint Beta)</div>
            <div class="assign-sub">Corner turn marker</div>
          </div>
          <span class="status-pill-badge" style="background:#F1F2F6;color:var(--text-muted)" id="tag-badge-2">Standby</span>
        </div>

        <div class="assignment-item" id="tag-item-3" onclick="setArucoTarget(3)">
          <div>
            <div class="assign-title">Tag #3 (Final Goal)</div>
            <div class="assign-sub">Mission destination</div>
          </div>
          <span class="status-pill-badge" style="background:#F1F2F6;color:var(--text-muted)" id="tag-badge-3">Standby</span>
        </div>

        <!-- Linked Route -->
        <div style="background:#F8F9FC;border-radius:14px;padding:12px;margin-top:10px">
          <div style="display:flex;justify-content:space-between;font-size:11px;font-weight:700">
            <span>Linked Route Sequence</span>
            <span id="route-active-lbl" style="color:var(--pastel-purple-txt)">Link 1 of 3</span>
          </div>
          <input type="text" id="route-input-list" value="0, 1, 2" style="width:100%;margin-top:6px;padding:6px 10px;border-radius:8px;border:1px solid var(--card-border);font-family:var(--font-mono);font-size:12px">
          <div style="display:flex;gap:6px;margin-top:8px">
            <button class="btn-clean" id="btn-route-toggle" style="background:var(--lime-accent);color:var(--dark-accent);font-weight:800" onclick="toggleRouteMode()">▶ Linked Route</button>
            <button class="btn-clean" onclick="advanceRoute()">Next ➔</button>
            <button class="btn-clean" onclick="setRouteList()">Save</button>
          </div>
        </div>

        <!-- Integrated Calibration PDF Option -->
        <div class="options-deck-card">
          <div>
            <div class="deck-title">Download Print Calibration PDF</div>
            <div class="deck-sub">Physical 10.0cm metric ArUco tags for 3D homing</div>
          </div>
          <a href="/download/aruco_markers.pdf" target="_blank" style="text-decoration:none">
            <div class="deck-btn-circle">↗</div>
          </a>
        </div>
      </div>

    </div>

  </div>

</div>

<!-- ── MODAL: ARUCO MARKERS PDF ── -->
<div class="modal-overlay" id="marker-modal" onclick="closeMarkerModal()">
  <div class="modal-window" onclick="event.stopPropagation()">
    <div class="modal-head">
      <div style="font-size:15px;font-weight:800;color:var(--text-main)">🏷️ Calibrated ArUco Navigation Markers</div>
      <button style="border:none;background:none;font-size:18px;cursor:pointer" onclick="closeMarkerModal()">✕</button>
    </div>
    <div class="modal-body" style="text-align:center">
      <p style="font-size:12px;color:var(--text-muted);margin-bottom:14px">
        Calibrated for 10.0 cm metric 3D homing. Display on phone screen or download the print-ready PDF.
      </p>
      <div style="display:flex;justify-content:center;gap:12px;margin-bottom:18px;flex-wrap:wrap">
        <img src="/arucotag?id=0" style="width:110px;height:110px;border-radius:12px;border:2px solid #E2E8F0" alt="Tag 0">
        <img src="/arucotag?id=1" style="width:110px;height:110px;border-radius:12px;border:2px solid #E2E8F0" alt="Tag 1">
        <img src="/arucotag?id=2" style="width:110px;height:110px;border-radius:12px;border:2px solid #E2E8F0" alt="Tag 2">
      </div>
      <a href="/download/aruco_markers.pdf" target="_blank" style="text-decoration:none">
        <button class="hero-action-btn hero-btn-lime" style="max-width:320px;margin:0 auto">
          📥 Download Print-Ready PDF
        </button>
      </a>
    </div>
  </div>
</div>

<!-- ── LIVE CONTROLLER JAVASCRIPT ── -->
<script>
let activeView = 'hud';
let isAuto = false;
let selectedTagId = 0;
let currentNavMode = 'HYBRID_FUSION';

function switchView(view) {
  activeView = view;
  ['hud', 'birdseye', 'raw'].forEach(v => {
    const el = document.getElementById('tab-' + v);
    if (el) el.className = 'cam-tab-btn' + (v === view ? ' active' : '');
  });
  const bText = document.getElementById('cam-badge-text');
  if (view === 'birdseye') bText.textContent = "SANOGUZHAN BIRD'S-EYE (LKA)";
  else if (view === 'raw') bText.textContent = "RAW SENSOR (400x240)";
  else bText.textContent = "LIVE · 3D PERCEPTION HUD";

  showView(view);
}

// ── Video transport: WebRTC (H.264) first, MJPEG as the fallback ──
let rtcPc = null, rtcGen = 0, rtcTimer = null;

function useMjpeg(view) {
  rtcGen++;
  if (rtcPc) { try { rtcPc.close(); } catch (e) {} rtcPc = null; }
  clearTimeout(rtcTimer);
  document.getElementById('cam-video').style.display = 'none';
  const img = document.getElementById('cam-img');
  img.style.display = '';
  img.src = '/stream?view=' + view + '&t=' + Date.now();
}

function showView(view) {
  if (!window.RTCPeerConnection) { useMjpeg(view); return; }
  startRtc(view).catch(() => useMjpeg(view));
}

async function startRtc(view) {
  const gen = ++rtcGen;
  if (rtcPc) { try { rtcPc.close(); } catch (e) {} }
  clearTimeout(rtcTimer);
  const pc = new RTCPeerConnection({ iceServers: [] });
  rtcPc = pc;
  pc.addTransceiver('video', { direction: 'recvonly' });
  const vid = document.getElementById('cam-video');
  pc.ontrack = (e) => { vid.srcObject = e.streams[0]; };
  pc.onconnectionstatechange = () => {
    if (gen !== rtcGen) return;
    if (pc.connectionState === 'connected') {
      clearTimeout(rtcTimer);
      vid.style.display = '';
      const img = document.getElementById('cam-img');
      img.style.display = 'none';
      img.removeAttribute('src');   // stop the MJPEG connection
    } else if (['failed', 'disconnected', 'closed'].includes(pc.connectionState)) {
      useMjpeg(view);
    }
  };
  rtcTimer = setTimeout(() => { if (gen === rtcGen && pc.connectionState !== 'connected') useMjpeg(view); }, 6000);

  await pc.setLocalDescription(await pc.createOffer());
  await new Promise((resolve) => {   // wait for local candidates (LAN only: fast)
    if (pc.iceGatheringState === 'complete') return resolve();
    pc.addEventListener('icegatheringstatechange', () => { if (pc.iceGatheringState === 'complete') resolve(); });
    setTimeout(resolve, 1500);
  });
  const res = await fetch('/webrtc/offer', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ sdp: pc.localDescription.sdp, type: pc.localDescription.type, view: view }),
  });
  if (!res.ok) throw new Error('offer rejected');
  if (gen !== rtcGen) return;
  await pc.setRemoteDescription(await res.json());
}

function onCamClick(e) {
  const vid = document.getElementById('cam-video');
  const img = vid.style.display === 'none' ? document.getElementById('cam-img') : vid;
  const rect = img.getBoundingClientRect();
  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;
  const scaleX = 400 / rect.width;
  const scaleY = 240 / rect.height;
  const tx = Math.round(clickX * scaleX);
  const ty = Math.round(clickY * scaleY);

  fetch('/destination?x=' + tx + '&y=' + ty)
    .then(r => r.json())
    .then(() => {
      document.getElementById('cam-status').textContent = '🎯 Goal locked at (' + tx + ', ' + ty + ') px. Click Start Mission to drive!';
    }).catch(() => {});
}

function clearDest() {
  fetch('/cleardest').then(() => {
    document.getElementById('cam-status').textContent = 'Goal cleared. Ready.';
  }).catch(() => {});
}

function setArucoTarget(tagId) {
  selectedTagId = tagId;
  fetch('/aruco?target=' + tagId).then(r => r.json()).then(() => {
    [0, 1, 2, 3].forEach(id => {
      const el = document.getElementById('tag-item-' + id);
      const bd = document.getElementById('tag-badge-' + id);
      if (el) el.className = 'assignment-item' + (id === tagId ? ' active' : '');
      if (bd) {
        bd.textContent = (id === tagId ? 'Active' : 'Standby');
        bd.style.background = (id === tagId ? 'var(--pastel-mint)' : '#F1F2F6');
        bd.style.color = (id === tagId ? 'var(--pastel-mint-txt)' : 'var(--text-muted)');
      }
    });
    document.getElementById('val-target-name').textContent = 'Tag #' + tagId;
  }).catch(() => {});
}

function toggleAuto(forceState) {
  const target = forceState !== undefined ? forceState : !isAuto;
  fetch('/auto?action=' + (target ? 'start' : 'stop'))
    .then(r => r.json())
    .then(d => {
      isAuto = d.auto;
      updateAutoUI(d.auto);
    }).catch(() => {});
}

function updateAutoUI(auto) {
  const btn = document.getElementById('btn-hero-auto');
  const chip = document.getElementById('chip-mode');
  if (auto) {
    btn.className = 'hero-action-btn hero-btn-stop';
    btn.textContent = '⏹ Stop Autonomous Mission';
    chip.className = 'chip chip-peach';
    chip.textContent = '● Mode: AUTONOMOUS';
  } else {
    btn.className = 'hero-action-btn hero-btn-lime';
    btn.textContent = '▶ Start Autonomous Mission';
    chip.className = 'chip chip-mint';
    chip.textContent = '● Mode: ' + currentNavMode;
  }
}

function toggleStandby() {
  fetch('/standby?action=toggle')
    .then(r => r.json())
    .then(d => updateStandbyUI(d.standby, d.cpu_temp))
    .catch(() => {});
}

let isStandbyMode = false;
function updateStandbyUI(standby, temp) {
  isStandbyMode = !!standby;
  const b = document.getElementById('btn-standby-top');
  if (b) {
    b.textContent = standby ? '❄️ Standby: ACTIVE' : '❄️ Standby: OFF';
    b.style.background = standby ? 'var(--pastel-peach)' : '#FFFFFF';
    b.style.color = standby ? 'var(--pastel-peach-txt)' : 'var(--text-main)';
  }
}

function toggleSwapRB() {
  fetch('/camalign?swap_rb=toggle').then(r => r.json()).then(d => {
    const b = document.getElementById('btn-swap-rb');
    if (b) {
      b.style.background = d.swap_rb ? 'var(--pastel-peach)' : '#F4F5F8';
      b.style.color = d.swap_rb ? 'var(--pastel-peach-txt)' : 'var(--text-main)';
    }
  }).catch(() => {});
}

function setNavMode(mode) {
  currentNavMode = mode;
  fetch('/navmode?mode=' + mode).then(r => r.json()).then(() => {
    const map = {
      'HYBRID_FUSION': 'mode-hybrid',
      'LANE_FOLLOWING': 'mode-lane',
      'SANOGUZHAN_CLASSIC': 'mode-classic',
      'ARUCO_WAYPOINT': 'mode-aruco',
      'AI_ONNX_PILOT': 'mode-ai'
    };
    Object.keys(map).forEach(k => {
      const el = document.getElementById(map[k]);
      if (el) el.className = 'strategy-btn' + (k === mode ? ' active' : '');
    });
  }).catch(() => {});
}

function toggleRouteMode() {
  fetch('/route?cmd=toggle').then(r => r.json()).then(d => {
    const btn = document.getElementById('btn-route-toggle');
    if (btn) {
      btn.textContent = d.route_mode ? '⏹ Stop Route' : '▶ Linked Route';
      btn.style.background = d.route_mode ? '#F87171' : 'var(--lime-accent)';
      btn.style.color = d.route_mode ? '#FFFFFF' : 'var(--dark-accent)';
    }
  }).catch(() => {});
}

function advanceRoute() {
  fetch('/route?cmd=next').then(r => r.json()).then(d => {
    document.getElementById('route-active-lbl').textContent = 'Link ' + (d.route_index + 1) + ' of ' + d.route_list.length;
  }).catch(() => {});
}

function setRouteList() {
  const val = document.getElementById('route-input-list').value;
  fetch('/route?cmd=set&list=' + encodeURIComponent(val)).then(r => r.json()).then(d => {
    document.getElementById('route-active-lbl').textContent = 'Active Tag: #' + d.active_tag;
  }).catch(() => {});
}

function openMarkerModal() { document.getElementById('marker-modal').style.display = 'flex'; }
function closeMarkerModal() { document.getElementById('marker-modal').style.display = 'none'; }

// ── Manual Driving ──
function cmd(c) { fetch('/cmd?c=' + encodeURIComponent(c)).catch(() => {}); }
let drivePressed = false;          // an on-screen drive button is being held down
function pressCmd(c) { if (!isAuto) { drivePressed = true; cmd(c); } }
function releaseCmd() { drivePressed = false; if (!isAuto) cmd('STOP'); }
function sendStop() {
  if (isAuto) toggleAuto(false);
  cmd('STOP');
}

const FORWARD_KEYS = ['w', 'arrowup'], BACK_KEYS = ['s', 'arrowdown'];
const LEFT_KEYS = ['a', 'arrowleft'], RIGHT_KEYS = ['d', 'arrowright'];
const STOP_KEYS = [' ', 'x'];
const keyElements = {'w':'btn-w','arrowup':'btn-w','s':'btn-s','arrowdown':'btn-s',
                     'a':'btn-a','arrowleft':'btn-a','d':'btn-d','arrowright':'btn-d',' ':'btn-space','x':'btn-space'};
const heldKeys = new Set();        // keys currently down that this page acted on
let lastDrive = 'STOP';            // last drive command sent from the keyboard

// What the held keys mean. Forward + left/right is a gentle arc (a slide), so the car
// keeps moving while it steers. Left/right on their own pivot in place. Reversing has
// no arc, so steering keys change nothing there. Both or neither side = straight.
function driveFor(keys) {
  const any = names => names.some(n => keys.has(n));
  const fwd = any(FORWARD_KEYS), back = any(BACK_KEYS);
  const left = any(LEFT_KEYS), right = any(RIGHT_KEYS);
  const steer = left === right ? 0 : (left ? -1 : 1);
  if (fwd && back) return 'STOP';
  if (fwd) return steer < 0 ? 'SLIDE_LEFT' : (steer > 0 ? 'SLIDE_RIGHT' : 'FORWARD');
  if (back) return 'BACKWARD';
  return steer < 0 ? 'LEFT' : (steer > 0 ? 'RIGHT' : 'STOP');
}

function applyKeyDrive() {
  const next = driveFor(heldKeys);
  if (next !== lastDrive && !isAuto) { lastDrive = next; cmd(next); }
}

function clearHeld() {
  heldKeys.clear();
  document.querySelectorAll('.wasd-btn.active').forEach(el => el.classList.remove('active'));
}

// Anything that can swallow a key-up or a button release (window loses focus, tab hidden,
// mouse let go off the button, touch cancelled) must not leave the car driving.
function releaseAll() {
  const wasDriving = drivePressed || lastDrive !== 'STOP';
  clearHeld();
  drivePressed = false;
  lastDrive = 'STOP';
  if (wasDriving && !isAuto) cmd('STOP');
}
['mouseup', 'touchend', 'touchcancel', 'pointercancel'].forEach(t =>
  document.addEventListener(t, () => { if (drivePressed) releaseAll(); }));
window.addEventListener('blur', releaseAll);
document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); });

// Keys that belong to a control being typed in or adjusted must not drive the car: arrow
// keys move the camera slider, and letters/space go into the route box.
function keyBelongsToControl(e) {
  const t = e.target, tag = t && t.tagName ? t.tagName.toLowerCase() : '';
  if (tag === 'textarea' || tag === 'select' || (t && t.isContentEditable)) return true;
  if (tag === 'input') return t.type === 'range' ? e.key.startsWith('Arrow') : true;
  return false;
}

document.addEventListener('keydown', e => {
  if (keyBelongsToControl(e)) return;
  const k = e.key.toLowerCase();
  const isStop = STOP_KEYS.includes(k);
  const isDrive = [...FORWARD_KEYS, ...BACK_KEYS, ...LEFT_KEYS, ...RIGHT_KEYS].includes(k);
  if (!isStop && !isDrive) return;
  e.preventDefault();
  if (heldKeys.has(k)) return;
  if (isStop) {
    clearHeld();                   // nothing resumes until the drive keys are pressed again
    lastDrive = 'STOP';
    sendStop();
  }
  heldKeys.add(k);
  const elId = keyElements[k];
  if (elId) document.getElementById(elId).classList.add('active');
  if (isDrive) applyKeyDrive();
});

document.addEventListener('keyup', e => {
  const k = e.key.toLowerCase();
  if (!heldKeys.delete(k)) return; // a key this page never acted on, e.g. an arrow used on the slider
  const elId = keyElements[k];
  if (elId) document.getElementById(elId).classList.remove('active');
  applyKeyDrive();
});

// ── Servo Slider ──
let _st = null;
let lastServoInput = 0;            // when the operator last touched the camera slider
function sendServo(val) {
  lastServoInput = Date.now();
  document.getElementById('servo-display').textContent = val + '°';
  document.getElementById('servo-slider').value = val;
  if (_st) clearTimeout(_st);
  _st = setTimeout(() => { cmd('SERVO:' + val); _st = null; }, 80);
}

// ── Fixed side looks (left / right / check both) ──
function lookSide(dir) { fetch('/look?dir=' + dir).catch(() => {}); }

// ── Telemetry Polling Loop ──
function poll() {
  return fetch('/status').then(r => r.json()).then(d => {
    // CPU Temp
    if (d.cpu_temp !== undefined) {
      const tb = document.getElementById('chip-temp');
      if (tb) tb.textContent = 'CPU: ' + d.cpu_temp + '°C';
    }

    // Standby Mode
    if (d.standby !== undefined) updateStandbyUI(d.standby, d.cpu_temp);

    // Serial Chip
    const sb = document.getElementById('chip-serial');
    if (d.serial && d.serial.startsWith('connected')) {
      sb.className = 'chip chip-purple';
      sb.textContent = 'Serial: ' + d.serial.replace('connected:', '');
    } else {
      sb.className = 'chip chip-peach';
      sb.textContent = 'Serial: ' + (d.serial || 'disconnected');
    }

    // AI Chip
    const ab = document.getElementById('chip-ai');
    if (d.ai_loaded) {
      ab.textContent = 'AI: SidewalkPilot ONNX (' + (d.ai_inference_ms || 42) + 'ms)';
    }

    // Servo display: the angle the servo owner is commanding (a look or a held slider angle),
    // else the last angle sent. Leave the slider alone while the operator is using it, or
    // the poll snaps it back to a stale value under their hand.
    const slv = d.side_look || {};
    const sAngle = (slv.angle !== null && slv.angle !== undefined)
      ? slv.angle : (d.servo !== undefined ? d.servo : 90);
    if (Date.now() - lastServoInput > 1500) {
      document.getElementById('servo-display').textContent = sAngle + '°';
      document.getElementById('servo-slider').value = sAngle;
    }

    // Side obstacle warning
    const sl = d.side_look || {};
    const slChip = document.getElementById('side-look-chip');
    if (slChip) {
      if (sl.warning) {
        slChip.className = 'chip chip-peach';
        slChip.textContent = '⚠ ' + sl.warning;
      } else if (sl.advice === 'CLEAR_TO_MOVE') {
        slChip.className = 'chip chip-mint';
        slChip.textContent = 'Sides clear';
      } else {
        slChip.className = 'chip chip-mint';
        slChip.textContent = (sl.mode && sl.mode !== 'idle') ? 'Sides: looking…' : 'Sides: not checked';
      }
    }

    const telem = d.auto_telemetry || {};

    // Lateral Error & Direction
    const laneErr = telem.error_px !== undefined ? telem.error_px : 0.0;
    const laneDir = telem.direction || 'STRAIGHT';
    document.getElementById('val-lateral-err').textContent = (laneErr > 0 ? '+' : '') + laneErr + ' px';
    document.getElementById('val-lane-dir').textContent = 'Dir: ' + laneDir;

    // Fused Steering
    const fs = telem.fused_steer !== undefined ? telem.fused_steer : 0.0;
    const angleDeg = Math.round(fs * 30);
    document.getElementById('val-fused-steer').textContent = (fs > 0 ? '+' : '') + fs.toFixed(2);
    document.getElementById('val-fused-angle').textContent = 'Angle: ' + (angleDeg > 0 ? '+' : '') + angleDeg + '°';

    // Vehicle State
    const curState = telem.state || (d.auto ? 'AUTONOMOUS' : 'IDLE');
    document.getElementById('val-veh-state').textContent = curState;
    document.getElementById('val-veh-sub').textContent = 'Cmd: ' + (telem.command || d.ack || 'STOP');

    // Sector Clearances
    const sc = telem.sector_clearances || {left: 1.0, center: 1.0, right: 1.0};
    document.getElementById('clr-left').textContent = Math.round((sc.left || 1.0) * 100) + '%';
    document.getElementById('clr-center').textContent = Math.round((sc.center || 1.0) * 100) + '%';
    document.getElementById('clr-right').textContent = Math.round((sc.right || 1.0) * 100) + '%';

    // Sonar & Ultrasonic Telemetry Updates (Slider Gauge + Tiles)
    const isStandby = Boolean(d.standby);
    const dist = (isStandby || d.dist === null || d.dist === undefined) ? null : d.dist;
    const sText = document.getElementById('sonar-dist-text');
    const sTop = document.getElementById('top-sonar-dist');
    const sCam = document.getElementById('cam-sonar-val');
    const sTileVal = document.getElementById('val-sonar-dist');
    const sTileSub = document.getElementById('val-sonar-status');
    const sChip = document.getElementById('sonar-status-chip');
    const sTileIcon = document.getElementById('tile-sonar-icon');

    // Ultrasonic Slider Elements
    const sThumb = document.getElementById('sonar-slider-thumb');
    const sFill = document.getElementById('sonar-active-fill');
    const sBubble = document.getElementById('sonar-thumb-bubble');
    const sPin = document.getElementById('sonar-thumb-pin');

    const maxRange = 150.0;

    if (isStandby) {
      if (sText) sText.textContent = 'STANDBY';
      if (sTop) sTop.textContent = 'Standby';
      if (sCam) sCam.textContent = 'Standby';
      if (sTileVal) sTileVal.textContent = 'Standby';
      if (sTileSub) { sTileSub.textContent = 'Sensor Off (Standby)'; sTileSub.style.color = 'var(--text-muted)'; }
      if (sChip) {
        sChip.textContent = 'STANDBY (OFF)';
        sChip.style.background = '#F3F4F6';
        sChip.style.color = '#6B7280';
      }
      if (sBubble) sBubble.textContent = 'STANDBY';
      if (sThumb) {
        sThumb.style.left = '0%';
        sThumb.style.borderColor = '#9CA3AF';
        sThumb.style.boxShadow = 'none';
      }
      if (sPin) sPin.style.background = '#9CA3AF';
      if (sFill) {
        sFill.style.width = '0%';
        sFill.style.background = '#E5E7EB';
      }
    } else {
      const isClear = (dist === null || dist >= 400);
      const measuredDist = isClear ? maxRange : Math.max(0, Math.min(maxRange, dist));
      const sliderPct = Math.round((measuredDist / maxRange) * 100);

      if (sThumb) sThumb.style.left = sliderPct + '%';
      if (sFill) sFill.style.width = sliderPct + '%';

      if (isClear) {
      if (sText) sText.textContent = '—';
      if (sTop) sTop.textContent = '— cm';
      if (sCam) sCam.textContent = '— cm';
      if (sTileVal) sTileVal.textContent = '— cm';
      if (sTileSub) { sTileSub.textContent = 'Path Clear (> 3m)'; sTileSub.style.color = 'var(--text-muted)'; }
      if (sChip) {
        sChip.textContent = 'CLEAR PATH';
        sChip.style.background = '#DCFCE7';
        sChip.style.color = '#16A34A';
      }
      if (sBubble) sBubble.textContent = 'CLEAR (>1.5m)';
      if (sThumb) {
        sThumb.style.borderColor = '#16A34A';
        sThumb.style.boxShadow = '0 2px 8px rgba(0,0,0,0.2), 0 0 10px rgba(22,163,74,0.4)';
      }
      if (sPin) sPin.style.background = '#16A34A';
      if (sFill) sFill.style.background = 'linear-gradient(90deg, #10B981, #D4F973)';
    } else {
      const distStr = dist + ' cm';
      if (sText) sText.textContent = distStr;
      if (sTop) sTop.textContent = distStr;
      if (sCam) sCam.textContent = distStr;
      if (sTileVal) sTileVal.textContent = distStr;
      if (sBubble) sBubble.textContent = distStr;

      if (dist <= 35) {
        if (sTileSub) { sTileSub.textContent = '🚨 Detour Hazard'; sTileSub.style.color = '#DC2626'; }
        if (sChip) { sChip.textContent = '🚨 HAZARD DETOUR'; sChip.style.background = '#FEE2E2'; sChip.style.color = '#DC2626'; }
        if (sTileIcon) sTileIcon.style.background = '#FEE2E2';
        if (sThumb) {
          sThumb.style.borderColor = '#DC2626';
          sThumb.style.boxShadow = '0 2px 8px rgba(0,0,0,0.25), 0 0 14px rgba(220,38,38,0.7)';
        }
        if (sPin) sPin.style.background = '#DC2626';
        if (sFill) sFill.style.background = 'linear-gradient(90deg, #F87171, #DC2626)';
      } else if (dist < 65) {
        if (sTileSub) { sTileSub.textContent = '⚠ Proximity Caution'; sTileSub.style.color = '#D97706'; }
        if (sChip) { sChip.textContent = '⚠ CAUTION'; sChip.style.background = '#FEF3C7'; sChip.style.color = '#D97706'; }
        if (sTileIcon) sTileIcon.style.background = '#FEF3C7';
        if (sThumb) {
          sThumb.style.borderColor = '#D97706';
          sThumb.style.boxShadow = '0 2px 8px rgba(0,0,0,0.2), 0 0 12px rgba(217,119,6,0.6)';
        }
        if (sPin) sPin.style.background = '#D97706';
        if (sFill) sFill.style.background = 'linear-gradient(90deg, #FCD34D, #D97706)';
      } else {
        if (sTileSub) { sTileSub.textContent = 'Path Clear'; sTileSub.style.color = 'var(--text-muted)'; }
        if (sChip) { sChip.textContent = 'CLEAR PATH'; sChip.style.background = '#DCFCE7'; sChip.style.color = '#16A34A'; }
        if (sTileIcon) sTileIcon.style.background = 'var(--pastel-mint)';
        if (sThumb) {
          sThumb.style.borderColor = '#16A34A';
          sThumb.style.boxShadow = '0 2px 8px rgba(0,0,0,0.2), 0 0 10px rgba(22,163,74,0.4)';
        }
        if (sPin) sPin.style.background = '#16A34A';
        if (sFill) sFill.style.background = 'linear-gradient(90deg, #10B981, #D4F973)';
      }
    }
    }

    // ArUco Target Telemetry
    const tags = telem.aruco_tags || [];
    let foundTarget = false;
    tags.forEach(t => {
      if (t.id === selectedTagId) {
        foundTarget = true;
        document.getElementById('val-target-name').textContent = 'Tag #' + t.id + ' (Locked)';
        document.getElementById('val-target-dist').textContent = t.dist_cm + 'cm · ' + (t.bearing_deg > 0 ? '+' : '') + t.bearing_deg + '°';
      }
    });
    if (!foundTarget && tags.length > 0) {
      document.getElementById('val-target-name').textContent = 'Tag #' + tags[0].id + ' (Visible)';
      document.getElementById('val-target-dist').textContent = tags[0].dist_cm + 'cm';
    } else if (!foundTarget) {
      document.getElementById('val-target-name').textContent = 'Tag #' + selectedTagId;
      document.getElementById('val-target-dist').textContent = 'Searching...';
    }

    // Auto Button Sync
    isAuto = d.auto;
    updateAutoUI(d.auto);
  }).catch(() => {});
}

let isPolling = false;
function pollLoop() {
  if (isPolling) return;
  isPolling = true;
  poll().catch(() => {}).finally(() => {
    isPolling = false;
    const nextInterval = isStandbyMode ? 1000 : 120;
    setTimeout(pollLoop, nextInterval);
  });
}
pollLoop();
showView('hud');
</script>
</body>
</html>
"""


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


try:
    import webrtc_stream
    STREAMER = webrtc_stream.WebRTCStreamer(LIVE_FRAMES.get, fps=STREAM_MAX_FPS) if webrtc_stream.AVAILABLE else None
except Exception as _rtc_err:
    print(f"[webrtc] disabled: {_rtc_err}")
    STREAMER = None


class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_POST(self):
        if urllib.parse.urlparse(self.path).path != "/webrtc/offer" or STREAMER is None:
            self._respond(404, "text/plain", b"Not found")
            return
        try:
            length = min(int(self.headers.get("Content-Length", "0")), 65536)
            body = json.loads(self.rfile.read(length))
            view = body.get("view", "hud")
            answer = STREAMER.offer(body["sdp"], body["type"], view if view in VIEW_NAMES else "hud")
            self._respond(200, "application/json", json.dumps(answer).encode())
        except Exception as e:
            print(f"[webrtc] offer failed: {e}")
            self._respond(500, "text/plain", b"WebRTC offer failed")

    def do_GET(self):
        global standby_mode, auto_running, servo_angle, swap_rb
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        params = urllib.parse.parse_qs(parsed.query)

        if path == "/" or path == "/index.html":
            self._respond(200, "text/html; charset=utf-8", HTML.encode("utf-8"))

        elif path == "/stream":
            view_mode = params.get("view", ["hud"])[0]

            # Zero-Latency MJPEG streaming: set TCP_NODELAY to flush packets immediately
            try:
                self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except Exception:
                pass

            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()

            if view_mode not in VIEW_NAMES:
                view_mode = "hud"
            last_sent_ts = 0.0
            last_write = 0.0
            min_gap = 1.0 / STREAM_MAX_FPS
            try:
                while True:
                    VIEW_WANTED[view_mode] = time.time()
                    # Wait for a genuinely new frame of *this* view (dedup keeps browser buffers small)
                    with FRAME_CONDITION:
                        while VIEW_TS[view_mode] == last_sent_ts:
                            VIEW_WANTED[view_mode] = time.time()
                            FRAME_CONDITION.wait(timeout=1.0)
                        if view_mode == "birdseye":
                            frame_data = CAMERA_BIRDSEYE_JPEG
                        elif view_mode == "raw":
                            frame_data = CAMERA_RAW_JPEG
                        else:
                            frame_data = CAMERA_HUD_JPEG
                        last_sent_ts = VIEW_TS[view_mode]

                    gap = time.time() - last_write
                    if gap < min_gap:
                        time.sleep(min_gap - gap)
                    if frame_data:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        self.wfile.write(frame_data)
                        self.wfile.write(b"\r\n")
                        last_write = time.time()
            except Exception:
                pass

        elif path == "/snapshot":
            view_mode = params.get("view", ["hud"])[0]
            if view_mode in VIEW_NAMES:
                VIEW_WANTED[view_mode] = time.time()
                deadline = time.time() + 0.5
                while time.time() - VIEW_TS[view_mode] > 0.5 and time.time() < deadline:
                    time.sleep(0.03)   # wait for the loop to encode a fresh frame
            with CAMERA_LOCK:
                if view_mode == "birdseye":
                    frame_data = CAMERA_BIRDSEYE_JPEG
                elif view_mode == "raw":
                    frame_data = CAMERA_RAW_JPEG
                else:
                    frame_data = CAMERA_HUD_JPEG
            if frame_data:
                self._respond(200, "image/jpeg", frame_data)
            else:
                self._respond(503, "text/plain", b"Camera frame unavailable")

        elif path == "/cmd":
            c = params.get("c", [""])[0]
            with standby_lock:
                is_sb = standby_mode
            if is_sb and c not in ["STANDBY", "RESUME", "PING"]:
                self._respond(200, "application/json", b'{"ok":false,"error":"Vehicle is in Standby Mode"}')
                return
            if c and c.startswith("SERVO:"):
                # The follower owns the servo (control/side_look.py). Sending SERVO here
                # as well made the camera loop snap the camera straight back to 90.
                try:
                    line_follower.side_look.hold(int(c.split(":")[1]), time.time())
                except (IndexError, ValueError):
                    self._respond(200, "application/json", b'{"ok":false,"error":"bad servo angle"}')
                    return
            elif c in ALLOWED_CMDS:
                send_cmd(c)
            elif c:
                self._respond(200, "application/json", b'{"ok":false,"error":"unknown command"}')
                return
            self._respond(200, "application/json", b'{"ok":true}')

        elif path == "/look":
            direction = params.get("dir", ["check"])[0]
            with standby_lock:
                is_sb = standby_mode
            if is_sb:
                self._respond(200, "application/json", b'{"ok":false,"error":"Vehicle is in Standby Mode"}')
            else:
                ok = line_follower.side_look.request(direction, time.time())
                self._respond(200, "application/json", json.dumps({
                    "ok": ok, "side_look": line_follower.side_look.status(time.time()),
                }).encode())

        elif path == "/aruco":
            try:
                target_id = int(params.get("target", [0])[0])
                line_follower.set_aruco_target(target_id)
                self._respond(200, "application/json", json.dumps({"ok": True, "target": target_id}).encode())
            except Exception as e:
                self._respond(400, "application/json", json.dumps({"error": str(e)}).encode())

        elif path == "/arucoclear":
            line_follower.aruco_navigator.clear_target()
            self._respond(200, "application/json", b'{"ok":true}')

        elif path == "/route":
            cmd_type = params.get("cmd", ["status"])[0]
            if cmd_type == "start":
                line_follower.toggle_route_mode(True)
            elif cmd_type == "stop":
                line_follower.toggle_route_mode(False)
            elif cmd_type == "toggle":
                line_follower.toggle_route_mode()
            elif cmd_type == "next":
                line_follower.advance_route_link()
            elif cmd_type == "set":
                r_str = params.get("list", ["0,1,2"])[0]
                try:
                    r_list = [int(x.strip()) for x in r_str.split(",") if x.strip()]
                    if r_list:
                        line_follower.set_route_list(r_list)
                except Exception:
                    pass
            self._respond(200, "application/json", json.dumps({
                "ok": True,
                "route_mode": line_follower.route_mode_enabled,
                "route_list": line_follower.route_list,
                "route_index": line_follower.route_index,
                "active_tag": line_follower.route_list[line_follower.route_index] if line_follower.route_list else None,
            }).encode())

        elif path == "/arucotag":
            tag_id = int(params.get("id", [0])[0])
            dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            marker = cv2.aruco.generateImageMarker(dict_4x4, tag_id, 240, 1)
            bordered = cv2.copyMakeBorder(marker, 35, 35, 35, 35, cv2.BORDER_CONSTANT, value=255)
            _, buf = cv2.imencode(".png", bordered)
            png_bytes = bytes(buf)
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(png_bytes)))
            self.end_headers()
            self.wfile.write(png_bytes)

        elif path == "/download/aruco_markers.pdf":
            # Search possible paths for the generated PDF
            possible_paths = [
                os.path.join(parent_dir, "aruco_markers.pdf"),
                os.path.join(curr_dir, "aruco_markers.pdf"),
                os.path.join(os.path.dirname(parent_dir), "aruco_markers.pdf"),
                "/home/alfin/autonomous_car/aruco_markers.pdf",
            ]
            pdf_bytes = None
            for p in possible_paths:
                if os.path.exists(p):
                    try:
                        with open(p, "rb") as f:
                            pdf_bytes = f.read()
                        break
                    except Exception:
                        pass
            if pdf_bytes:
                self.send_response(200)
                self.send_header("Content-Type", "application/pdf")
                self.send_header("Content-Disposition", 'attachment; filename="aruco_markers.pdf"')
                self.send_header("Content-Length", str(len(pdf_bytes)))
                self.end_headers()
                self.wfile.write(pdf_bytes)
            else:
                self._respond(404, "text/plain", b"PDF not found")

        elif path == "/destination":
            try:
                x = int(params.get("x", [200])[0])
                y = int(params.get("y", [120])[0])
                line_follower.set_destination(x, y)
                self._respond(200, "application/json", json.dumps({
                    "ok": True, "target": [x, y], "dist_m": line_follower.dest_distance_est_m
                }).encode())
            except Exception as e:
                self._respond(400, "application/json", json.dumps({"error": str(e)}).encode())

        elif path == "/cleardest":
            line_follower.clear_destination()
            self._respond(200, "application/json", b'{"ok":true}')

        elif path == "/auto":
            action = params.get("action", ["toggle"])[0]
            with auto_lock:
                if action == "start":
                    auto_running = True
                elif action == "stop":
                    auto_running = False
                else:
                    auto_running = not auto_running
                current_state = auto_running
            if not current_state:
                send_cmd("STOP")
            self._respond(200, "application/json", json.dumps({"ok": True, "auto": current_state}).encode())

        elif path == "/navmode":
            mode_str = params.get("mode", ["HYBRID_FUSION"])[0]
            try:
                mode = NavigationMode(mode_str)
                line_follower.set_navigation_mode(mode)
                self._respond(200, "application/json", json.dumps({"ok": True, "nav_mode": mode.value}).encode())
            except Exception as e:
                self._respond(400, "application/json", json.dumps({"error": str(e)}).encode())

        elif path == "/surveymode":
            mode_str = params.get("mode", ["OFF"])[0]
            try:
                mode = SurveyMode(mode_str)
                line_follower.set_survey_mode(mode)
                self._respond(200, "application/json", json.dumps({"ok": True, "survey_mode": mode.value}).encode())
            except Exception as e:
                self._respond(400, "application/json", json.dumps({"error": str(e)}).encode())

        elif path == "/standby":
            action = params.get("action", ["toggle"])[0]
            with standby_lock:
                if action == "on":
                    new_standby = True
                elif action == "off":
                    new_standby = False
                else:
                    new_standby = not standby_mode
                changed = (new_standby != standby_mode)
                standby_mode = new_standby
                cur_standby = standby_mode

            if changed:
                if cur_standby:
                    with auto_lock:
                        if auto_running:
                            auto_running = False
                    try:
                        line_follower.stop()
                    except Exception:
                        pass
                    # Standby cut off: stop ultrasonic reading & cut off hardware
                    global latest_dist
                    latest_dist = None
                    with ser_lock:
                        if ser and ser.is_open:
                            try:
                                ser.reset_input_buffer()
                            except Exception:
                                pass
                    send_cmd("STANDBY")
                    send_cmd("STOP")
                    standby_wake_event.clear()
                    print("[STANDBY] Activated: ultrasonic reading stopped, all hardware cut off, STANDBY signal sent.")
                else:
                    with ser_lock:
                        if ser and ser.is_open:
                            try:
                                ser.reset_input_buffer()
                            except Exception:
                                pass
                    send_cmd("RESUME")
                    standby_wake_event.set()
                    print("[STANDBY] Resumed: ultrasonic reading active, hardware and perception running.")

            self._respond(200, "application/json", json.dumps({
                "ok": True, "standby": cur_standby, "cpu_temp": get_cpu_temp()
            }).encode())

        elif path == "/camalign":
            act = params.get("swap_rb", ["toggle"])[0]
            if act == "toggle":
                swap_rb = not swap_rb
            elif act == "1":
                swap_rb = True
            elif act == "0":
                swap_rb = False
            self._respond(200, "application/json", json.dumps({"ok": True, "swap_rb": swap_rb}).encode())

        elif path == "/status":
            telem = line_follower.get_telemetry()
            detections = [
                {
                    "class": detection.class_name,
                    "confidence": round(float(detection.confidence), 3),
                    "box": [
                        int(detection.bbox.x1),
                        int(detection.bbox.y1),
                        int(detection.bbox.x2),
                        int(detection.bbox.y2),
                    ],
                }
                for detection in latest_detections
            ]
            data = {
                "serial": serial_status,
                "dist": latest_dist,
                "dist_age_s": None if latest_dist_ts is None else round(time.time() - latest_dist_ts, 2),
                "sonar_simulated": SIM_SONAR,
                "ack": last_ack,
                "servo": servo_angle,
                "servo_acked": servo_acked,
                "side_look": line_follower.side_look.status(time.time()),
                "camera_ok": CAMERA_OK,
                "camera_error": CAMERA_ERROR,
                "auto": auto_running,
                "standby": standby_mode,
                "swap_rb": swap_rb,
                "cpu_temp": get_cpu_temp(),
                "ai_loaded": line_follower.ai_loaded,
                "ai_inference_ms": line_follower.ai_inference_ms,
                "object_detection": {
                    "loaded": bool(object_detector and object_detector.is_initialized),
                    "model": "yolov8n_int8.tflite",
                    "classes": 80,
                    "last_update": object_detection_last_ts,
                    "error": object_detection_error,
                    "detections": detections,
                },
                "auto_telemetry": telem,
            }
            self._respond(200, "application/json", json.dumps(data).encode())

        else:
            self.send_response(404)
            self.end_headers()

    def _respond(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


# ── Entry Point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("[main] Initializing RC Car Dashboard & Perception Subsystems...")
    connect_serial()
    threading.Thread(target=reader_thread, daemon=True).start()
    threading.Thread(target=camera_vision_thread, daemon=True).start()

    time.sleep(1.5)
    server = ThreadedHTTPServer(("0.0.0.0", 5555), DashboardHandler)
    print("[main] Dashboard running at: http://10.91.16.78:5555")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[main] Stopping dashboard server...")
        send_cmd("STOP")
        server.server_close()
