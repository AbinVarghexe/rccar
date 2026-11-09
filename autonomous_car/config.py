"""
Configuration settings for the Autonomous RC Car.
Tailored for Raspberry Pi 3 & Arduino UNO.
"""

import sys
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

@dataclass
class CameraConfig:
    width: int = 640
    height: int = 480
    fps: int = 30
    flip_v: bool = False
    flip_h: bool = False

@dataclass
class PreprocessingConfig:
    blur_kernel_size: Tuple[int, int] = (5, 5)
    gaussian_sigma: float = 0.0
    binary_threshold: int = 160
    max_binary_value: int = 255
    use_adaptive_threshold: bool = False
    adaptive_block_size: int = 11
    adaptive_c: int = 2
    morph_kernel_size: Tuple[int, int] = (3, 3)
    roi_polygon_normalized: List[Tuple[float, float]] = field(
        default_factory=lambda: [(0.0, 0.45), (1.0, 0.45), (1.0, 1.0), (0.0, 1.0)]
    )

@dataclass
class PIDConfig:
    kp: float = 0.45
    ki: float = 0.001
    kd: float = 0.12
    setpoint: float = 0.0
    output_min: float = -1.0
    output_max: float = 1.0
    integral_max: float = 10.0

@dataclass
class SensorConfig:
    # HC-SR04 is wired per USER_MANUAL.md. Keep it active so both autonomous
    # and manual forward commands receive the obstacle safety gate.
    enabled: bool = True
    front_trig_pin: int = 24  # BCM GPIO 24 (Physical Pin 18)
    front_echo_pin: int = 25  # BCM GPIO 25 (Physical Pin 22, via 1k/2k voltage divider)
    rear_trig_pin: Optional[int] = None
    rear_echo_pin: Optional[int] = None
    warning_distance_cm: float = 30.0
    emergency_stop_distance_cm: float = 15.0
    collision_reverse_distance_cm: float = 10.0
    sonar_avoid_distance_cm: float = 8.0
    sensor_timeout_sec: float = 0.03

@dataclass
class ServoConfig:
    pin: int = 12  # Raspberry Pi 3/4 GPIO 12 (Physical Pin 32, Hardware PWM0)
    center_angle: int = 90
    left_angle: int = 150
    right_angle: int = 30
    scan_delay_sec: float = 0.5
    scan_left_angle: int = 150   # Safe left sweep angle to prevent mechanical stall & overheating
    scan_right_angle: int = 30   # Safe right sweep angle to prevent mechanical stall & overheating

@dataclass
class SerialConfig:
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200
    timeout: float = 1.0
    mock_serial: bool = True

@dataclass
class MotorConfig:
    # L298N wiring: physical pins 11/12/15/16.
    # RPi.GPIO uses BCM numbering: GPIO 17/18/22/23.
    in1_pin: int = 17      # Physical Pin 11, left motor input 1
    in2_pin: int = 18      # Physical Pin 12, left motor input 2
    in3_pin: int = 22      # Physical Pin 15, right motor input 1
    in4_pin: int = 23      # Physical Pin 16, right motor input 2
    in2_alt_pin: Optional[int] = None  # Optional legacy mirror; do not wire by default
    # With this controller's polarity semantics, -1 maps logical FORWARD to
    # the manual's FWD inputs: left GPIO17/IN1 and right GPIO22/IN3.
    left_polarity: int = -1
    right_polarity: int = -1

@dataclass
class YOLOConfig:
    model_name: str = "models/yolov8n_int8.tflite"
    confidence_threshold: float = 0.22
    enable_yolo: bool = True
    target_classes: Optional[List[str]] = None  # None = Detect all 80 COCO classes across full frame
    inference_interval_sec: float = 0.05

@dataclass
class WaypointConfig:
    arrival_tolerance_cm: float = 20.0
    nominal_speed_cm_s: float = 25.0
    turn_rate_deg_s: float = 45.0
    cam_height_cm: float = 12.0       # Camera optical center height above floor in cm
    cam_pitch_deg: float = 15.0       # Camera downward pitch angle in degrees
    fov_h_deg: float = 62.2           # Horizontal field of view in degrees
    fov_v_deg: float = 48.8           # Vertical field of view in degrees

@dataclass
class BluetoothConfig:
    enabled: bool = True
    channel: int = 1
    device_name: str = "RCCar-Alfin"

@dataclass
class WiFiConfig:
    hotspot_ssid: str = "RCCar-WiFi"
    hotspot_password: str = "rccar12345"
    hotspot_ip: str = "192.168.4.1"

@dataclass
class DashboardConfig:
    host: str = "0.0.0.0"
    port: int = 5000
    debug: bool = False
    bluetooth: BluetoothConfig = field(default_factory=BluetoothConfig)
    wifi: WiFiConfig = field(default_factory=WiFiConfig)

@dataclass
class AutonomousConfig:
    enabled: bool = False
    mode: str = "local_ai"   # "hybrid", "local_ai", "server_ai"
    server_url: str = "http://127.0.0.1:8000"
    server_ws_url: str = "ws://127.0.0.1:8000/ws"
    cruise_speed: float = 0.52         # Steady continuous forward cruising throttle for 4WD L298N
    avoid_speed: float = 0.48          # Responsive steering throttle when navigating around obstacles
    stop_area_threshold: float = 0.16  # Area ratio threshold to trigger emergency stop
    slow_area_threshold: float = 0.06  # Area ratio threshold to slow down / steer
    lane_width_ratio: float = 0.38     # Width ratio of center corridor
    sonar_avoid_distance_cm: float = 8.0 # Sonar threshold (<8 cm) to avoid blocked path and take better path
    scan_on_block: bool = True         # Auto-pan servo to find clearer path when corridor blocked
    scan_hold_duration_sec: float = 0.40   # Fast & responsive scan pause (0.4s) to eliminate long freezing
    scan_smooth_step_deg: float = 3.5      # Angular step for smooth jitter-free slewing
    scan_smooth_delay_sec: float = 0.01    # Delay per step for snappy, responsive camera panning
    show_ai_overlay: bool = True       # Overlay HUD bounding boxes and decision banner
    roam_without_target: bool = True   # Automatically cruise and roam continuously when auto mode is enabled
    # Export this model once on a faster machine, then copy it to the Pi.
    # INT8 TFLite avoids the PyTorch/Ultralytics footprint on Raspberry Pi 3.
    model_name: str = "models/yolov8n_int8.tflite"
    confidence_threshold: float = 0.22
    target_classes: Optional[List[str]] = None  # None = Detect all 80 COCO objects across full frame
    # Object detection is advisory; keep the control loop responsive by running
    # the expensive Pi CPU inference at a bounded cadence.
    yolo_inference_interval_sec: float = 0.35
    paid_ai_endpoint: str = ""
    paid_ai_model: str = "gpt-4o-mini"
    paid_ai_api_key_env: str = "RCCAR_PAID_AI_API_KEY"
    paid_ai_interval_sec: float = 1.0
    paid_ai_timeout_sec: float = 4.0
    ground_station_timeout_sec: float = 2.0  # Seconds before flagging loss-of-link to central ground station
    enable_lane_assist: bool = True     # Toggle Lane Keeping Assist (LKA) and boundary departure prevention
    enable_traffic_rules: bool = True   # Toggle traffic lights (red light stop) and stop signs compliance
    # Offline region-to-region navigation. Disabled until a marker route is
    # explicitly configured through the dashboard API.
    marker_navigation_enabled: bool = False
    marker_route: List[int] = field(default_factory=list)
    marker_size_cm: float = 8.0
    marker_dictionary: str = "DICT_4X4_50"
    marker_arrival_distance_cm: float = 35.0
    marker_throttle: float = 0.30
    # Vision safety thresholds
    darkness_mean_threshold: float = 14.0  # Mean luminance (0-255) below which scene is too dark/black
    blank_std_threshold: float = 4.0      # Contrast standard deviation below which frame is blank/dead

@dataclass
class BehavioralCloningConfig:
    model_path: str = "models/SidewalkPilot-v3.1b.onnx"
    data_dir: str = "data/tub"
    # ``local_angle`` uses the learned steering and a conservative fixed
    # throttle. ``local_pilot`` also accepts a throttle output when the
    # trained model has two outputs.
    mode: str = "local_angle"
    enabled: bool = False
    inference_interval_sec: float = 0.08
    constant_throttle: float = 0.35
    # Same meaning as AutonomousConfig: stop when the scene is black or flat.
    darkness_mean_threshold: float = 14.0
    blank_std_threshold: float = 4.0
    # Drive telemetry for autonomous runs (see utils/telemetry.py). Off by
    # default so the SD card is not written to unless asked.
    telemetry_enabled: bool = False
    telemetry_dir: str = "data/drives"
    telemetry_frame_every: int = 5     # save a JPEG on every Nth row (0 = none)

@dataclass
class SafetyConfig:
    emergency_stop_distance_cm: float = 15.0
    warning_distance_cm: float = 30.0
    collision_reverse_distance_cm: float = 10.0
    sonar_avoid_distance_cm: float = 8.0
    max_stale_sensor_sec: float = 0.5
    max_camera_age_sec: float = 0.8
    watchdog_timeout_sec: float = 0.6
    max_speed_limit: float = 0.75
    reversal_deadband_sec: float = 0.08
    dry_run: bool = False

@dataclass
class PlannerConfig:
    hysteresis_threshold: float = 0.15
    reassess_duration_sec: float = 0.5
    max_consecutive_turns: int = 4
    turn_timeout_sec: float = 3.0
    reverse_safety_distance_cm: float = 25.0

@dataclass
class DiagnosticsConfig:
    metrics_history_len: int = 100
    log_interval_sec: float = 1.0
    enable_file_logging: bool = True
    log_file_path: str = "autonomous_car.log"

@dataclass
class AppConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    preprocessing: PreprocessingConfig = field(default_factory=PreprocessingConfig)
    pid: PIDConfig = field(default_factory=PIDConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    servo: ServoConfig = field(default_factory=ServoConfig)
    motor: MotorConfig = field(default_factory=MotorConfig)
    serial: SerialConfig = field(default_factory=SerialConfig)
    yolo: YOLOConfig = field(default_factory=YOLOConfig)
    waypoint: WaypointConfig = field(default_factory=WaypointConfig)
    dashboard: DashboardConfig = field(default_factory=DashboardConfig)
    bluetooth: BluetoothConfig = field(default_factory=BluetoothConfig)
    wifi: WiFiConfig = field(default_factory=WiFiConfig)
    autonomous: AutonomousConfig = field(default_factory=AutonomousConfig)
    behavioral_cloning: BehavioralCloningConfig = field(default_factory=BehavioralCloningConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    diagnostics: DiagnosticsConfig = field(default_factory=DiagnosticsConfig)

config = AppConfig()
