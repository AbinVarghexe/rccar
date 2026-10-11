"""
Configuration settings for the Autonomous RC Car.
Tailored for Raspberry Pi 3 & Arduino UNO.
"""

import sys
from dataclasses import dataclass, field
from typing import Tuple, List

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
    front_trig_pin: int = 23
    front_echo_pin: int = 24
    rear_trig_pin: int = 17
    rear_echo_pin: int = 27
    warning_distance_cm: float = 30.0
    emergency_stop_distance_cm: float = 15.0
    sensor_timeout_sec: float = 0.04

@dataclass
class ServoConfig:
    pin: int = 12  # Raspberry Pi 3 GPIO 12 (Physical Pin 32, Hardware PWM0)
    center_angle: int = 90
    left_angle: int = 150
    right_angle: int = 30
    scan_delay_sec: float = 0.5

@dataclass
class SerialConfig:
    port: str = "/dev/ttyACM0"
    baudrate: int = 115200
    timeout: float = 1.0
    mock_serial: bool = True

@dataclass
class MotorConfig:
    in1_pin: int = 17      # Physical Pin 11
    in2_pin: int = 27      # Physical Pin 13
    in3_pin: int = 22      # Physical Pin 15
    in4_pin: int = 23      # Physical Pin 16
    in2_alt_pin: int = 18  # Physical Pin 12 (Dual-drive fallback)
    left_polarity: int = 1
    right_polarity: int = 1

@dataclass
class YOLOConfig:
    model_name: str = "yolov8n.pt"
    confidence_threshold: float = 0.45
    enable_yolo: bool = False
    target_classes: List[str] = field(
        default_factory=lambda: [
            "person", "bottle", "chair", "car", "stop sign", "traffic cone"
        ]
    )
    inference_interval_sec: float = 0.5

@dataclass
class WaypointConfig:
    arrival_tolerance_cm: float = 15.0
    nominal_speed_cm_s: float = 25.0
    turn_rate_deg_s: float = 45.0

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
    mode: str = "hybrid"   # "hybrid", "local_ai", "server_ai"
    server_url: str = "http://127.0.0.1:8000"
    server_ws_url: str = "ws://127.0.0.1:8000/ws"
    cruise_speed: float = 0.55         # Default forward cruising throttle (0.0 - 1.0)
    avoid_speed: float = 0.65          # Throttle when turning for obstacle avoidance
    stop_area_threshold: float = 0.12  # Area ratio threshold to trigger emergency stop
    slow_area_threshold: float = 0.05  # Area ratio threshold to slow down / steer
    lane_width_ratio: float = 0.42     # Width ratio of center corridor
    scan_on_block: bool = True         # Auto-pan servo to find clearer path when corridor blocked
    show_ai_overlay: bool = True       # Overlay HUD bounding boxes and decision banner
    model_name: str = "yolov8n.pt"
    confidence_threshold: float = 0.35
    target_classes: List[str] = field(
        default_factory=lambda: [
            "bus", "car", "truck", "person", "bicycle", "motorcycle",
            "stop sign", "traffic cone", "chair", "bottle", "dog", "cat"
        ]
    )

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

config = AppConfig()


