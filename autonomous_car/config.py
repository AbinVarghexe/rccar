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
    pin: int = 18  # Raspberry Pi 3 GPIO 18 (Header Pin 12)
    center_angle: int = 90
    left_angle: int = 150
    right_angle: int = 30
    scan_delay_sec: float = 0.5

@dataclass
class SerialConfig:
    port: str = "/dev/ttyACM0" if sys.platform.startswith("linux") else "COM3"
    baudrate: int = 115200
    timeout: float = 1.0
    mock_serial: bool = False

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
class DashboardConfig:
    host: str = "0.0.0.0"
    port: int = 5000
    debug: bool = False

@dataclass
class AppConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    preprocessing: PreprocessingConfig = field(default_factory=PreprocessingConfig)
    pid: PIDConfig = field(default_factory=PIDConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    servo: ServoConfig = field(default_factory=ServoConfig)
    serial: SerialConfig = field(default_factory=SerialConfig)
    yolo: YOLOConfig = field(default_factory=YOLOConfig)
    waypoint: WaypointConfig = field(default_factory=WaypointConfig)
    dashboard: DashboardConfig = field(default_factory=DashboardConfig)

config = AppConfig()
