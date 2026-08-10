"""
Arduino Serial Communication module using PySerial.
Supports automatic port discovery (/dev/ttyACM0, /dev/ttyACM1, /dev/ttyUSB0),
motion commands (FORWARD, BACKWARD, LEFT, RIGHT, STOP),
and Camera Servo commands (SERVO:angle).
"""

import os
import sys
import time
import threading
from enum import Enum
from typing import Optional, Tuple
import serial
import serial.tools.list_ports

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
root_dir = os.path.dirname(parent_dir)
for p in [curr_dir, parent_dir, root_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

try:
    from config import SerialConfig
    from utils.logger import car_logger
except ImportError:
    from autonomous_car.config import SerialConfig
    from autonomous_car.utils.logger import car_logger

class VehicleCommand(str, Enum):
    FORWARD = "FORWARD"
    BACKWARD = "BACKWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    STOP = "STOP"

class SerialCommunicator:
    def __init__(self, config: SerialConfig) -> None:
        self.config: SerialConfig = config
        self._serial: Optional[serial.Serial] = None
        self._connected: bool = False
        self._lock = threading.Lock()
        self._last_sent_command: Optional[VehicleCommand] = None

        self._front_distance_cm: float = 120.0
        self._rear_distance_cm: float = 150.0
        self._current_servo_angle: int = 90
        self._reader_thread: Optional[threading.Thread] = None
        self._stopped: bool = False

        if not self.config.mock_serial:
            if self._connect():
                self.start_reader()

    def _auto_detect_port(self) -> Optional[str]:
        """Auto-detects available Arduino/USB serial port."""
        ports = serial.tools.list_ports.comports()
        for p in ports:
            if "ACM" in p.device or "USB" in p.device:
                return p.device
        return None

    def _connect(self) -> bool:
        with self._lock:
            # 1. Try configured port first
            target_ports = [self.config.port]
            auto_port = self._auto_detect_port()
            if auto_port and auto_port not in target_ports:
                target_ports.append(auto_port)

            # Standard fallback ports
            for p in ["/dev/ttyACM1", "/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
                if p not in target_ports:
                    target_ports.append(p)

            for port in target_ports:
                try:
                    car_logger.info(f"Attempting to open Serial port {port} at {self.config.baudrate} baud...")
                    self._serial = serial.Serial(
                        port=port,
                        baudrate=self.config.baudrate,
                        timeout=self.config.timeout
                    )
                    time.sleep(2.0)
                    self._connected = True
                    self.config.port = port
                    car_logger.info(f"Successfully connected to Arduino on {port}")
                    return True
                except Exception as e:
                    car_logger.debug(f"Serial port {port} failed: {e}")

            self._connected = False
            self._serial = None
            car_logger.warning("Could not open any Arduino serial ports. Operating in Mock Serial mode.")
            return False

    def start_reader(self) -> None:
        self._stopped = False
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True, name="SerialReaderThread")
        self._reader_thread.start()

    def _read_loop(self) -> None:
        while not self._stopped and self._connected and self._serial and self._serial.is_open:
            try:
                line = self._serial.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("DIST:"):
                    parts = line.replace("DIST:", "").split(",")
                    if len(parts) >= 2:
                        f_dist = float(parts[0])
                        r_dist = float(parts[1])
                        with self._lock:
                            self._front_distance_cm = f_dist
                            self._rear_distance_cm = r_dist
            except Exception:
                pass
            time.sleep(0.02)

    def get_ultrasonic_distances(self) -> Tuple[float, float]:
        with self._lock:
            return (self._front_distance_cm, self._rear_distance_cm)

    def send_servo_angle(self, angle: int) -> bool:
        angle = max(0, min(180, angle))
        command_str = f"SERVO:{angle}\n"
        
        with self._lock:
            self._current_servo_angle = angle
            if self._connected and self._serial is not None and self._serial.is_open:
                try:
                    self._serial.write(command_str.encode("utf-8"))
                    self._serial.flush()
                    car_logger.info(f"[SERIAL OUT -> CAMERA SERVO]: {angle} degrees")
                    return True
                except Exception as e:
                    car_logger.error(f"Failed to write servo command: {e}")
                    return False
            else:
                car_logger.info(f"[MOCK SERIAL OUT -> SERVO]: {angle} degrees")
                return True

    def send_command(self, command: VehicleCommand) -> bool:
        command_str = f"{command.value}\n"
        
        with self._lock:
            self._last_sent_command = command

            if self._connected and self._serial is not None and self._serial.is_open:
                try:
                    self._serial.write(command_str.encode("utf-8"))
                    self._serial.flush()
                    car_logger.info(f"[SERIAL OUT -> ARDUINO]: {command.value}")
                    return True
                except Exception as e:
                    car_logger.error(f"Failed to write to serial port: {e}")
                    self._connected = False
                    return False
            else:
                car_logger.info(f"[MOCK SERIAL OUT]: {command.value}")
                return True

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def last_command(self) -> Optional[VehicleCommand]:
        return self._last_sent_command

    @property
    def current_servo_angle(self) -> int:
        return self._current_servo_angle

    def close(self) -> None:
        self._stopped = True
        with self._lock:
            if self._serial is not None and self._serial.is_open:
                try:
                    self._serial.write(f"{VehicleCommand.STOP.value}\n".encode("utf-8"))
                    self._serial.close()
                    car_logger.info("Serial connection closed gracefully.")
                except Exception as e:
                    car_logger.error(f"Error closing serial port: {e}")
            self._connected = False
