"""
Bluetooth RFCOMM Server for Raspberry Pi 3 RC Car.
Allows zero-latency direct wireless control from smartphones, tablets,
and PCs without requiring a Wi-Fi router or network connection.

Features:
  - RFCOMM Serial Channel 1 (SPP - Serial Port Profile)
  - Automatic bluetoothctl discoverable & pairable configuration
  - Dual control protocol:
      1. Single-character & discrete words (W, A, S, D, F, B, L, R, STOP)
      2. High-precision vector format: JOY:<x>,<y> (e.g. JOY:0.5,-0.8)
      3. Camera pan servo format: SERVO:<angle> (e.g. SERVO:90)
  - Real-time telemetry reporting back over Bluetooth
  - Automatic failsafe: stops motors instantly on Bluetooth disconnect
"""

import os
import sys
import time
import socket
import logging
import threading
import subprocess
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)

# Check native Linux Bluetooth socket support
_BLUETOOTH_AVAILABLE = hasattr(socket, "AF_BLUETOOTH") and hasattr(socket, "BTPROTO_RFCOMM")


class BluetoothServer:
    def __init__(
        self,
        motor_controller=None,
        servo_controller=None,
        channel: int = 1,
        device_name: str = "RCCar-Alfin",
    ) -> None:
        self.motors = motor_controller
        self.servo = servo_controller
        self.channel = channel
        self.device_name = device_name

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._server_sock: Optional[socket.socket] = None
        self._client_sock: Optional[socket.socket] = None
        self._client_addr = None
        self._lock = threading.Lock()
        self._last_cmd_time = 0.0

    @property
    def is_available(self) -> bool:
        return _BLUETOOTH_AVAILABLE

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._client_sock is not None

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            connected = self._client_sock is not None
            client = str(self._client_addr[0]) if self._client_addr else None
        return {
            "available": _BLUETOOTH_AVAILABLE,
            "running": self._running,
            "connected": connected,
            "client": client,
            "channel": self.channel,
            "device_name": self.device_name,
        }

    def start(self) -> "BluetoothServer":
        if self._running:
            return self

        if not _BLUETOOTH_AVAILABLE:
            print("[BLUETOOTH] AF_BLUETOOTH not available on this platform (Simulated mode).")
            self._running = True
            return self

        self._running = True
        self._init_bluetooth_adapter()
        self._thread = threading.Thread(
            target=self._run_server, daemon=True, name="bt-rfcomm-server"
        )
        self._thread.start()
        print(f"[BLUETOOTH READY] RFCOMM Server listening on channel {self.channel} ({self.device_name})")
        return self

    def stop(self) -> None:
        self._running = False
        with self._lock:
            if self._client_sock:
                try:
                    self._client_sock.close()
                except Exception:
                    pass
                self._client_sock = None
            if self._server_sock:
                try:
                    self._server_sock.close()
                except Exception:
                    pass
                self._server_sock = None
        if self.motors:
            try:
                self.motors.stop()
            except Exception:
                pass

    def _init_bluetooth_adapter(self) -> None:
        """Configure Raspberry Pi Bluetooth adapter to be visible and pairable."""
        try:
            # Set adapter name
            subprocess.run(
                ["bluetoothctl", "system-alias", self.device_name],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
            )
            # Make discoverable & pairable
            subprocess.run(
                ["bluetoothctl", "discoverable", "on"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
            )
            subprocess.run(
                ["bluetoothctl", "pairable", "on"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2
            )
        except Exception as e:
            logger.debug(f"Bluetooth adapter configuration note: {e}")

    def _run_server(self) -> None:
        while self._running:
            try:
                self._server_sock = socket.socket(
                    socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM
                )
                self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self._server_sock.bind(("", self.channel))
                self._server_sock.listen(1)
                self._server_sock.settimeout(2.0)

                while self._running:
                    try:
                        client_sock, client_addr = self._server_sock.accept()
                    except socket.timeout:
                        continue
                    except Exception:
                        break

                    print(f"[BLUETOOTH] Client connected from: {client_addr}")
                    with self._lock:
                        self._client_sock = client_sock
                        self._client_addr = client_addr

                    self._handle_client(client_sock)

                    # Disconnected
                    print(f"[BLUETOOTH] Client disconnected: {client_addr}")
                    with self._lock:
                        self._client_sock = None
                        self._client_addr = None
                    if self.motors:
                        self.motors.stop()

            except Exception as e:
                logger.error(f"[BLUETOOTH] Server loop error: {e}")
                time.sleep(2.0)
            finally:
                if self._server_sock:
                    try:
                        self._server_sock.close()
                    except Exception:
                        pass
                    self._server_sock = None

    def _handle_client(self, client_sock: socket.socket) -> None:
        client_sock.settimeout(0.5)
        # Send initial welcome banner
        try:
            welcome = f"OK:RC_CAR_READY (BT 4.1 RFCOMM CH{self.channel})\r\n"
            client_sock.send(welcome.encode("utf-8"))
        except Exception:
            return

        buf = ""
        while self._running:
            try:
                data = client_sock.recv(256)
                if not data:
                    break
                buf += data.decode("utf-8", errors="ignore")
                while "\n" in buf or "\r" in buf:
                    # Extract line
                    line, _, remainder = buf.partition("\n")
                    if not line and "\r" in remainder:
                        line, _, remainder = remainder.partition("\r")
                    buf = remainder
                    cmd = line.strip().upper()
                    if cmd:
                        self._process_command(cmd, client_sock)
            except socket.timeout:
                continue
            except Exception:
                break

    def _process_command(self, cmd: str, client_sock: socket.socket) -> None:
        """Process incoming Bluetooth command."""
        self._last_cmd_time = time.time()
        try:
            # 1. Discrete Motor Commands
            if cmd in ("W", "FORWARD", "F"):
                if self.motors: self.motors.forward()
                self._send_reply(client_sock, "ACK:FORWARD\r\n")

            elif cmd in ("S", "BACKWARD", "B", "REVERSE"):
                if self.motors: self.motors.backward()
                self._send_reply(client_sock, "ACK:REVERSE\r\n")

            elif cmd in ("A", "LEFT", "L"):
                if self.motors: self.motors.left()
                self._send_reply(client_sock, "ACK:LEFT\r\n")

            elif cmd in ("D", "RIGHT", "R"):
                if self.motors: self.motors.right()
                self._send_reply(client_sock, "ACK:RIGHT\r\n")

            elif cmd in ("STOP", "X", "STP", " "):
                if self.motors: self.motors.stop()
                self._send_reply(client_sock, "ACK:STOP\r\n")

            # 2. Vector Joystick: JOY:x,y (e.g. JOY:0.5,-0.8)
            elif cmd.startswith("JOY:"):
                parts = cmd[4:].split(",")
                if len(parts) == 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    if self.motors:
                        self.motors.drive(throttle=y, steering=x)
                    self._send_reply(client_sock, f"ACK:JOY:{x:.2f},{y:.2f}\r\n")

            # 3. Servo Pan: SERVO:angle (e.g. SERVO:90)
            elif cmd.startswith("SERVO:"):
                angle = float(cmd[6:])
                if self.servo:
                    self.servo.set_angle(angle)
                self._send_reply(client_sock, f"ACK:SERVO:{angle:.1f}\r\n")

            # 4. Status / Ping
            elif cmd == "PING":
                self._send_reply(client_sock, "PONG\r\n")

            elif cmd in ("STATUS", "TELEM"):
                msg = f"TELEM:CONNECTED\r\n"
                self._send_reply(client_sock, msg)

        except Exception as e:
            logger.error(f"Error executing Bluetooth command '{cmd}': {e}")

    def _send_reply(self, client_sock: socket.socket, msg: str) -> None:
        try:
            client_sock.send(msg.encode("utf-8"))
        except Exception:
            pass
