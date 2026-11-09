"""DeepPiCar-style behavioral-cloning driver.

The original DeepPiCar workflow has three deliberately separate stages:

1. manually drive while saving camera frames and control labels;
2. train a small DAVE-2/NVIDIA-style model off-board;
3. run the trained model locally and turn its prediction into steering.

This module implements stages one and three for the existing Raspberry Pi
runtime.  TensorFlow/LiteRT are imported lazily so manual driving and data
collection still work on a Pi that has no ML runtime installed.  A pilot
prediction is advisory: the local camera-freshness and ultrasonic safety
checks remain the final authority before a motor command is emitted.
"""

from __future__ import annotations

import csv
import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover - OpenCV is supplied by Raspberry Pi OS
    cv2 = None

try:
    from autonomous_car.ai.classes import ActionEnum, DecisionResult
    from autonomous_car.control.safety_supervisor import MotionCommand, SafetySupervisor
    from autonomous_car.config import BehavioralCloningConfig, SafetyConfig, WaypointConfig
    from autonomous_car.navigation.waypoint import WaypointNavigator
    from autonomous_car.utils.stage_timer import StageTimer
    from autonomous_car.utils.telemetry import TelemetryLogger
except ImportError:  # Supports running from autonomous_car/ directly.
    from ai.classes import ActionEnum, DecisionResult
    from control.safety_supervisor import MotionCommand, SafetySupervisor
    from config import BehavioralCloningConfig, SafetyConfig, WaypointConfig
    from navigation.waypoint import WaypointNavigator
    from utils.stage_timer import StageTimer
    from utils.telemetry import TelemetryLogger


logger = logging.getLogger(__name__)


def _project_root() -> Path:
    return Path(__file__).resolve().parent


def _resolve_path(value: str, default: str) -> Path:
    raw = Path(value or default)
    return raw if raw.is_absolute() else _project_root() / raw


def _finite_clamp(value: Any, low: float, high: float, fallback: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return fallback
    if not math.isfinite(number):
        return fallback
    return max(low, min(high, number))


def frame_blindness(frame: np.ndarray, dark_mean: float, flat_std: float) -> str:
    """Return why a frame is unusable ("" when it is fine).

    Samples every 16th pixel so the check costs well under a millisecond on a
    Raspberry Pi. A black or featureless frame makes the pilot guess a
    steering angle from nothing, so the caller must treat it as a STOP.
    """
    sampled = np.asarray(frame)[::16, ::16]
    if sampled.size == 0:
        return "empty frame"
    gray = sampled.mean(axis=2) if sampled.ndim == 3 else sampled
    mean = float(gray.mean())
    if mean < dark_mean:
        return f"scene too dark (mean {mean:.1f})"
    if float(gray.std()) < flat_std:
        return "image flat (no contrast)"
    return ""


class PilotModel:
    """Lazy adapter for Keras, TFLite, and SidewalkPilot ONNX models."""

    def __init__(self, model_path: str) -> None:
        self.model_path = str(_resolve_path(model_path, "models/SidewalkPilot-v3.1b.onnx"))
        self.backend = "none"
        self.loaded = False
        self.error = ""
        self._model = None
        self._interpreter = None
        self._input = None
        self._output = None
        self._onnx_session = None
        self._onnx_input_name = None
        self._onnx_output_name = None
        self._onnx_input_shape = None
        self._onnx_layout = "NCHW"
        self._load_attempted = False

    def load(self) -> bool:
        if self._load_attempted:
            return self.loaded
        self._load_attempted = True
        if not os.path.isfile(self.model_path):
            self.error = f"Pilot model not found: {self.model_path}"
            return False

        suffix = Path(self.model_path).suffix.lower()
        if suffix == ".onnx":
            return self._load_onnx()
        if suffix in (".tflite", ".lite"):
            return self._load_tflite()
        return self._load_keras()

    def _load_onnx(self) -> bool:
        try:
            import onnxruntime as ort  # type: ignore

            opts = ort.SessionOptions()
            # Leave cores for capture, lane vision and serial control on the Pi 4.
            opts.intra_op_num_threads = 2
            opts.inter_op_num_threads = 1
            opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            self._onnx_session = ort.InferenceSession(
                self.model_path,
                sess_options=opts,
                providers=["CPUExecutionProvider"],
            )
            input_meta = self._onnx_session.get_inputs()[0]
            output_meta = self._onnx_session.get_outputs()[0]
            self._onnx_input_name = input_meta.name
            self._onnx_output_name = output_meta.name
            self._onnx_input_shape = list(input_meta.shape)
            shape = self._onnx_input_shape
            if len(shape) == 4 and shape[1] == 3:
                self._onnx_layout = "NCHW"
            else:
                self._onnx_layout = "NHWC"
            self.backend = "onnx"
            self.loaded = True
            logger.info(
                "[PILOT] Loaded ONNX model %s input=%s output=%s",
                self.model_path,
                shape,
                output_meta.shape,
            )
            return True
        except Exception as exc:
            self.error = f"ONNX model load failed: {exc}"
            logger.warning("[PILOT] %s", self.error)
            return False

    def _load_keras(self) -> bool:
        try:
            import tensorflow as tf  # type: ignore

            self._model = tf.keras.models.load_model(self.model_path, compile=False)
            self.backend = "tensorflow"
            self.loaded = True
            logger.info("[PILOT] Loaded Keras model %s", self.model_path)
            return True
        except Exception as exc:
            self.error = f"TensorFlow model load failed: {exc}"
            logger.warning("[PILOT] %s", self.error)
            return False

    def _load_tflite(self) -> bool:
        interpreter_cls = None
        errors = []
        for module_name in ("ai_edge_litert.interpreter", "tflite_runtime.interpreter"):
            try:
                module = __import__(module_name, fromlist=["Interpreter"])
                interpreter_cls = module.Interpreter
                break
            except Exception as exc:
                errors.append(f"{module_name}: {exc}")
        if interpreter_cls is None:
            self.error = "TFLite runtime unavailable; " + "; ".join(errors)
            return False
        try:
            self._interpreter = interpreter_cls(model_path=self.model_path)
            self._interpreter.allocate_tensors()
            self._input = self._interpreter.get_input_details()[0]
            self._output = self._interpreter.get_output_details()[0]
            self.backend = "tflite"
            self.loaded = True
            logger.info("[PILOT] Loaded TFLite model %s", self.model_path)
            return True
        except Exception as exc:
            self.error = f"TFLite model load failed: {exc}"
            logger.warning("[PILOT] %s", self.error)
            return False

    @property
    def input_shape(self) -> Tuple[int, int]:
        """Return ``(width, height)`` expected by the pilot model."""
        shape = None
        if self._model is not None:
            shape = getattr(self._model, "input_shape", None)
        elif self._input is not None:
            shape = self._input.get("shape")
        elif self._onnx_input_shape is not None:
            shape = self._onnx_input_shape
        try:
            if self._onnx_input_shape is not None and len(self._onnx_input_shape) == 4:
                dims = [int(x) for x in self._onnx_input_shape if isinstance(x, (int, np.integer)) and int(x) > 1]
                if len(dims) >= 3 and self._onnx_layout == "NCHW":
                    return dims[-1], dims[-2]
            dims = [int(x) for x in shape if int(x) > 1]
            if len(dims) >= 3:
                # NHWC is the normal Keras/TFLite layout.
                return max(1, dims[-2]), max(1, dims[-3])
        except Exception:
            pass
        return 200, 66

    def _prepare(self, frame: np.ndarray) -> np.ndarray:
        if cv2 is None:
            raise RuntimeError("OpenCV is required for pilot preprocessing")
        width, height = self.input_shape
        image = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        if self._onnx_session is not None:
            # SidewalkPilot specifies OpenCV BGR and (x / 255 - 0.5) / 0.5.
            image = (image.astype(np.float32) / 255.0 - 0.5) / 0.5
            if self._onnx_layout == "NCHW":
                return np.transpose(image, (2, 0, 1))
            return image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).astype(np.float32)
        return image / 127.5 - 1.0

    @staticmethod
    def _decode_sidewalkpilot(raw: Any) -> Tuple[float, Optional[float]]:
        """Decode SidewalkPilot v3.1's 9-bucket steering head."""
        values = np.asarray(raw, dtype=np.float32).reshape(-1)
        if values.size < 19:
            raise ValueError(f"SidewalkPilot output must contain 19 values, got {values.size}")
        bucket = int(np.argmax(values[:9]))
        fraction = 1.0 / (1.0 + math.exp(-float(values[9 + bucket])))
        # The model contract divides logical 0..180 degrees into nine 20°
        # buckets. Convert its result to this project's -1..1 steering.
        angle_deg = bucket * 20.0 + fraction * 20.0
        steering = (angle_deg - 90.0) / 90.0
        throttle = 1.0 / (1.0 + math.exp(-float(values[18])))
        return _finite_clamp(steering, -1.0, 1.0), _finite_clamp(throttle, 0.0, 1.0)

    @staticmethod
    def _decode_prediction(raw: Any) -> Tuple[float, Optional[float]]:
        if isinstance(raw, (list, tuple)):
            values = [np.asarray(item).reshape(-1) for item in raw]
            values = [float(item[0]) for item in values if item.size]
        else:
            values = np.asarray(raw).reshape(-1).astype(float).tolist()
        if not values:
            raise ValueError("Pilot returned no values")
        steering = values[0]
        throttle = values[1] if len(values) > 1 else None
        # The original DeepPiCar examples commonly represented steering as a
        # servo angle around 90 degrees, while this project labels joystick
        # steering as -1..1. Accept both conventions at the boundary.
        if abs(steering) > 1.0:
            steering = (steering - 90.0) / 90.0 if 0.0 <= steering <= 180.0 else steering / 45.0
        # Models trained with 0..1 steering are normalized here. Models that
        # already return -1..1 pass through unchanged.
        if 0.0 <= steering <= 1.0:
            steering = steering * 2.0 - 1.0
        if throttle is not None and 0.0 <= throttle <= 1.0:
            throttle = throttle * 2.0 - 1.0
        return _finite_clamp(steering, -1.0, 1.0), (
            _finite_clamp(throttle, -1.0, 1.0) if throttle is not None else None
        )

    def predict(self, frame: np.ndarray) -> Tuple[float, Optional[float]]:
        if not self.load():
            raise RuntimeError(self.error or "Pilot model is not loaded")
        batch = np.expand_dims(self._prepare(frame), axis=0)
        if self._onnx_session is not None:
            raw = self._onnx_session.run(
                [self._onnx_output_name],
                {self._onnx_input_name: batch.astype(np.float32)},
            )[0]
            return self._decode_sidewalkpilot(raw)
        if self._model is not None:
            raw = self._model.predict(batch, verbose=0)
            return self._decode_prediction(raw)

        details = self._input
        tensor = batch
        dtype = details["dtype"]
        scale, zero = details.get("quantization", (0.0, 0))
        if np.issubdtype(dtype, np.integer) and scale:
            tensor = np.round(batch / scale + zero).clip(
                np.iinfo(dtype).min, np.iinfo(dtype).max
            ).astype(dtype)
        else:
            tensor = batch.astype(dtype)
        self._interpreter.set_tensor(details["index"], tensor)
        self._interpreter.invoke()
        raw = self._interpreter.get_tensor(self._output["index"])
        scale, zero = self._output.get("quantization", (0.0, 0))
        if scale:
            raw = (raw.astype(np.float32) - zero) * scale
        return self._decode_prediction(raw)


class PilotDatasetRecorder:
    """Stores one JPEG and one label row per manual control sample."""

    def __init__(self, data_dir: str) -> None:
        self.data_dir = _resolve_path(data_dir, "data/tub")
        self._lock = threading.RLock()
        self._active = False
        self._session_dir: Optional[Path] = None
        self._metadata = None
        self._csv = None
        self._count = 0
        self._started_at = 0.0

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def start(self) -> Dict[str, Any]:
        with self._lock:
            if self._active:
                return self.status()
            self.data_dir.mkdir(parents=True, exist_ok=True)
            session_name = time.strftime("session-%Y%m%d-%H%M%S")
            session = self.data_dir / session_name
            suffix = 1
            while session.exists():
                session = self.data_dir / f"{session_name}-{suffix}"
                suffix += 1
            session.mkdir(parents=True)
            self._session_dir = session
            self._metadata = open(session / "metadata.jsonl", "a", encoding="utf-8", newline="")
            self._csv = open(session / "labels.csv", "a", encoding="utf-8", newline="")
            csv.writer(self._csv).writerow(["image", "steering", "throttle", "timestamp"])
            self._count = 0
            self._started_at = time.time()
            self._active = True
            return self.status()

    def add(
        self,
        frame: Optional[np.ndarray],
        steering: float,
        throttle: float,
        extra: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """Store one sample. ``extra`` goes to metadata.jsonl only, so the
        four-column labels.csv that training scripts read is unchanged."""
        if frame is None or cv2 is None:
            return False
        with self._lock:
            if not self._active or self._session_dir is None or self._metadata is None or self._csv is None:
                return False
            image_name = f"frame-{self._count:06d}.jpg"
            path = self._session_dir / image_name
            ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
            if not ok:
                return False
            path.write_bytes(encoded.tobytes())
            timestamp = time.time()
            record = {
                "image": image_name,
                "steering": round(_finite_clamp(steering, -1.0, 1.0), 6),
                "throttle": round(_finite_clamp(throttle, -1.0, 1.0), 6),
                "timestamp": timestamp,
            }
            csv_row = [image_name, record["steering"], record["throttle"], timestamp]
            if extra:
                record.update(extra)
            self._metadata.write(json.dumps(record) + "\n")
            csv.writer(self._csv).writerow(csv_row)
            self._metadata.flush()
            self._csv.flush()
            self._count += 1
            return True

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            if self._metadata:
                self._metadata.close()
            if self._csv:
                self._csv.close()
            self._metadata = None
            self._csv = None
            self._active = False
            return self.status()

    def status(self) -> Dict[str, Any]:
        return {
            "recording": self._active,
            "samples": self._count,
            "session": str(self._session_dir) if self._session_dir else None,
            "data_dir": str(self.data_dir),
            "started_at": self._started_at or None,
        }


class BehavioralCloningDriver:
    """Safety-gated local pilot controller compatible with the dashboard API."""

    def __init__(
        self,
        config: BehavioralCloningConfig,
        motors: Any,
        camera: Any,
        ultrasonic_sensors: Optional[Any] = None,
        safety_config: Optional[SafetyConfig] = None,
    ) -> None:
        self.config = config
        self.motors = motors
        self.camera = camera
        self.ultrasonic_sensors = ultrasonic_sensors
        self.model = PilotModel(config.model_path)
        self.recorder = PilotDatasetRecorder(config.data_dir)
        self.stage_timer = StageTimer()
        self.telemetry = TelemetryLogger(
            _resolve_path(getattr(config, "telemetry_dir", ""), "data/drives"),
            frame_every=getattr(config, "telemetry_frame_every", 5),
        )
        self.safety_supervisor = SafetySupervisor(safety_config or SafetyConfig())
        # Keep the dashboard's multi-point route API available in pilot mode.
        # This is dead-reckoned from wheel commands because this vehicle has
        # no wheel encoders; the route is therefore intended for short,
        # controlled indoor paths, not precision surveying.
        self.waypoint_nav = WaypointNavigator(WaypointConfig())
        self._destination_reached = False
        self.mode = str(getattr(config, "mode", "local_angle"))
        self.is_enabled = bool(getattr(config, "enabled", False))
        self.show_overlay = True
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        self._current_prediction = (0.0, None)
        # Neutral compatibility fields consumed by the shared dashboard's
        # manual safety path. Pilot mode does not run object detection.
        self._current_detections = []
        self._is_blind = False
        self._blind_reason = ""
        self._current_decision = DecisionResult(
            action=ActionEnum.STOP,
            reason="Pilot standby; manual control is active",
            source="behavioral_cloning",
        )
        self._fps = 0.0
        self._latency_ms = 0.0
        self._front_distance_cm = None
        self._front_sensor_data = None
        self._last_prediction_at = 0.0
        self._last_sensor_poll_at = 0.0
        self._sensor_poll_interval_sec = 0.12
        self._last_manual_control = (0.0, 0.0)
        self._last_error = ""
        self._last_ground_station_heartbeat = time.time()
        self._last_loop_at = time.monotonic()
        self._control_generation = 0

    def start(self) -> None:
        if self._running:
            return
        # Validate the configured pilot at service startup while keeping the
        # vehicle disabled.  Inference remains safety-gated by is_enabled,
        # but the dashboard can now report model readiness before a user
        # enables autonomous driving.
        if not self.model.load():
            self._last_error = self.model.error or "Pilot model is not loaded"
        self._running = True
        self._thread = threading.Thread(target=self._driving_loop, daemon=True, name="pilot-driver-loop")
        self._thread.start()

    def stop(self) -> None:
        with self._lock:
            self._running = False
            self.is_enabled = False
            self._control_generation += 1
        self.motors.stop()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None
        self.recorder.stop()
        self.telemetry.stop()

    def set_enabled(self, enabled: bool) -> bool:
        with self._lock:
            self.is_enabled = bool(enabled)
            self._control_generation += 1
            if not self.is_enabled:
                self.motors.stop()
                self._current_decision = DecisionResult(
                    action=ActionEnum.STOP,
                    reason="Pilot disabled; manual control is active",
                    source="behavioral_cloning",
                )
        return self.is_enabled

    def set_mode(self, mode: str) -> str:
        value = str(mode or "local_angle").lower().strip()
        aliases = {"local_ai": "local_angle", "behavioral_cloning": "local_angle", "pilot": "local_pilot"}
        value = aliases.get(value, value)
        if value in {"local_angle", "local_pilot"}:
            with self._lock:
                self.mode = value
        return self.mode

    def set_config_params(self, params: Dict[str, Any]) -> None:
        if "constant_throttle" in params:
            self.config.constant_throttle = _finite_clamp(params["constant_throttle"], 0.0, 0.75, 0.35)
        if "inference_interval_sec" in params:
            self.config.inference_interval_sec = _finite_clamp(params["inference_interval_sec"], 0.03, 1.0, 0.08)
        if "telemetry_enabled" in params:
            self.config.telemetry_enabled = bool(params["telemetry_enabled"])

    def _sensor_sample(self):
        sensors = self.ultrasonic_sensors
        if sensors is None:
            return None
        # Hardware unavailable/disabled means this driver has no sonar gate;
        # it must not convert a desktop fallback into a permanent stop.
        if not getattr(sensors, "_gpio_available", False) and not getattr(sensors, "mock_mode", False):
            return None
        try:
            sample = sensors.read_all()
            self._front_sensor_data = sample.front
            self._front_distance_cm = sample.front.distance_cm
            return sample.front
        except Exception as exc:
            self._last_error = f"Ultrasonic read failed: {exc}"
            # Hand back the last known sample instead of None. None means "no
            # sonar gate", which would turn a failing sensor into a clear
            # path; the old sample goes stale and the supervisor stops the car.
            return self._front_sensor_data

    def _sync_telemetry(self) -> None:
        """Log autonomous runs only: start with the pilot, stop with it."""
        wanted = self.is_enabled and bool(getattr(self.config, "telemetry_enabled", False))
        if wanted and not self.telemetry.active:
            self.telemetry.start()
        elif not wanted and self.telemetry.active:
            self.telemetry.stop()

    def _driving_loop(self) -> None:
        last_tick = time.monotonic()
        while self._running:
            started = time.monotonic()
            self.safety_supervisor.heartbeat()

            # Poll the HC-SR04 even in manual standby so the dashboard and
            # manual motor guards have a current obstacle distance available.
            sensor = self._front_sensor_data
            if started - self._last_sensor_poll_at >= self._sensor_poll_interval_sec:
                with self.stage_timer.stage("sonar"):
                    sensor = self._sensor_sample()
                self._last_sensor_poll_at = started

            self._sync_telemetry()
            if not self.is_enabled:
                self._last_loop_at = started
                time.sleep(0.02)
                continue

            with self.stage_timer.stage("capture"):
                frame = self.camera.get_bgr_frame()
            if frame is None:
                self.motors.stop()
                self._set_decision(ActionEnum.STOP, "No camera frame available", hazard=True)
                time.sleep(0.05)
                continue
            try:
                blind_reason = frame_blindness(
                    frame,
                    float(getattr(self.config, "darkness_mean_threshold", 14.0)),
                    float(getattr(self.config, "blank_std_threshold", 4.0)),
                )
                self._is_blind = bool(blind_reason)
                self._blind_reason = blind_reason
                if started - self._last_prediction_at >= max(0.03, float(self.config.inference_interval_sec)):
                    with self.stage_timer.stage("inference"):
                        self._current_prediction = self.model.predict(frame)
                    self._last_prediction_at = started
                steering, learned_throttle = self._current_prediction
                throttle = learned_throttle if self.mode == "local_pilot" and learned_throttle is not None else float(self.config.constant_throttle)
                throttle = _finite_clamp(throttle, 0.0, 0.75, 0.0)

                # A queued destination takes priority over the model's free-
                # driving steering prediction. The pilot still supplies the
                # camera-based throttle/inference, while the waypoint
                # steering keeps the vehicle aimed at the next point.
                active_destination = self.waypoint_nav.get_active_target()
                if active_destination is not None:
                    angle_error, distance_cm, reached = self.waypoint_nav.compute_steering_to_target()
                    if reached:
                        active_destination = self.waypoint_nav.get_active_target()
                        if active_destination is None:
                            self.is_enabled = False
                            self._destination_reached = True
                            self.motors.stop()
                            self._set_decision(ActionEnum.STOP, "Destination route complete; vehicle stopped safely")
                            continue
                        angle_error, distance_cm, _ = self.waypoint_nav.compute_steering_to_target()
                    steering = _finite_clamp(angle_error / 90.0, -1.0, 1.0)
                    throttle = min(throttle, 0.28 if abs(angle_error) > 45.0 else 0.35)
                proposed = MotionCommand(
                    throttle=throttle,
                    steering=_finite_clamp(steering, -1.0, 1.0),
                    action="FORWARD",
                    source="behavioral_cloning",
                )
                frame_age = self.camera.frame_age_seconds()
                with self.stage_timer.stage("safety"):
                    safe = self.safety_supervisor.validate_command(
                        proposed,
                        ultrasonic=sensor,
                        camera_frame_age_sec=frame_age,
                        is_blind=self._is_blind,
                    )
                with self.stage_timer.stage("motor"):
                    self.motors.drive(safe.safe_throttle, safe.safe_steering)
                self.waypoint_nav.odometry.update_motion(safe.safe_throttle, safe.safe_steering)
                if self.telemetry.active:
                    self.telemetry.log(
                        self._telemetry_row(proposed, safe, sensor, frame_age, blind_reason),
                        frame,
                    )
                action = ActionEnum.FORWARD if safe.safe_throttle > 0.02 else ActionEnum.STOP
                if safe.safe_steering < -0.15:
                    action = ActionEnum.LEFT
                elif safe.safe_steering > 0.15:
                    action = ActionEnum.RIGHT
                self._set_decision(action, safe.reason, hazard=not safe.allowed, throttle=safe.safe_throttle, steering=safe.safe_steering)
            except Exception as exc:
                self._last_error = str(exc)
                self.motors.stop()
                self._set_decision(ActionEnum.STOP, f"Pilot unavailable: {exc}", hazard=True)

            elapsed = max(1e-6, time.monotonic() - last_tick)
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / elapsed) if self._fps else 1.0 / elapsed
            self._latency_ms = (time.monotonic() - started) * 1000.0
            last_tick = time.monotonic()
            self._last_loop_at = last_tick
            time.sleep(0.01)

    def _telemetry_row(
        self,
        proposed: MotionCommand,
        safe: Any,
        sensor: Any,
        frame_age: Optional[float],
        blind_reason: str,
    ) -> Dict[str, Any]:
        """One telemetry row. ``steering``/``throttle`` are the commands that
        reached the motors, matching the tub label names."""
        sonar_age = None
        if sensor is not None and sensor.timestamp > 0:
            sonar_age = round(time.time() - sensor.timestamp, 3)
        return {
            "mode": self.mode,
            "pred_steering": round(self._current_prediction[0], 4),
            "pred_throttle": self._current_prediction[1],
            "proposed_steering": round(proposed.steering, 4),
            "proposed_throttle": round(proposed.throttle, 4),
            "steering": round(safe.safe_steering, 4),
            "throttle": round(safe.safe_throttle, 4),
            "allowed": bool(safe.allowed),
            "reason": safe.reason,
            "sonar_cm": getattr(sensor, "distance_cm", None),
            "sonar_valid": getattr(sensor, "is_valid", None),
            "sonar_age_s": sonar_age,
            "frame_age_s": None if frame_age is None else round(frame_age, 3),
            "blind": blind_reason or None,
            "stage_ms": self.stage_timer.last_ms(),
            "loop_ms": round(self._latency_ms, 1),
        }

    def _set_decision(self, action: ActionEnum, reason: str, hazard: bool = False, throttle: float = 0.0, steering: float = 0.0) -> None:
        with self._lock:
            self._current_decision = DecisionResult(
                action=action,
                reason=reason,
                hazard_detected=hazard,
                suggested_throttle=throttle,
                suggested_steering=steering,
                source="behavioral_cloning",
            )

    # Data collection -------------------------------------------------
    def start_recording(self) -> Dict[str, Any]:
        self.set_enabled(False)
        return self.recorder.start()

    def stop_recording(self) -> Dict[str, Any]:
        return self.recorder.stop()

    def get_recording_status(self) -> Dict[str, Any]:
        return self.recorder.status()

    def record_sample(self, steering: float, throttle: float) -> bool:
        self._last_manual_control = (float(steering), float(throttle))
        sensor = self._front_sensor_data
        extra = {
            "mode": "manual",
            "sonar_cm": getattr(sensor, "distance_cm", None),
            "sonar_valid": getattr(sensor, "is_valid", None),
            "sonar_age_s": (
                round(time.time() - sensor.timestamp, 3)
                if sensor is not None and sensor.timestamp > 0
                else None
            ),
            "frame_age_s": self._rounded_frame_age(),
        }
        return self.recorder.add(self.camera.get_bgr_frame(), steering, throttle, extra)

    def _rounded_frame_age(self) -> Optional[float]:
        try:
            age = self.camera.frame_age_seconds()
        except Exception:
            return None
        return None if age is None else round(float(age), 3)

    record_control = record_sample

    # Dashboard compatibility and telemetry -------------------------
    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            ultrasonic_status = {
                "enabled": False,
                "gpio_available": False,
                "distance_cm": self._front_distance_cm,
                "valid": bool(self._front_sensor_data and self._front_sensor_data.is_valid),
                "state": getattr(self._front_sensor_data, "status", "DISABLED"),
            }
            if self.ultrasonic_sensors is not None:
                ultrasonic_status.update({
                    "enabled": bool(self.ultrasonic_sensors.is_enabled()),
                    "gpio_available": bool(getattr(self.ultrasonic_sensors, "_gpio_available", False)),
                })
            return {
                "autonomous_enabled": self.is_enabled,
                "mode": self.mode,
                "action": self._current_decision.action.value,
                "reason": self._current_decision.reason,
                "hazard_detected": self._current_decision.hazard_detected,
                "fps": round(self._fps, 1),
                "latency_ms": round(self._latency_ms, 1),
                "stage_latency_ms": self.stage_timer.snapshot(),
                "telemetry": self.telemetry.status(),
                "camera_blind": self._blind_reason or None,
                "pilot": {
                    "path": self.model.model_path,
                    "loaded": self.model.loaded,
                    "backend": self.model.backend,
                    "error": self.model.error or self._last_error,
                    "steering": round(self._current_prediction[0], 4),
                    "throttle": self._current_prediction[1],
                },
                "recording": self.recorder.status(),
                "front_distance_cm": self._front_distance_cm,
                "ultrasonic": ultrasonic_status,
                "destinations": self.get_destinations(),
                "lane_assist": {"enabled": False, "supported": False},
                "traffic_rules": {"enabled": False, "supported": False},
                "server_ai": {"connected": False, "supported": False},
            }

    def annotate_frame(self, frame: np.ndarray) -> np.ndarray:
        if cv2 is None or frame is None:
            return frame
        out = frame.copy()
        steering, learned_throttle = self._current_prediction
        lines = [
            f"PILOT: {self.mode.upper()}  {'ON' if self.is_enabled else 'OFF'}",
            f"STEER {steering:+.2f}  THROTTLE {(learned_throttle if learned_throttle is not None else self.config.constant_throttle):.2f}",
            self._current_decision.reason[:72],
        ]
        for index, line in enumerate(lines):
            cv2.putText(out, line, (12, 24 + index * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 180), 1, cv2.LINE_AA)
        return out

    def record_ground_station_heartbeat(self) -> None:
        self._last_ground_station_heartbeat = time.time()

    def get_ground_station_status(self) -> Dict[str, Any]:
        return {"connected": True, "last_seen": self._last_ground_station_heartbeat}

    def _execute_path_finding_scan(self) -> None:
        self.motors.stop()
        self._set_decision(ActionEnum.STOP, "Path scan is not part of pilot mode")

    def toggle_lane_assist(self, enabled: Optional[bool] = None) -> bool:
        return False

    def toggle_traffic_rules(self, enabled: Optional[bool] = None) -> bool:
        return False

    def clear_target_pin(self) -> None:
        return None

    @staticmethod
    def _waypoint_dict(waypoint: Any) -> Dict[str, Any]:
        return {
            "id": waypoint.id,
            "x": waypoint.x,
            "y": waypoint.y,
            "label": waypoint.label,
            "reached": waypoint.reached,
        }

    def add_destination(self, x: float, y: float, label: str = "") -> Dict[str, Any]:
        self._destination_reached = False
        waypoint = self.waypoint_nav.add_waypoint(float(x), float(y), label)
        return self._waypoint_dict(waypoint)

    def clear_destinations(self) -> None:
        self._destination_reached = False
        self.motors.stop()
        self.waypoint_nav.clear_waypoints()

    def get_destinations(self) -> Dict[str, Any]:
        active = self.waypoint_nav.get_active_target()
        return {
            "pose": self.waypoint_nav.odometry.pose.__dict__.copy(),
            "active": self._waypoint_dict(active) if active else None,
            "waypoints": [self._waypoint_dict(item) for item in self.waypoint_nav.waypoints],
        }

    def reset_pose(self, x: float = 0.0, y: float = 0.0, heading_deg: float = 90.0) -> None:
        self.waypoint_nav.odometry.reset(float(x), float(y), float(heading_deg))

    def clear_marker_route(self) -> None:
        return None

