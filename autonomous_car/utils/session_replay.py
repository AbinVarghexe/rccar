"""
Offline replay of recorded driving sessions.

Works on both log formats the car produces:

* ``data/tub/session-*``  -> ``metadata.jsonl`` (manual driving, human labels)
* ``data/drives/drive-*`` -> ``telemetry.jsonl`` (autonomous runs)

Two independent checks, both runnable on a PC with no car attached:

1. ``replay_model``  - run a pilot model over the saved frames and measure how
   well it agrees with what was recorded (a new model vs the old one, or vs
   the human driver).
2. ``replay_safety`` - push the logged commands, sonar readings and frame ages
   through ``SafetySupervisor`` with candidate thresholds and count the stops,
   so thresholds can be tuned without driving.

Usage (from the autonomous_car directory)::

    python -m utils.session_replay data/drives/drive-20261029-101500
    python -m utils.session_replay data/tub/session-... --model models/new.onnx
    python -m utils.session_replay data/drives/drive-... --stop-cm 12 --no-model
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import re
import sys
import time
from collections import Counter
from dataclasses import replace
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

if __package__ in (None, ""):  # running as a script from autonomous_car/
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

try:
    from autonomous_car.config import SafetyConfig
    from autonomous_car.control.safety_supervisor import MotionCommand, SafetySupervisor
    from autonomous_car.sensors.ultrasonic import SingleSensorData
except ImportError:
    from config import SafetyConfig
    from control.safety_supervisor import MotionCommand, SafetySupervisor
    from sensors.ultrasonic import SingleSensorData

logger = logging.getLogger(__name__)

LOG_FILES = ("telemetry.jsonl", "metadata.jsonl")
STEERING_DEADZONE = 0.15  # same threshold the supervisor uses for TURN_*


def load_rows(session_dir: Path) -> Tuple[List[Dict[str, Any]], int]:
    """Read a session log. Returns ``(rows, skipped_malformed_lines)``."""
    session_dir = Path(session_dir)
    log_path = next((session_dir / n for n in LOG_FILES if (session_dir / n).is_file()), None)
    if log_path is None:
        raise FileNotFoundError(f"No {' or '.join(LOG_FILES)} in {session_dir}")
    rows: List[Dict[str, Any]] = []
    skipped = 0
    with open(log_path, "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                skipped += 1  # a power cut can leave a truncated last line
                continue
            if isinstance(row, dict):
                rows.append(row)
            else:
                skipped += 1
    return rows, skipped


def _finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def reference_steering(row: Dict[str, Any]) -> Optional[float]:
    """What to compare a replayed model against: the logged model output for
    autonomous runs, otherwise the human label."""
    value = _finite(row.get("pred_steering"))
    return value if value is not None else _finite(row.get("steering"))


def agreement_stats(predicted: Sequence[float], reference: Sequence[float]) -> Dict[str, Any]:
    """Error metrics between two steering series (both in -1..1)."""
    pred = np.asarray(predicted, dtype=float)
    ref = np.asarray(reference, dtype=float)
    if pred.size == 0 or pred.size != ref.size:
        return {"n": 0}
    diff = pred - ref
    corr: Optional[float] = None
    if pred.size > 1 and pred.std() > 1e-9 and ref.std() > 1e-9:
        corr = round(float(np.corrcoef(pred, ref)[0, 1]), 4)
    turning = np.abs(ref) > STEERING_DEADZONE
    sign_agreement: Optional[float] = None
    if turning.any():
        sign_agreement = round(float(np.mean(np.sign(pred[turning]) == np.sign(ref[turning]))), 4)
    return {
        "n": int(pred.size),
        "mae": round(float(np.mean(np.abs(diff))), 4),
        "rmse": round(float(np.sqrt(np.mean(diff ** 2))), 4),
        "bias": round(float(np.mean(diff)), 4),
        "corr": corr,
        "turn_sign_agreement": sign_agreement,
    }


def replay_model(
    rows: Sequence[Dict[str, Any]],
    session_dir: Path,
    model: Any,
    limit: int = 0,
) -> Dict[str, Any]:
    """Run ``model.predict(frame)`` over every row that has a saved image."""
    import cv2  # local import: the safety replay must work without OpenCV

    predicted: List[float] = []
    reference: List[float] = []
    infer_ms: List[float] = []
    unreadable = 0
    for row in rows:
        image_name = row.get("image")
        ref = reference_steering(row)
        if not image_name or ref is None:
            continue
        if limit and len(predicted) >= limit:
            break
        frame = cv2.imread(str(Path(session_dir) / image_name))
        if frame is None:
            unreadable += 1
            continue
        started = time.perf_counter()
        steering, _ = model.predict(frame)
        infer_ms.append((time.perf_counter() - started) * 1000.0)
        predicted.append(float(steering))
        reference.append(ref)
    stats = agreement_stats(predicted, reference)
    stats["unreadable_images"] = unreadable
    if infer_ms:
        ordered = sorted(infer_ms)
        stats["inference_ms_avg"] = round(sum(infer_ms) / len(infer_ms), 2)
        stats["inference_ms_p95"] = round(ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)], 2)
    return stats


def _normalise_reason(reason: str) -> str:
    return re.sub(r"[-+]?\d+(\.\d+)?", "#", reason)


def _row_command(row: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    """(throttle, steering) the planner asked for before the safety gate."""
    throttle = _finite(row.get("proposed_throttle", row.get("throttle")))
    steering = _finite(row.get("proposed_steering", row.get("steering")))
    if throttle is None or steering is None:
        return None
    return throttle, steering


def replay_safety(rows: Sequence[Dict[str, Any]], config: SafetyConfig) -> Dict[str, Any]:
    """Re-judge every logged command under ``config`` and count the stops."""
    supervisor = SafetySupervisor(config)
    # The supervisor logs a warning per stop; a long replay would flood stderr.
    supervisor_logger = logging.getLogger(SafetySupervisor.__module__)
    previous_level = supervisor_logger.level
    supervisor_logger.setLevel(logging.ERROR)
    reasons: Counter = Counter()
    judged = stops = moving = 0
    try:
        for row in rows:
            command = _row_command(row)
            if command is None:
                continue
            throttle, steering = command
            now = _finite(row.get("timestamp")) or time.time()
            sensor = _sensor_from_row(row, now)
            supervisor.heartbeat()  # replay runs faster than real time
            result = supervisor.validate_command(
                MotionCommand(throttle=throttle, steering=steering, source="replay"),
                ultrasonic=sensor,
                camera_frame_age_sec=_finite(row.get("frame_age_s")),
                is_blind=bool(row.get("blind")),
                now=now,
            )
            judged += 1
            if throttle > 0.05:
                moving += 1
                if result.action == "STOP":
                    stops += 1
                    reasons[_normalise_reason(result.reason)] += 1
    finally:
        supervisor_logger.setLevel(previous_level)
    return {
        "frames_judged": judged,
        "frames_with_forward_request": moving,
        "forward_requests_stopped": stops,
        "stop_rate": round(stops / moving, 4) if moving else None,
        "stop_reasons": dict(reasons.most_common()),
    }


def _sensor_from_row(row: Dict[str, Any], now: float) -> Optional[SingleSensorData]:
    if row.get("sonar_valid") is None and row.get("sonar_cm") is None:
        return None  # run had no sonar gate
    distance = _finite(row.get("sonar_cm"))
    age = _finite(row.get("sonar_age_s")) or 0.0
    valid = bool(row.get("sonar_valid")) and distance is not None
    return SingleSensorData(
        distance_cm=distance,
        obstacle_warning=False,
        emergency_stop=False,
        is_valid=valid,
        timestamp=now - age,
        status="OK" if valid else "TIMEOUT",
    )


def timing_summary(rows: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    """Mean/max of the stage timings stored in telemetry rows."""
    stages: Dict[str, List[float]] = {}
    for row in rows:
        stage_ms = row.get("stage_ms")
        if isinstance(stage_ms, dict):
            for name, value in stage_ms.items():
                number = _finite(value)
                if number is not None:
                    stages.setdefault(name, []).append(number)
    return {
        name: {"avg": round(sum(v) / len(v), 2), "max": round(max(v), 2), "n": len(v)}
        for name, v in stages.items()
    }


def build_report(
    session_dir: Path,
    safety_config: SafetyConfig,
    model: Any = None,
    limit: int = 0,
) -> Dict[str, Any]:
    rows, skipped = load_rows(session_dir)
    report: Dict[str, Any] = {
        "session": str(session_dir),
        "rows": len(rows),
        "rows_with_image": sum(1 for r in rows if r.get("image")),
        "malformed_lines_skipped": skipped,
        "safety_replay": replay_safety(rows, safety_config),
        "logged_stage_ms": timing_summary(rows),
    }
    if model is not None:
        report["model_replay"] = replay_model(rows, session_dir, model, limit)
    return report


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a recorded RC-car session offline.")
    parser.add_argument("session", type=Path, help="session directory (drive-* or session-*)")
    parser.add_argument("--model", help="pilot model to replay (default: config model)")
    parser.add_argument("--no-model", action="store_true", help="skip model replay, safety only")
    parser.add_argument("--limit", type=int, default=0, help="max frames for model replay")
    parser.add_argument("--stop-cm", type=float, help="override emergency_stop_distance_cm")
    parser.add_argument("--max-camera-age", type=float, help="override max_camera_age_sec")
    parser.add_argument("--max-sonar-age", type=float, help="override max_stale_sensor_sec")
    parser.add_argument("--out", type=Path, help="also write the report as JSON here")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    overrides = {
        "emergency_stop_distance_cm": args.stop_cm,
        "max_camera_age_sec": args.max_camera_age,
        "max_stale_sensor_sec": args.max_sonar_age,
    }
    safety_config = replace(SafetyConfig(), **{k: v for k, v in overrides.items() if v is not None})

    model = None
    if not args.no_model:
        try:
            from autonomous_car.behavioral_cloning import PilotModel
            from autonomous_car.config import config as app_config
        except ImportError:
            from behavioral_cloning import PilotModel
            from config import config as app_config
        model = PilotModel(args.model or app_config.behavioral_cloning.model_path)
        if not model.load():
            sys.stderr.write(f"Model not loaded ({model.error}); running safety replay only.\n")
            model = None

    try:
        report = build_report(args.session, safety_config, model, args.limit)
    except FileNotFoundError as exc:
        sys.stderr.write(f"{exc}\n")
        return 2
    text = json.dumps(report, indent=2)
    sys.stdout.write(text + "\n")
    if args.out:
        args.out.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
