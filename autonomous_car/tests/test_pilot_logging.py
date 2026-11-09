"""Offline tests for stage timing, telemetry logging, the blind-camera gate,
the sonar fail-closed fix and session replay. No Pi, camera or model needed."""

import json
import os
import sys
import time
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import behavioral_cloning as bc  # noqa: E402
from config import BehavioralCloningConfig, SafetyConfig  # noqa: E402
from utils import session_replay as replay  # noqa: E402
from utils.stage_timer import StageTimer  # noqa: E402
from utils.telemetry import TelemetryLogger  # noqa: E402


def _noise_frame(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.integers(30, 220, size=(120, 160, 3), dtype=np.uint8)


# --- StageTimer ------------------------------------------------------------

def test_stage_timer_reports_avg_p95_max_and_count():
    timer = StageTimer(window=100)
    for ms in range(1, 101):
        timer.record("inference", ms / 1000.0)

    stats = timer.snapshot()["inference"]

    assert stats["count"] == 100
    assert stats["max"] == 100.0
    assert stats["p95"] == 95.0
    assert stats["avg"] == pytest.approx(50.5)
    assert stats["last"] == 100.0


def test_stage_timer_window_drops_old_samples():
    timer = StageTimer(window=5)
    for _ in range(50):
        timer.record("capture", 0.5)
    for _ in range(5):
        timer.record("capture", 0.001)

    assert timer.snapshot()["capture"]["max"] == 1.0


def test_stage_timer_records_even_when_block_raises():
    timer = StageTimer()
    with pytest.raises(RuntimeError):
        with timer.stage("safety"):
            raise RuntimeError("boom")

    assert timer.snapshot()["safety"]["count"] == 1


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1.0])
def test_stage_timer_ignores_invalid_samples(bad):
    timer = StageTimer()
    timer.record("motor", bad)

    assert timer.snapshot() == {}


# --- frame_blindness -------------------------------------------------------

def test_black_frame_is_blind():
    black = np.zeros((120, 160, 3), dtype=np.uint8)

    assert "dark" in bc.frame_blindness(black, 14.0, 4.0)


def test_flat_bright_frame_is_blind():
    flat = np.full((120, 160, 3), 128, dtype=np.uint8)

    assert "flat" in bc.frame_blindness(flat, 14.0, 4.0)


def test_textured_frame_is_not_blind():
    assert bc.frame_blindness(_noise_frame(), 14.0, 4.0) == ""


def test_empty_frame_is_blind():
    assert bc.frame_blindness(np.zeros((0, 0, 3), dtype=np.uint8), 14.0, 4.0) == "empty frame"


# --- TelemetryLogger -------------------------------------------------------

def test_telemetry_writes_rows_and_every_nth_image(tmp_path):
    logger = TelemetryLogger(tmp_path, frame_every=3, min_free_mb=0)
    session = logger.start()["session"]

    for i in range(7):
        assert logger.log({"steering": i / 10}, _noise_frame(i))
    status = logger.stop()

    rows = [json.loads(line) for line in open(os.path.join(session, "telemetry.jsonl"))]
    assert status["rows"] == 7 and len(rows) == 7
    assert [("image" in r) for r in rows] == [True, False, False, True, False, False, True]
    assert status["images"] == 3
    assert os.path.isfile(os.path.join(session, rows[0]["image"]))


def test_telemetry_inactive_log_returns_false(tmp_path):
    logger = TelemetryLogger(tmp_path, min_free_mb=0)

    assert logger.log({"a": 1}) is False


def test_telemetry_serialises_numpy_scalars(tmp_path):
    logger = TelemetryLogger(tmp_path, frame_every=0, min_free_mb=0)
    session = logger.start()["session"]

    assert logger.log({"x": np.float32(0.5), "n": np.int64(3)})
    logger.stop()

    row = json.loads(open(os.path.join(session, "telemetry.jsonl")).readline())
    assert row["x"] == 0.5 and row["n"] == 3


def test_telemetry_refuses_to_start_when_disk_is_low(tmp_path):
    logger = TelemetryLogger(tmp_path, min_free_mb=10 ** 9)

    status = logger.start()

    assert status["logging"] is False
    assert "disk" in status["stopped_reason"]


def test_telemetry_sessions_do_not_collide(tmp_path):
    first = TelemetryLogger(tmp_path, min_free_mb=0).start()["session"]
    second = TelemetryLogger(tmp_path, min_free_mb=0).start()["session"]

    assert first != second


# --- session_replay --------------------------------------------------------

def test_load_rows_skips_truncated_last_line(tmp_path):
    path = tmp_path / "telemetry.jsonl"
    path.write_text('{"steering": 0.1}\n{"steering": 0.2}\n{"steering": 0.', encoding="utf-8")

    rows, skipped = replay.load_rows(tmp_path)

    assert len(rows) == 2 and skipped == 1


def test_load_rows_reads_manual_tub_metadata(tmp_path):
    (tmp_path / "metadata.jsonl").write_text('{"image": "frame-000000.jpg", "steering": 0.3}\n')

    rows, _ = replay.load_rows(tmp_path)

    assert rows[0]["image"] == "frame-000000.jpg"


def test_load_rows_missing_log_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        replay.load_rows(tmp_path)


def test_agreement_identical_series():
    series = [-0.5, -0.2, 0.0, 0.4, 0.8]

    stats = replay.agreement_stats(series, series)

    assert stats["mae"] == 0.0 and stats["corr"] == 1.0
    assert stats["turn_sign_agreement"] == 1.0


def test_agreement_opposite_series():
    ref = [-0.5, -0.4, 0.4, 0.5]

    stats = replay.agreement_stats([-r for r in ref], ref)

    assert stats["turn_sign_agreement"] == 0.0
    assert stats["corr"] == -1.0


def test_agreement_constant_series_has_no_correlation():
    stats = replay.agreement_stats([0.2, 0.2, 0.2], [0.1, 0.3, 0.2])

    assert stats["corr"] is None


def test_agreement_length_mismatch_is_empty():
    assert replay.agreement_stats([0.1], [0.1, 0.2]) == {"n": 0}


def _drive_row(**overrides):
    row = {
        "timestamp": 1000.0,
        "proposed_throttle": 0.35,
        "proposed_steering": 0.0,
        "sonar_cm": 100.0,
        "sonar_valid": True,
        "sonar_age_s": 0.05,
        "frame_age_s": 0.05,
    }
    row.update(overrides)
    return row


def test_safety_replay_clear_path_has_no_stops():
    result = replay.replay_safety([_drive_row() for _ in range(5)], SafetyConfig())

    assert result["forward_requests_stopped"] == 0
    assert result["stop_rate"] == 0.0


def test_safety_replay_close_obstacle_stops_and_threshold_override_changes_it():
    rows = [_drive_row(sonar_cm=10.0)]

    strict = replay.replay_safety(rows, SafetyConfig())
    relaxed = replay.replay_safety(rows, replace(SafetyConfig(), emergency_stop_distance_cm=5.0))

    assert strict["forward_requests_stopped"] == 1
    assert relaxed["forward_requests_stopped"] == 0


def test_safety_replay_stale_camera_and_blind_frames_stop():
    rows = [_drive_row(frame_age_s=2.0), _drive_row(blind="scene too dark")]

    result = replay.replay_safety(rows, SafetyConfig())

    assert result["forward_requests_stopped"] == 2
    assert len(result["stop_reasons"]) == 2


def test_safety_replay_invalid_sonar_stops_forward_motion():
    result = replay.replay_safety(
        [_drive_row(sonar_cm=None, sonar_valid=False)], SafetyConfig()
    )

    assert result["forward_requests_stopped"] == 1


def test_safety_replay_stale_sonar_stops():
    result = replay.replay_safety([_drive_row(sonar_age_s=3.0)], SafetyConfig())

    assert result["forward_requests_stopped"] == 1


def test_safety_replay_groups_reasons_across_distances():
    rows = [_drive_row(sonar_cm=10.0), _drive_row(sonar_cm=12.0)]

    result = replay.replay_safety(rows, SafetyConfig())

    assert len(result["stop_reasons"]) == 1
    assert sum(result["stop_reasons"].values()) == 2


def test_safety_replay_manual_rows_without_sonar_use_label_throttle():
    rows = [{"image": "a.jpg", "steering": 0.0, "throttle": 0.4, "timestamp": 1000.0}]

    result = replay.replay_safety(rows, SafetyConfig())

    assert result["frames_with_forward_request"] == 1
    assert result["forward_requests_stopped"] == 0


class _FakeModel:
    def __init__(self, offset: float) -> None:
        self.offset = offset
        self.calls = 0

    def predict(self, frame):
        self.calls += 1
        return self.offset, 0.3


def test_build_report_replays_model_against_logged_predictions(tmp_path):
    import cv2

    rows = []
    for i in range(4):
        name = f"frame-{i:06d}.jpg"
        cv2.imwrite(str(tmp_path / name), _noise_frame(i))
        rows.append(_drive_row(image=name, pred_steering=0.2, steering=0.2))
    rows.append(_drive_row())  # no image: must be skipped by the model replay
    (tmp_path / "telemetry.jsonl").write_text("\n".join(json.dumps(r) for r in rows))

    report = replay.build_report(tmp_path, SafetyConfig(), model=_FakeModel(0.3))

    assert report["rows"] == 5 and report["rows_with_image"] == 4
    assert report["model_replay"]["n"] == 4
    assert report["model_replay"]["mae"] == pytest.approx(0.1)
    assert report["model_replay"]["bias"] == pytest.approx(0.1)


def test_cli_returns_error_for_missing_session(tmp_path, capsys):
    assert replay.main([str(tmp_path / "nope"), "--no-model"]) == 2


def test_cli_safety_only_writes_json_report(tmp_path, capsys):
    (tmp_path / "telemetry.jsonl").write_text(json.dumps(_drive_row(sonar_cm=10.0)))
    out = tmp_path / "report.json"

    code = replay.main([str(tmp_path), "--no-model", "--stop-cm", "5", "--out", str(out)])

    assert code == 0
    assert json.loads(out.read_text())["safety_replay"]["forward_requests_stopped"] == 0


# --- driver integration ----------------------------------------------------

class _FakeMotors:
    def __init__(self) -> None:
        self.drives = []

    def drive(self, throttle, steering):
        self.drives.append((throttle, steering))

    def stop(self):
        self.drives.append((0.0, 0.0))


class _FakeCamera:
    def __init__(self, frame) -> None:
        self.frame = frame

    def get_bgr_frame(self):
        return self.frame

    def frame_age_seconds(self):
        return 0.01


def _driver(tmp_path, frame, sensors=None, telemetry=False):
    config = BehavioralCloningConfig(
        data_dir=str(tmp_path / "tub"),
        telemetry_enabled=telemetry,
        telemetry_dir=str(tmp_path / "drives"),
        telemetry_frame_every=1,
        inference_interval_sec=0.03,
    )
    driver = bc.BehavioralCloningDriver(
        config, _FakeMotors(), _FakeCamera(frame), sensors, SafetyConfig()
    )
    driver.telemetry.min_free_bytes = 0
    driver.model.loaded = True
    driver.model._load_attempted = True
    driver.model.predict = lambda f: (0.25, 0.4)
    return driver


def _run_driver(driver, seconds=0.4):
    driver.start()
    driver.set_enabled(True)
    time.sleep(seconds)
    driver.stop()


def test_black_frame_stops_car_and_reports_blindness(tmp_path):
    driver = _driver(tmp_path, np.zeros((120, 160, 3), dtype=np.uint8))

    _run_driver(driver)

    assert driver.motors.drives, "loop never ran"
    assert all(t == 0.0 for t, _ in driver.motors.drives)
    assert "dark" in (driver.get_status()["camera_blind"] or "")


def test_textured_frame_drives_and_exposes_stage_latency(tmp_path):
    driver = _driver(tmp_path, _noise_frame())

    _run_driver(driver)

    assert any(t > 0.0 for t, _ in driver.motors.drives)
    stages = driver.get_status()["stage_latency_ms"]
    assert {"capture", "inference", "safety", "motor"} <= set(stages)


def test_slow_sonar_read_shows_up_as_its_own_stage(tmp_path):
    good = SimpleNamespace(
        front=SimpleNamespace(distance_cm=80.0, is_valid=True, timestamp=time.time(), status="OK")
    )

    class _SlowSensors:
        _gpio_available = True

        def is_enabled(self):
            return True

        def read_all(self):
            time.sleep(0.05)
            return good

    driver = _driver(tmp_path, _noise_frame(), _SlowSensors())

    _run_driver(driver, seconds=0.6)

    assert driver.get_status()["stage_latency_ms"]["sonar"]["avg"] >= 45.0


def test_autonomous_run_is_logged_to_telemetry(tmp_path):
    driver = _driver(tmp_path, _noise_frame(), telemetry=True)

    _run_driver(driver)

    sessions = list((tmp_path / "drives").glob("drive-*"))
    assert len(sessions) == 1
    rows = [json.loads(l) for l in open(sessions[0] / "telemetry.jsonl")]
    assert rows and rows[0]["pred_steering"] == 0.25
    assert {"steering", "throttle", "allowed", "stage_ms", "frame_age_s"} <= set(rows[0])
    assert any("image" in r for r in rows)


def test_no_telemetry_written_unless_enabled(tmp_path):
    driver = _driver(tmp_path, _noise_frame(), telemetry=False)

    _run_driver(driver)

    assert not (tmp_path / "drives").exists()


def test_sensor_read_failure_returns_last_sample_not_none(tmp_path):
    good = SimpleNamespace(
        front=SimpleNamespace(distance_cm=80.0, is_valid=True, timestamp=time.time(), status="OK")
    )

    class _Sensors:
        _gpio_available = True
        fail = False

        def read_all(self):
            if self.fail:
                raise OSError("gpio gone")
            return good

    sensors = _Sensors()
    driver = _driver(tmp_path, _noise_frame(), sensors)
    first = driver._sensor_sample()
    sensors.fail = True

    second = driver._sensor_sample()

    assert first is good.front
    assert second is good.front  # old sample kept, so the staleness gate can trip


def test_sensor_failure_before_any_sample_stays_none(tmp_path):
    class _Sensors:
        _gpio_available = True

        def read_all(self):
            raise OSError("gpio gone")

    driver = _driver(tmp_path, _noise_frame(), _Sensors())

    assert driver._sensor_sample() is None


def test_manual_recording_stores_sonar_in_metadata_but_not_csv(tmp_path):
    driver = _driver(tmp_path, _noise_frame())
    driver._front_sensor_data = SimpleNamespace(
        distance_cm=42.0, is_valid=True, timestamp=time.time(), status="OK"
    )
    driver.start_recording()

    assert driver.record_sample(steering=0.2, throttle=0.3)
    session = driver.recorder.status()["session"]
    driver.stop_recording()

    meta = json.loads(open(os.path.join(session, "metadata.jsonl")).readline())
    assert meta["sonar_cm"] == 42.0 and meta["mode"] == "manual"
    header = open(os.path.join(session, "labels.csv")).readline().strip()
    assert header == "image,steering,throttle,timestamp"
