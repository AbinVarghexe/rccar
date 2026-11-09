"""A dead serial link must never produce a believable sonar distance.

Regression tests for the fake 68 +/- 14 cm reading that scripts/test_dashboard.py
used to invent when the Arduino link was down. It carried no timestamp, so the
obstacle avoider stamped it "now" and treated it as a fresh, clear reading.
"""

import importlib.util
import os
import sys
import time
from types import SimpleNamespace

import pytest

AUTONOMOUS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, AUTONOMOUS_DIR)

from control.obstacle_avoider import AvoidState, ObstacleAvoider, trusted_reading  # noqa: E402


class _Stop(BaseException):
    """Ends reader_thread's endless loop; BaseException so it is not swallowed."""


@pytest.fixture()
def dash(monkeypatch):
    """The real test_dashboard module, loaded without starting any thread."""
    monkeypatch.chdir(AUTONOMOUS_DIR)  # model paths in the module are relative
    spec = importlib.util.spec_from_file_location(
        "test_dashboard_under_test", os.path.join(AUTONOMOUS_DIR, "scripts", "test_dashboard.py")
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fake_time = SimpleNamespace(time=time.time, sleep=lambda _s: None)
    monkeypatch.setattr(module, "time", fake_time)

    def stop_after_one_pass():
        raise _Stop

    monkeypatch.setattr(module, "_trigger_reconnect", stop_after_one_pass)
    return module


# --- trusted_reading ---------------------------------------------------------

@pytest.mark.parametrize("dist, ts", [(None, None), (None, 5.0), (55.0, None)])
def test_reading_without_distance_or_timestamp_is_dropped(dist, ts):
    assert trusted_reading(dist, ts) == (None, None)


def test_reading_with_distance_and_timestamp_passes_through():
    assert trusted_reading(55.0, 123.4) == (55.0, 123.4)


def test_untimestamped_distance_is_a_sensor_fault_not_a_clear_path():
    now = time.time()
    avoider = ObstacleAvoider()

    decision = avoider.update(now, *trusted_reading(70.0, None))

    assert decision.state == AvoidState.SENSOR_FAULT
    assert decision.cmd == "STOP"


def test_genuine_timestamped_distance_still_drives_normally():
    now = time.time()
    avoider = ObstacleAvoider()

    decision = avoider.update(now, *trusted_reading(120.0, now))

    assert decision.state == AvoidState.CLEAR
    assert decision.cmd is None


def test_genuine_reading_goes_stale_and_stops():
    now = time.time()
    avoider = ObstacleAvoider()
    avoider.update(now, *trusted_reading(120.0, now))

    decision = avoider.update(now + 2.0, *trusted_reading(120.0, now))

    assert decision.state == AvoidState.SENSOR_FAULT


# --- test_dashboard serial-offline behaviour ---------------------------------

def test_offline_sonar_clears_a_leftover_reading(dash):
    dash.latest_dist, dash.latest_dist_ts = 70, None

    dash._offline_sonar()

    assert dash.latest_dist is None and dash.latest_dist_ts is None


def test_offline_sonar_clears_a_real_but_old_reading(dash):
    dash.latest_dist, dash.latest_dist_ts = 40, time.time() - 30

    dash._offline_sonar()

    assert dash.latest_dist is None and dash.latest_dist_ts is None


def test_simulation_is_opt_in_and_always_timestamped(dash, monkeypatch):
    monkeypatch.setattr(dash, "SIM_SONAR", True)
    dash.latest_dist, dash.latest_dist_ts = None, None

    dash._offline_sonar()

    assert 54 <= dash.latest_dist <= 82
    assert dash.latest_dist_ts is not None


def test_simulation_is_off_by_default(dash):
    assert dash.SIM_SONAR is False


def test_reader_thread_without_serial_reports_no_sonar_data(dash):
    dash.ser = None
    dash.latest_dist, dash.latest_dist_ts = 70, None

    with pytest.raises(_Stop):
        dash.reader_thread()

    assert dash.latest_dist is None and dash.latest_dist_ts is None


def test_reader_thread_serial_error_reports_no_sonar_data(dash):
    class _BrokenPort:
        @property
        def is_open(self):
            raise OSError("device unplugged")

    dash.ser = _BrokenPort()
    dash.latest_dist, dash.latest_dist_ts = 70, None

    with pytest.raises(_Stop):
        dash.reader_thread()

    assert dash.latest_dist is None and dash.latest_dist_ts is None
    assert "reconnecting" in dash.serial_status


def test_vision_loop_only_feeds_the_follower_trusted_readings(dash):
    import inspect

    source = inspect.getsource(dash.camera_vision_thread)

    assert "trusted_reading(latest_dist, latest_dist_ts)" in source
    assert "process_frame(bgr_frame, latest_dist" not in source
