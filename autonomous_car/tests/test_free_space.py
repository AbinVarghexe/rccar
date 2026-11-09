"""Offline tests for the camera free-space ranging and heading selection."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vision.free_space import (  # noqa: E402
    BAND_ANGLES_DEG, CENTER_BAND, MAX_RANGE_CM, N_BANDS, ROI_TOP_FRAC,
    FreeSpaceAnalyzer, RangeModel, choose_heading,
)

ROWS, COLS = 60, 160
A_TRUE, B_TRUE = 0.114, -0.0473        # 1/Z = a*row + b : bottom ~15 cm, ROI top ~250 cm


def _true_cm(row_frac: float) -> float:
    return 1.0 / (A_TRUE * row_frac + B_TRUE)


def _floor() -> np.ndarray:
    return np.ones((ROWS, COLS), dtype=bool)


def _row_frac(grid_row: int) -> float:
    return ROI_TOP_FRAC + (grid_row + 0.5) / ROWS * (1 - ROI_TOP_FRAC)


def test_open_floor_reports_no_obstacle():
    fs = FreeSpaceAnalyzer().update(_floor())
    assert all(r == MAX_RANGE_CM for r in fs.ranges_cm)


def test_thin_lane_line_is_not_an_obstacle():
    floor = _floor()
    floor[:, 80:83] = False                  # 3-px wide painted line running ahead
    fs = FreeSpaceAnalyzer().update(floor)
    assert fs.base_rows[CENTER_BAND] is None


def test_box_gives_contact_row_in_its_bands_only():
    floor = _floor()
    floor[20:40, 55:105] = False             # box spanning the centre bands, base at grid row 39
    bases = FreeSpaceAnalyzer._band_bases(floor)
    assert bases[CENTER_BAND] == 39
    assert bases[0] is None and bases[N_BANDS - 1] is None


def test_range_model_learns_from_sonar_pairs():
    model = RangeModel()
    assert not model.calibrated
    for row in np.linspace(0.62, 0.95, 24):
        model.add(float(row), _true_cm(float(row)))
    assert model.calibrated
    for row in (0.7, 0.8, 0.9):
        assert abs(model.range_cm(row) - _true_cm(row)) / _true_cm(row) < 0.10


def test_analyzer_calibrates_then_ranges_off_axis_obstacle():
    an = FreeSpaceAnalyzer()
    # approach a box at varying distances so the centre band sweeps through rows
    for grid_row in range(28, 52):
        floor = _floor()
        floor[10:grid_row + 1, 55:105] = False
        cm = _true_cm(_row_frac(grid_row))
        an.update(floor, sonar_cm=cm)
        an.update(floor, sonar_cm=cm)       # second reading makes the sonar "steady"
    assert an.model.calibrated
    # an obstacle only the camera sees, in the left band, is now ranged correctly
    floor = _floor()
    floor[10:45, 0:20] = False
    fs = an.update(floor, sonar_cm=None)
    expected = _true_cm(_row_frac(45))
    assert fs.calibrated and abs(fs.ranges_cm[0] - expected) / expected < 0.15


def test_choose_heading_keeps_goal_when_free():
    assert choose_heading([300.0] * N_BANDS, desired_deg=5.0) == 5.0


def test_choose_heading_steers_around_blocked_centre():
    ranges = [300.0] * N_BANDS
    for i in (CENTER_BAND - 1, CENTER_BAND, CENTER_BAND + 1):
        ranges[i] = 40.0
    heading = choose_heading(ranges, desired_deg=0.0)
    assert heading is not None and abs(heading) >= abs(BAND_ANGLES_DEG[0]) - 9.0
    assert heading in (BAND_ANGLES_DEG[0], BAND_ANGLES_DEG[1], BAND_ANGLES_DEG[-1], BAND_ANGLES_DEG[-2])


def test_choose_heading_prefers_side_nearer_the_goal():
    ranges = [300.0] * N_BANDS
    for i in (CENTER_BAND - 1, CENTER_BAND, CENTER_BAND + 1):
        ranges[i] = 40.0
    right = choose_heading(ranges, desired_deg=10.0)
    left = choose_heading(ranges, desired_deg=-10.0)
    assert right > 0 > left


def test_choose_heading_none_when_boxed_in():
    assert choose_heading([30.0] * N_BANDS, desired_deg=0.0) is None
