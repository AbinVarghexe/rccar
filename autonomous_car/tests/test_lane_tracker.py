"""Lane keeping on a two-lane track made of three white tapes.

Layout: solid tape on both outer edges, dashed tape down the centre. The car
stays in its lane, may cross the dashed centre to change lanes, and must never
cross a solid outer line. All frames here are synthetic: lines are drawn in the
bird's-eye plane and warped into a camera view with the detector's own inverse
perspective, so the detector sees exactly what its geometry predicts.
"""

import os
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from vision.lane_detector import Direction, LaneDetector, LaneResult  # noqa: E402
from vision.lane_tracker import (  # noqa: E402
    ThreeLineLaneDetector,
    constrain_side_clearance,
)

DASH = 24  # 12 px on, 12 px off in the bird's-eye plane
LANE = 100  # lane width in bird's-eye px for most tests


def _line(x0, dashed=False, bend=0.0, phase=0):
    """A tape line; ``bend`` moves it sideways with distance (bird's-eye px at the top)."""
    return {"x0": x0, "dashed": dashed, "bend": bend, "phase": phase}


def render(det, lines):
    bev = np.zeros((240, 400, 3), np.uint8)
    for ln in lines:
        for y in range(240):
            if ln["dashed"] and ((y + ln["phase"]) // (DASH // 2)) % 2:
                continue
            t = (240 - y) / 240.0
            x = int(round(ln["x0"] + ln["bend"] * t * t))
            if 0 <= x < 400:
                bev[y, max(0, x - 3): x + 4] = 255
    return cv2.warpPerspective(bev, det.Minv, (400, 240))


@pytest.fixture()
def det():
    return ThreeLineLaneDetector()


def _centre(det):
    return det.bev_center_x


def left_lane(det, shift=0, **kw):
    """Car in the left lane: solid outer edge on its left, dashed centre on its right."""
    c = _centre(det)
    return [_line(c - LANE // 2 + shift, **kw), _line(c + LANE // 2 + shift, dashed=True, **kw)]


def right_lane(det, shift=0):
    """Car in the right lane: dashed centre on its left, solid outer edge on its right."""
    c = _centre(det)
    return [_line(c - LANE // 2 + shift, dashed=True), _line(c + LANE // 2 + shift)]


# --- straight lane, both lanes ------------------------------------------------

def test_centred_in_left_lane_has_no_error(det):
    r = det.detect_from_bgr(render(det, left_lane(det)))

    assert r.lane_detected
    assert abs(r.error) <= 6
    assert r.lane_side == "left"
    assert (r.left_kind, r.right_kind) == ("solid", "dashed")


def test_centred_in_right_lane_has_no_error(det):
    r = det.detect_from_bgr(render(det, right_lane(det)))

    assert abs(r.error) <= 6
    assert r.lane_side == "right"
    assert (r.left_kind, r.right_kind) == ("dashed", "solid")


def test_dashed_centre_no_longer_beats_the_far_edge_when_all_three_tapes_are_visible(det):
    """The old detector picked the far solid edge here and steered by 23-28 px."""
    c = _centre(det)
    lines = [_line(c - 25), _line(c + 25, dashed=True), _line(c + 75)]

    r = det.detect_from_bgr(render(det, lines))

    assert abs(r.error) <= 8
    assert r.right_kind == "dashed"
    assert r.lane_side == "left"


@pytest.mark.parametrize("shift", [-30, -15, 0, 15, 30])
def test_error_tracks_the_sideways_offset_with_the_right_sign(det, shift):
    r = det.detect_from_bgr(render(det, left_lane(det, shift)))

    assert r.error == pytest.approx(shift, abs=7)


def test_steering_pushes_back_toward_the_lane_centre(det):
    c = _centre(det)
    car_left_of_centre = ThreeLineLaneDetector().detect_from_bgr(render(det, left_lane(det, 25)))
    car_right_of_centre = ThreeLineLaneDetector().detect_from_bgr(render(det, left_lane(det, -25)))

    assert car_left_of_centre.suggested_steering > 0 > car_right_of_centre.suggested_steering
    assert c == _centre(det)


def test_lane_width_is_learned_from_a_solid_dashed_pair(det):
    det.detect_from_bgr(render(det, left_lane(det)))

    assert det.nominal_lane_width == pytest.approx(LANE, abs=6)


# --- solid versus dashed -------------------------------------------------------

def test_solid_and_dashed_are_classified_stably_over_changing_dash_phase(det):
    kinds = []
    for phase in range(0, 24, 4):
        c = _centre(det)
        lines = [_line(c - 50), _line(c + 50, dashed=True, phase=phase)]
        r = det.detect_from_bgr(render(det, lines))
        kinds.append((r.left_kind, r.right_kind))

    assert set(kinds) == {("solid", "dashed")}


# --- curves --------------------------------------------------------------------

def test_right_hand_bend_makes_the_car_steer_right_before_it_arrives(det):
    c = _centre(det)
    lines = [_line(c - 50, bend=120), _line(c + 50, dashed=True, bend=120)]

    r = det.detect_from_bgr(render(det, lines))

    assert r.look_ahead_error > 20
    assert r.error > 8
    assert r.suggested_steering > 0


def test_left_hand_bend_makes_the_car_steer_left(det):
    c = _centre(det)
    lines = [_line(c - 50, bend=-90), _line(c + 50, dashed=True, bend=-90)]

    r = det.detect_from_bgr(render(det, lines))

    assert r.look_ahead_error < -15
    assert r.error < -6


def test_bend_reads_more_error_than_the_same_lane_straight(det):
    c = _centre(det)
    straight = ThreeLineLaneDetector().detect_from_bgr(render(det, left_lane(det)))
    bent = ThreeLineLaneDetector().detect_from_bgr(
        render(det, [_line(c - 50, bend=100), _line(c + 50, dashed=True, bend=100)])
    )

    assert bent.error > straight.error + 6


# --- hard outer limits, soft centre -------------------------------------------

def test_close_to_the_solid_outer_line_forbids_steering_further_out(det):
    r = det.detect_from_bgr(render(det, left_lane(det, shift=30)))

    assert r.is_left_boundary_violated
    assert r.departure_warning == "LEFT"
    assert r.min_allowed_steering == 0.0
    assert r.max_allowed_steering == 1.0


def test_close_to_the_right_outer_line_forbids_steering_right(det):
    r = det.detect_from_bgr(render(det, right_lane(det, shift=-30)))

    assert r.is_right_boundary_violated
    assert r.max_allowed_steering == 0.0
    assert r.min_allowed_steering == -1.0


def test_hugging_the_outer_line_on_a_bend_toward_it_still_lets_the_lane_steer_that_way(det):
    """The limit stops outside influences pushing the car out; it must not stop the
    lane's own steering, or the car would drive straight over the line on a bend."""
    c = _centre(det)
    # Car close to the left tape (8 px inside the margin) with a sharp left bend ahead.
    lines = [_line(c - 38, bend=-160), _line(c + 62, dashed=True, bend=-160)]

    r = det.detect_from_bgr(render(det, lines))

    assert r.is_left_boundary_violated
    assert r.error < -1.0
    assert r.min_allowed_steering == pytest.approx(r.error / 100.0, abs=0.02)
    assert r.min_allowed_steering < 0.0
    assert r.suggested_steering < 0.0


def test_hugging_the_right_outer_line_on_a_bend_toward_it_still_lets_the_lane_steer_that_way(det):
    c = _centre(det)
    lines = [_line(c - 62, dashed=True, bend=160), _line(c + 38, bend=160)]

    r = det.detect_from_bgr(render(det, lines))

    assert r.is_right_boundary_violated
    assert r.error > 1.0
    assert r.max_allowed_steering == pytest.approx(r.error / 100.0, abs=0.02)
    assert r.max_allowed_steering > 0.0


def test_dashed_centre_line_is_never_a_hard_wall(det):
    """Right on top of the dashed centre the car may still steer across it."""
    r = det.detect_from_bgr(render(det, left_lane(det, shift=-45)))
    c = _centre(det)

    assert r.right_line_x is not None and r.right_line_x - c < 20
    assert not r.is_right_boundary_violated
    assert r.max_allowed_steering == 1.0


def test_outer_side_is_blocked_for_the_avoider_and_centre_side_is_not(det):
    left = det.detect_from_bgr(render(det, left_lane(det)))
    right = ThreeLineLaneDetector().detect_from_bgr(render(det, right_lane(det)))

    assert "left" in left.blocked_sides and "right" not in left.blocked_sides
    assert "right" in right.blocked_sides and "left" not in right.blocked_sides


def test_side_clearance_for_a_blocked_side_is_zeroed():
    res = LaneResult()
    res.blocked_sides = ("left",)

    assert constrain_side_clearance(0.8, 0.6, res) == (0.0, 0.6)


def test_side_clearance_untouched_when_nothing_is_blocked():
    assert constrain_side_clearance(0.8, 0.6, LaneResult()) == (0.8, 0.6)


# --- partial views and memory --------------------------------------------------

def test_only_the_dashed_centre_visible_still_centres_using_the_learned_width(det):
    det.detect_from_bgr(render(det, left_lane(det)))
    c = _centre(det)

    r = det.detect_from_bgr(render(det, [_line(c + LANE // 2, dashed=True)]))

    assert r.lane_detected
    assert abs(r.error) <= 8
    assert r.lane_side == "left"  # remembered from the earlier full view
    assert "left" in r.blocked_sides


def test_only_the_solid_outer_line_visible_still_centres(det):
    det.detect_from_bgr(render(det, right_lane(det)))
    c = _centre(det)

    r = det.detect_from_bgr(render(det, [_line(c + LANE // 2)]))

    assert abs(r.error) <= 8
    assert r.is_right_boundary_violated is False


def test_both_outer_lines_seen_with_the_centre_in_a_gap_keeps_the_remembered_lane(det):
    det.detect_from_bgr(render(det, [_line(_centre(det) - 40), _line(_centre(det) + 40, dashed=True)]))
    c = _centre(det)

    r = det.detect_from_bgr(render(det, [_line(c - 80), _line(c + 80)]))

    assert r.lane_side == "left"
    assert r.error == pytest.approx(-40, abs=10)  # left lane centre is 40 px left of the car


def test_first_view_of_two_outer_lines_infers_the_lane_width_from_the_road(det):
    c = _centre(det)

    r = det.detect_from_bgr(render(det, [_line(c - 80), _line(c + 80)]))

    assert r.lane_detected
    assert det.nominal_lane_width == pytest.approx(80, abs=8)


def test_no_lines_means_no_lane_and_zero_steering(det):
    r = det.detect_from_bgr(np.zeros((240, 400, 3), np.uint8))

    assert not r.lane_detected
    assert r.direction == Direction.NO_LANE
    assert r.error == 0.0 and r.suggested_steering == 0.0


def test_a_noisy_floor_speck_is_not_a_lane(det):
    frame = np.zeros((240, 400, 3), np.uint8)
    cv2.circle(frame, (200, 190), 4, (255, 255, 255), -1)

    assert not det.detect_from_bgr(frame).lane_detected


def test_lines_vanish_after_being_seen_and_the_error_does_not_linger(det):
    det.detect_from_bgr(render(det, left_lane(det, 20)))

    r = det.detect_from_bgr(np.zeros((240, 400, 3), np.uint8))

    assert not r.lane_detected and r.error == 0.0


# --- a real floor is not black ------------------------------------------------

def scene(det, lines, floor=100, tape_v=235, glare=None, specks=0, seed=0):
    """Tape on a textured, unevenly lit grey floor, optionally with glare / dust specks."""
    rng = np.random.default_rng(seed)
    cam = render(det, lines)
    tape = cam[..., :1] > 0
    img = np.full((240, 400, 3), float(floor))
    img += rng.normal(0, 9, img.shape)
    img += np.linspace(-18, 18, 400)[None, :, None]
    img = np.where(tape, tape_v + rng.normal(0, 6, img.shape), img)
    for _ in range(specks):
        cv2.circle(img, (int(rng.integers(0, 400)), int(rng.integers(130, 235))),
                   int(rng.integers(2, 5)), (255, 255, 255), -1)
    if glare:
        cv2.circle(img, glare[:2], glare[2], (255, 255, 255), -1)
    return np.clip(img, 0, 255).astype(np.uint8)


def _settled(det, **kw):
    """Three frames of the same scene so solid/dashed classification can settle."""
    c = _centre(det)
    lines = [_line(c - 50), _line(c + 50, dashed=True)]
    result = None
    for i in range(3):
        result = det.detect_from_bgr(scene(det, lines, seed=i, **kw))
    return result


def _is_the_lane(r):
    return r.lane_detected and abs(r.error) <= 8 and (r.left_kind, r.right_kind) == ("solid", "dashed")


def test_textured_unevenly_lit_floor(det):
    assert _is_the_lane(_settled(det))


@pytest.mark.parametrize("floor, tape_v", [(150, 235), (170, 240), (140, 185), (80, 150)])
def test_floor_brightness_does_not_matter_only_tape_contrast(det, floor, tape_v):
    assert _is_the_lane(_settled(det, floor=floor, tape_v=tape_v))


def test_big_glare_patch_inside_the_lane_is_not_a_line(det):
    assert _is_the_lane(_settled(det, glare=(226, 185, 30)))


def test_glare_patch_outside_the_lane_is_ignored(det):
    # camera x=60 is well left of the left tape (camera x~121 at this row)
    assert _is_the_lane(_settled(det, glare=(60, 190, 18)))


def test_bright_dust_specks_are_ignored(det):
    assert _is_the_lane(_settled(det, specks=12))


def test_glare_and_specks_together(det):
    r = _settled(det, glare=(300, 200, 22), specks=10)

    assert r.lane_detected and abs(r.error) <= 8
    assert r.left_kind == "solid" and r.right_kind in (None, "dashed")


def test_a_closer_stray_line_loses_to_the_neighbour_that_matches_the_lane_width(det):
    """Once the lane width is known, a stray line 69 px away must not replace the real
    dashed centre line 100 px away just because it is nearer."""
    c = _centre(det)
    for _ in range(5):
        det.detect_from_bgr(render(det, left_lane(det)))
    stray = [_line(c - 50), _line(c + 19, dashed=True), _line(c + 50, dashed=True)]

    r = det.detect_from_bgr(render(det, stray))

    assert abs(r.error) <= 8
    assert r.right_line_x == pytest.approx(c + 50, abs=6)


@pytest.mark.parametrize("bend", [80, -80])
@pytest.mark.parametrize("seed", [0, 1, 2, 3])
def test_specks_on_a_bend_do_not_form_a_fake_line(det, bend, seed):
    """Scattered specks can line up into a chain; real tape is a smooth, dense curve."""
    c = _centre(det)
    lines = [_line(c - 50, bend=bend), _line(c + 50, dashed=True, bend=bend)]
    for i in range(6):  # a few frames of driving so the lane width is known
        r = det.detect_from_bgr(scene(det, lines, specks=12, seed=seed * 10 + i))
    truth = 0.6 * bend * (15 / 240) ** 2 + 0.4 * bend * 0.25

    assert r.error == pytest.approx(truth, abs=8)
    assert r.left_kind == "solid"
    # The dashed centre may drop out for a frame, but must never turn into a solid wall.
    assert r.right_kind in (None, "dashed")


def test_a_bogus_first_frame_does_not_lock_the_real_lane_out(det):
    """A stray dashed line in the lane teaches a width of 55. The real lane (100) must
    still be accepted afterwards instead of being rejected as 'too wide' forever."""
    c = _centre(det)
    det.detect_from_bgr(render(det, [_line(c - 50), _line(c + 5, dashed=True)]))
    good = render(det, left_lane(det))

    results = [det.detect_from_bgr(good) for _ in range(8)]
    r = results[-1]

    assert r.right_kind == "dashed" and r.lane_side == "left"
    assert abs(r.error) <= 8
    assert det.nominal_lane_width == pytest.approx(LANE, abs=10)


def test_lane_width_is_a_median_so_a_single_outlier_barely_moves_it(det):
    c = _centre(det)
    good = render(det, left_lane(det))
    for _ in range(9):
        det.detect_from_bgr(good)

    det.detect_from_bgr(render(det, [_line(c - 50), _line(c + 5, dashed=True)]))

    assert det.nominal_lane_width == pytest.approx(LANE, abs=4)


def test_a_short_bright_streak_is_not_a_line(det):
    """A reflection streak 40 rows tall spans less than half the view."""
    frame = np.zeros((240, 400, 3), np.uint8)
    bev = np.zeros((240, 400, 3), np.uint8)
    bev[170:215, 223:230] = 255
    frame = cv2.warpPerspective(bev, det.Minv, (400, 240))

    assert not det.detect_from_bgr(frame).lane_detected


# --- result compatibility with the follower -----------------------------------

def test_result_fills_every_field_the_follower_reads(det):
    r = det.detect_from_bgr(render(det, left_lane(det)))

    assert r.contour_found and r.lane_center_x is not None
    assert r.annotated_frame.shape == (240, 400, 3)
    assert r.birdseye_frame.shape == (240, 400, 3)
    assert isinstance(r.direction, Direction)
    assert -1.0 <= r.min_allowed_steering <= 0.0 <= r.max_allowed_steering <= 1.0


def test_other_frame_sizes_are_accepted(det):
    big = cv2.resize(render(det, left_lane(det)), (800, 480))

    r = det.detect_from_bgr(big)

    assert r.lane_detected
    assert r.annotated_frame.shape == (480, 800, 3)


def _drive(follower, frames):
    """Feed frames with a healthy fresh sonar reading; return the motor commands."""
    import time as _time

    commands = []
    for frame in frames:
        cmd, _servo, _hud, _telem = follower.process_frame(frame, 200.0, _time.time())
        commands.append(cmd)
    return commands


def _pilot_follower(layout, steer):
    """A follower whose neural pilot insists on ``steer`` (an outside influence)."""
    from control.line_follower import LineFollower, NavigationMode

    follower = LineFollower(lane_layout=layout)
    follower.ai_loaded = True
    follower.pilot_model.predict = lambda _frame: (steer, 0.3)
    follower.set_navigation_mode(NavigationMode.AI_ONNX_PILOT)
    follower.start()
    return follower


def test_pilot_cannot_push_the_car_over_the_solid_outer_line():
    det = ThreeLineLaneDetector()
    frame = render(det, left_lane(det, shift=30))  # hugging the left outer tape
    follower = _pilot_follower("three_line", steer=-0.9)  # pilot wants hard left

    commands = _drive(follower, [frame] * 14)

    assert "LEFT" not in commands
    assert follower._steer_smooth >= -1e-6


def test_pilot_may_steer_left_when_clear_of_the_outer_line():
    det = ThreeLineLaneDetector()
    frame = render(det, left_lane(det, shift=-10))  # comfortably inside the lane
    follower = _pilot_follower("three_line", steer=-0.9)

    commands = _drive(follower, [frame] * 14)

    assert "LEFT" in commands


def test_the_dashboard_service_uses_the_three_line_detector(monkeypatch):
    import importlib.util

    root = os.path.join(os.path.dirname(__file__), "..")
    monkeypatch.chdir(root)  # model paths in the service script are relative
    spec = importlib.util.spec_from_file_location(
        "service_under_test", os.path.join(root, "scripts", "test_dashboard.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert isinstance(module.line_follower.lane_detector, ThreeLineLaneDetector)
    assert module.line_follower.lane_layout == "three_line"


def test_follower_picks_the_detector_from_its_layout_flag():
    from control.line_follower import LineFollower

    classic = LineFollower()
    three = LineFollower(lane_layout="three_line")

    assert type(classic.lane_detector) is LaneDetector
    assert isinstance(three.lane_detector, ThreeLineLaneDetector)
