"""Fixed camera positions (look left / look right / centre) with a side obstacle warning.

The camera servo has three jobs only: sit at centre, or turn to a fixed left/right
angle on command so the car can check what is beside it. There is no sweep that
hunts for a path. One controller owns the servo, so the dashboard slider, the look
commands and the follower can no longer fight over it.
"""

import json
import os
import sys
import threading
import time
import urllib.parse
import urllib.request

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from control.side_look import SideLook  # noqa: E402
from vision.side_obstacle import ObstacleWatch, SideObstacle, find_side_obstacle  # noqa: E402

AUTONOMOUS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


# --- synthetic frames ----------------------------------------------------------

def floor_frame(seed=0, grey=112):
    rng = np.random.default_rng(seed)
    img = np.full((240, 400, 3), float(grey)) + rng.normal(0, 5, (240, 400, 3))
    return np.clip(img, 0, 255).astype(np.uint8)


def box(frame, x0, y0, x1, y1, colour=(40, 40, 200)):
    out = frame.copy()
    cv2.rectangle(out, (x0, y0), (x1, y1), colour, -1)
    return out


def floor_model(frame):
    """Mean/std LAB of the floor, computed the way the follower does (160x58 ROI)."""
    roi = cv2.resize(frame[int(240 * 0.4):], (160, 58), interpolation=cv2.INTER_AREA)
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)[-14:].reshape(-1, 3)
    return lab.mean(axis=0), np.maximum(lab.std(axis=0), [10.0, 5.0, 5.0])


# --- find_side_obstacle --------------------------------------------------------

def test_empty_floor_has_no_obstacle():
    assert not find_side_obstacle(floor_frame()).present


def test_box_on_the_floor_is_found_where_it_stands():
    frame = box(floor_frame(), 40, 130, 140, 215)

    obs = find_side_obstacle(frame)

    assert obs.present
    assert obs.cx < 0.35               # left part of the image
    assert 0.80 < obs.base < 0.95      # its lowest edge is near the bottom of the view
    assert obs.area_frac > 0.05


def test_a_tiny_speck_is_not_an_obstacle():
    assert not find_side_obstacle(box(floor_frame(), 200, 200, 204, 204)).present


def test_a_thin_tape_line_across_the_floor_is_not_an_obstacle():
    frame = floor_frame()
    cv2.line(frame, (0, 190), (399, 170), (235, 235, 235), 3)

    assert not find_side_obstacle(frame).present


def test_known_floor_model_finds_an_obstacle_that_fills_the_bottom_of_the_view():
    """A wall right beside the car fills the bottom band; judging the floor from that band
    alone would call the wall 'floor'. The follower's own floor model avoids that."""
    clear = floor_frame()
    wall = box(clear, 0, 100, 399, 239)
    mean, std = floor_model(clear)

    with_model = find_side_obstacle(wall, mean, std)
    fallback = find_side_obstacle(wall)

    assert with_model.present and with_model.area_frac > 0.6   # the wall is most of the view
    assert fallback.from_band                                   # flagged: it took the wall for floor
    assert fallback.area_frac < 0.1


def test_floor_model_flag_is_reported():
    mean, std = floor_model(floor_frame())

    assert not find_side_obstacle(floor_frame(), mean, std).from_band


def test_missing_frame_is_not_an_obstacle():
    assert not find_side_obstacle(None).present
    assert not find_side_obstacle(np.zeros((0, 0, 3), np.uint8)).present


# --- ObstacleWatch -------------------------------------------------------------

def obs(cx=0.3, base=0.8, present=True):
    return SideObstacle(present, 0.1 if present else 0.0, cx, base)


def run_watch(watch, samples, dt=0.2):
    result = None
    for i, sample in enumerate(samples):
        result = watch.update(100.0 + i * dt, sample)
    return result


def test_nothing_seen_reads_clear():
    assert ObstacleWatch().update(1.0, obs(present=False)).status == "clear"


def test_a_stationary_obstacle_is_reported_as_an_obstacle():
    result = run_watch(ObstacleWatch(), [obs()] * 8)

    assert result.status == "obstacle"
    assert result.seen_s == pytest.approx(1.4, abs=0.05)


def test_an_obstacle_getting_closer_is_approaching():
    samples = [obs(base=0.6 + 0.03 * i) for i in range(8)]  # +0.15 per second

    assert run_watch(ObstacleWatch(), samples).status == "approaching"


def test_an_obstacle_sliding_sideways_is_moving():
    samples = [obs(cx=0.2 + 0.04 * i) for i in range(8)]  # +0.2 per second

    assert run_watch(ObstacleWatch(), samples).status == "moving"


def sized(area, cx=0.3, base=0.8):
    """A sample whose blob is ``area`` big; 'present' only if it reaches the entry size."""
    return SideObstacle(area >= 0.02, area, cx, base)


def test_one_noisy_frame_does_not_raise_a_warning():
    watch = ObstacleWatch()

    first = watch.update(100.0, obs())
    later = [watch.update(100.2 + 0.2 * i, obs(present=False)) for i in range(6)]

    assert first.status == "clear"
    assert {w.status for w in later} == {"clear"}


def test_a_blob_flickering_around_the_entry_size_never_confirms():
    watch = ObstacleWatch()
    samples = [sized(0.021), sized(0.018), sized(0.022), sized(0.017)] * 4

    statuses = {watch.update(100.0 + 0.2 * i, s).status for i, s in enumerate(samples)}

    assert statuses == {"clear"}


def test_an_obstacle_is_confirmed_after_a_few_frames_in_a_row():
    watch = ObstacleWatch()

    statuses = [watch.update(100.0 + 0.2 * i, obs()).status for i in range(5)]

    assert statuses[:2] == ["clear", "clear"]
    assert statuses[2:] == ["obstacle"] * 3


def test_a_confirmed_obstacle_survives_dipping_just_under_the_entry_size():
    watch = ObstacleWatch()
    for i in range(4):
        watch.update(100.0 + 0.2 * i, sized(0.05))

    # 2.4 s of dipping, far longer than the 1 s 'gone' debounce, so only the exit-size rule can hold it
    sizes = (0.015, 0.012, 0.018, 0.014) * 3
    dips = [watch.update(100.8 + 0.2 * i, sized(a)).status for i, a in enumerate(sizes)]

    assert set(dips) == {"obstacle"}


def test_a_confirmed_obstacle_that_really_shrinks_away_does_clear():
    watch = ObstacleWatch(clear_after_s=0.5)
    for i in range(4):
        watch.update(100.0 + 0.2 * i, sized(0.05))

    tail = [watch.update(100.8 + 0.2 * i, sized(0.002)) for i in range(8)]

    assert tail[-1].status == "clear" and any(w.cleared for w in tail)


def test_it_clears_only_after_staying_gone_for_a_moment():
    watch = ObstacleWatch(clear_after_s=1.0)
    run_watch(watch, [obs()] * 4)

    flicker = watch.update(101.0, obs(present=False))
    gone = [watch.update(101.0 + 0.3 * i, obs(present=False)) for i in range(1, 6)]

    assert flicker.status == "obstacle"          # one empty frame is not 'clear'
    assert gone[-1].status == "clear"
    assert any(g.cleared for g in gone)           # the moment it left is reported


def test_cleared_is_reported_once():
    watch = ObstacleWatch(clear_after_s=0.5)
    run_watch(watch, [obs()] * 3)
    events = [watch.update(102.0 + 0.3 * i, obs(present=False)).cleared for i in range(8)]

    assert events.count(True) == 1


# --- SideLook ------------------------------------------------------------------

class Clock:
    def __init__(self):
        self.t = 1000.0

    def step(self, dt):
        self.t += dt
        return self.t


def drive(look, clock, frames, seconds, dt=0.1, model=(None, None)):
    """Advance the controller, feeding it frames, and record the angles it commands."""
    angles = []
    for _ in range(int(seconds / dt)):
        now = clock.step(dt)
        look.update(now, frames() if callable(frames) else frames, *model)
        angles.append(look.angle(now))
    return angles


def make_look(**kw):
    kw.setdefault("settle_s", 0.5)
    kw.setdefault("sample_s", 1.0)
    return SideLook(**kw)


def test_idle_has_no_opinion_and_is_not_active():
    look = make_look()

    assert look.angle(1000.0) is None and not look.active


def test_look_left_turns_to_the_fixed_left_angle_at_once():
    look, clock = make_look(), Clock()

    look.request("left", clock.t)

    assert look.angle(clock.t) == look.left_angle and look.active


def test_look_right_turns_to_the_fixed_right_angle():
    look, clock = make_look(), Clock()

    look.request("right", clock.t)

    assert look.angle(clock.t) == look.right_angle


def test_unknown_direction_is_refused():
    look = make_look()

    assert look.request("up", 1000.0) is False
    assert look.angle(1000.0) is None


def test_nothing_is_judged_while_the_camera_is_still_moving():
    look, clock = make_look(), Clock()
    look.request("left", clock.t)

    look.update(clock.step(0.2), box(floor_frame(), 40, 130, 140, 215), None, None)

    assert look.status(clock.t)["sides"]["left"]["status"] == "unknown"


def test_obstacle_on_the_left_gives_a_warning_naming_the_side():
    look, clock = make_look(), Clock()
    look.request("left", clock.t)
    frame = box(floor_frame(), 40, 130, 140, 215)

    drive(look, clock, frame, 1.5)
    status = look.status(clock.t)

    assert status["sides"]["left"]["status"] == "obstacle"
    assert "LEFT" in status["warning"]
    assert status["advice"] == "HOLD"


def test_clear_left_side_gives_no_warning():
    look, clock = make_look(), Clock()
    look.request("left", clock.t)

    drive(look, clock, floor_frame(), 2.0)
    status = look.status(clock.t)

    assert status["sides"]["left"]["status"] == "clear"
    assert status["warning"] is None


def test_a_look_that_finds_nothing_returns_to_centre_and_goes_idle():
    look, clock = make_look(), Clock()
    look.request("left", clock.t)

    angles = drive(look, clock, floor_frame(), 4.0)

    assert angles[0] == look.left_angle
    assert angles[-1] is None and not look.active
    assert look.centre_angle in angles  # it went back through centre


def test_an_obstacle_keeps_the_camera_on_it_while_it_stays():
    look, clock = make_look(observe_max_s=5.0), Clock()
    look.request("left", clock.t)
    frame = box(floor_frame(), 40, 130, 140, 215)

    angles = drive(look, clock, frame, 4.0)

    assert all(a == look.left_angle for a in angles)  # still watching it


def test_observing_gives_up_after_the_limit_and_the_warning_stays():
    look, clock = make_look(observe_max_s=2.0), Clock()
    look.request("left", clock.t)
    frame = box(floor_frame(), 40, 130, 140, 215)

    angles = drive(look, clock, frame, 5.0)

    assert angles[-1] is None
    assert "LEFT" in look.status(clock.t)["warning"]


def test_when_the_obstacle_leaves_the_warning_clears_and_the_camera_returns():
    look, clock = make_look(observe_max_s=10.0, clear_after_s=0.5), Clock()
    look.request("left", clock.t)
    present = box(floor_frame(), 40, 130, 140, 215)
    drive(look, clock, present, 1.8)
    assert look.status(clock.t)["warning"]

    angles = drive(look, clock, floor_frame(), 4.0, model=floor_model(floor_frame()))
    status = look.status(clock.t)

    assert status["warning"] is None and status["sides"]["left"]["status"] == "clear"
    assert status["advice"] == "CLEAR_TO_MOVE"
    assert angles[-1] is None


def test_clear_judged_without_a_floor_model_is_not_trusted_enough_to_advise_moving():
    """Without the follower's floor model a wall filling the bottom of the view reads as floor."""
    look, clock = make_look(), Clock()
    look.request("left", clock.t)

    drive(look, clock, floor_frame(), 2.0)  # no floor model
    status = look.status(clock.t)

    assert status["sides"]["left"]["status"] == "clear"
    assert status["sides"]["left"]["low_confidence"] is True
    assert status["advice"] == "UNKNOWN"


def test_check_looks_left_then_right_then_returns_to_centre():
    look, clock = make_look(), Clock()
    look.request("check", clock.t)

    angles = drive(look, clock, floor_frame(), 6.0)
    seen = [a for i, a in enumerate(angles) if i == 0 or a != angles[i - 1]]

    assert seen[:3] == [look.left_angle, look.right_angle, look.centre_angle]
    assert angles[-1] is None
    status = look.status(clock.t)
    assert status["sides"]["left"]["status"] == status["sides"]["right"]["status"] == "clear"


def test_check_warns_about_the_side_that_has_the_obstacle_only():
    look, clock = make_look(), Clock()
    look.request("check", clock.t)
    clear = floor_frame()
    blocked = box(floor_frame(), 250, 130, 360, 215)

    def frames():
        return blocked if look.angle(clock.t) == look.right_angle else clear

    drive(look, clock, frames, 6.0)
    status = look.status(clock.t)

    assert status["sides"]["right"]["status"] == "obstacle"
    assert status["sides"]["left"]["status"] == "clear"
    assert "RIGHT" in status["warning"] and "LEFT" not in status["warning"]


def test_centre_command_cancels_a_look_and_returns_home():
    look, clock = make_look(), Clock()
    look.request("left", clock.t)
    drive(look, clock, floor_frame(), 0.3)

    look.request("centre", clock.t)
    angles = drive(look, clock, floor_frame(), 1.5)

    assert angles[0] == look.centre_angle
    assert angles[-1] is None


def test_manual_slider_holds_the_angle_instead_of_snapping_back():
    look, clock = make_look(hold_max_s=5.0), Clock()

    look.hold(60, clock.t)
    angles = drive(look, clock, floor_frame(), 3.0)

    assert set(angles) == {60}


def test_manual_hold_is_clamped_to_the_safe_servo_range():
    look = make_look()

    look.hold(200, 1000.0)
    high = look.angle(1000.0)
    look.hold(-20, 1000.0)

    assert high == 150 and look.angle(1000.0) == 30


def test_manual_hold_returns_to_centre_when_it_times_out():
    look, clock = make_look(hold_max_s=2.0), Clock()
    look.hold(60, clock.t)

    angles = drive(look, clock, floor_frame(), 4.0)

    assert angles[0] == 60 and angles[-1] is None


def test_old_reports_expire_so_a_stale_warning_does_not_stick():
    look, clock = make_look(observe_max_s=1.0, report_ttl_s=5.0), Clock()
    look.request("left", clock.t)
    drive(look, clock, box(floor_frame(), 40, 130, 140, 215), 3.0)
    assert look.status(clock.t)["warning"]

    clock.step(10.0)

    status = look.status(clock.t)
    assert status["warning"] is None and status["sides"]["left"]["status"] == "unknown"
    assert status["advice"] == "UNKNOWN"


def test_status_is_json_serialisable():
    look, clock = make_look(), Clock()
    look.request("check", clock.t)
    drive(look, clock, box(floor_frame(), 40, 130, 140, 215), 1.0)

    json.dumps(look.status(clock.t))


def test_requests_from_other_threads_are_safe():
    look = make_look()
    stop = threading.Event()

    def spam():
        i = 0
        while not stop.is_set():
            look.request(("left", "right", "centre", "check")[i % 4], time.time())
            look.hold(60 + i % 40, time.time())
            i += 1

    worker = threading.Thread(target=spam)
    worker.start()
    try:
        frame = floor_frame()
        for _ in range(40):
            look.update(time.time(), frame, None, None)
            look.angle(time.time())
            look.status(time.time())
    finally:
        stop.set()
        worker.join()


# --- LineFollower integration ---------------------------------------------------

def make_follower(**kw):
    from control.line_follower import LineFollower

    follower = LineFollower(lane_layout="three_line", **kw)
    return follower


def blank():
    return floor_frame()


def test_follower_without_tag_search_never_sweeps_the_servo(monkeypatch):
    follower = make_follower(servo_search=False)
    follower.aruco_navigator.set_target_tag(7)  # a tag that is never visible
    monkeypatch.setattr(follower.servo_scanner, "step_search",
                        lambda now: pytest.fail("tag-search sweep ran"))
    follower.start()

    servos = [follower.process_frame(blank(), 200.0, time.time())[1] for _ in range(6)]

    assert set(servos) == {90}


def test_follower_by_default_still_sweeps_for_a_missing_tag(monkeypatch):
    follower = make_follower()
    follower.aruco_navigator.set_target_tag(7)
    called = []
    monkeypatch.setattr(follower.servo_scanner, "step_search",
                        lambda now: called.append(1) or 90)
    follower.start()

    follower.process_frame(blank(), 200.0, time.time())

    assert called


def test_look_command_moves_the_servo_even_when_the_car_is_idle():
    follower = make_follower(servo_search=False)

    follower.side_look.request("left", time.time())
    _cmd, servo, _hud, telem = follower.process_frame(blank(), 200.0, time.time())

    assert servo == follower.side_look.left_angle
    assert telem["side_look"]["mode"] != "idle"


def test_a_driving_car_stops_while_the_camera_is_turned_to_the_side():
    follower = make_follower(servo_search=False)
    follower.start()
    follower.side_look.request("right", time.time())

    cmd, servo, _hud, _telem = follower.process_frame(blank(), 200.0, time.time())

    assert cmd == "STOP"
    assert servo == follower.side_look.right_angle


def _lane_frame(follower):
    """A frame with a real two-lane tape pattern, drawn through the detector's own geometry."""
    det = follower.lane_detector
    bev = np.zeros((240, 400, 3), np.uint8)
    c = det.bev_center_x
    for x0, dashed in ((c - 50, False), (c + 50, True)):
        for y in range(240):
            if dashed and (y // 12) % 2:
                continue
            bev[y, x0 - 3:x0 + 4] = 255
    return cv2.warpPerspective(bev, det.Minv, (400, 240))


def _spy_on_lane_detector(follower, monkeypatch):
    calls = []
    original = follower.lane_detector.detect_from_bgr

    def spy(frame):
        calls.append(1)
        return original(frame)

    monkeypatch.setattr(follower.lane_detector, "detect_from_bgr", spy)
    return calls


def test_lane_detection_runs_normally_when_the_camera_is_straight(monkeypatch):
    follower = make_follower(servo_search=False)
    calls = _spy_on_lane_detector(follower, monkeypatch)

    for _ in range(3):
        follower.process_frame(_lane_frame(follower), 200.0, time.time())

    assert len(calls) == 3


@pytest.mark.parametrize("how", ["look_left", "look_right", "check", "slider"])
def test_lane_detection_is_paused_while_the_camera_is_turned(monkeypatch, how):
    follower = make_follower(servo_search=False)
    calls = _spy_on_lane_detector(follower, monkeypatch)
    now = time.time()
    if how == "slider":
        follower.side_look.hold(60, now)
    else:
        follower.side_look.request(how.replace("look_", ""), now)

    telems = [follower.process_frame(_lane_frame(follower), 200.0, time.time())[3] for _ in range(3)]

    assert calls == []
    assert all(t["direction"] == "NO LANE" and t["error_px"] == 0.0 for t in telems)


def _hugging_outer_line_frame(follower):
    """Car close to the solid outer tape, which makes the lane tracker draw its warning."""
    det = follower.lane_detector
    bev = np.zeros((240, 400, 3), np.uint8)
    c = det.bev_center_x
    for x0, dashed in ((c - 20, False), (c + 80, True)):
        for y in range(240):
            if dashed and (y // 12) % 2:
                continue
            bev[y, x0 - 3:x0 + 4] = 255
    return cv2.warpPerspective(bev, det.Minv, (400, 240))


def _red_banner_pixels(hud):
    region = hud[6:32, 8:280]   # where the lane tracker draws its OUTER LINE warning
    return int(((region[..., 2] > 200) & (region[..., 0] < 60) & (region[..., 1] < 60)).sum())


def test_outer_line_warning_is_drawn_on_a_straight_view_but_not_on_a_side_view():
    straight = make_follower(servo_search=False)
    for _ in range(3):
        _c, _s, hud_straight, _t = straight.process_frame(
            _hugging_outer_line_frame(straight), 200.0, time.time())
    turned = make_follower(servo_search=False)
    turned.side_look.request("right", time.time())
    _c, _s, hud_turned, _t = turned.process_frame(
        _hugging_outer_line_frame(turned), 200.0, time.time())

    assert _red_banner_pixels(hud_straight) > 200      # control: the banner really is drawn
    assert _red_banner_pixels(hud_turned) == 0


def test_lane_detection_resumes_once_the_camera_is_back_at_centre(monkeypatch):
    follower = make_follower(servo_search=False)
    calls = _spy_on_lane_detector(follower, monkeypatch)
    follower.side_look.request("centre", time.time())  # a quick home move, then idle
    deadline = time.time() + 3.0
    while follower.side_look.active and time.time() < deadline:
        follower.process_frame(_lane_frame(follower), 200.0, time.time())
        time.sleep(0.05)
    paused_calls = len(calls)

    for _ in range(3):
        follower.process_frame(_lane_frame(follower), 200.0, time.time())

    assert not follower.side_look.active
    assert paused_calls == 0
    assert len(calls) == 3


def test_a_lane_seen_before_the_look_does_not_leak_into_the_paused_view():
    follower = make_follower(servo_search=False)
    for _ in range(3):
        _c, _s, _h, before = follower.process_frame(_lane_frame(follower), 200.0, time.time())
    assert before["direction"] != "NO LANE"

    follower.side_look.request("left", time.time())
    _c, _s, _h, during = follower.process_frame(_lane_frame(follower), 200.0, time.time())

    assert during["direction"] == "NO LANE" and during["error_px"] == 0.0


def test_the_floor_model_is_not_trained_on_a_side_view():
    follower = make_follower(servo_search=False)
    for _ in range(3):
        follower.process_frame(blank(), 200.0, time.time())
    learned = follower._running_floor_mean.copy()

    follower.side_look.request("left", time.time())
    wall = box(blank(), 0, 90, 399, 239)  # a wall fills the view
    for _ in range(4):
        follower.process_frame(wall, 200.0, time.time())

    assert np.allclose(follower._running_floor_mean, learned)


def test_side_obstacle_warning_appears_in_the_follower_telemetry_and_hud():
    follower = make_follower(servo_search=False)
    for _ in range(3):
        follower.process_frame(blank(), 200.0, time.time())
    follower.side_look.request("left", time.time())
    wall = box(blank(), 20, 120, 220, 235)
    deadline = time.time() + 3.0
    telem = hud = None
    while time.time() < deadline:
        _c, _s, hud, telem = follower.process_frame(wall, 200.0, time.time())
        if telem["side_look"]["warning"]:
            break
        time.sleep(0.05)

    assert telem["side_look"]["warning"] and "LEFT" in telem["side_look"]["warning"]
    assert hud.shape == (240, 400, 3)


# --- the dashboard service --------------------------------------------------------

@pytest.fixture()
def service(monkeypatch):
    """The real service script with a live HTTP server on a free port; no Arduino."""
    import importlib.util

    monkeypatch.chdir(AUTONOMOUS_DIR)
    spec = importlib.util.spec_from_file_location(
        "service_side_look", os.path.join(AUTONOMOUS_DIR, "scripts", "test_dashboard.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sent = []
    monkeypatch.setattr(module, "send_cmd", lambda c: sent.append(c))
    monkeypatch.setattr(module, "_trigger_reconnect", lambda: None)
    server = module.ThreadedHTTPServer(("127.0.0.1", 0), module.DashboardHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    module.base_url = f"http://127.0.0.1:{server.server_address[1]}"
    module.sent = sent
    yield module
    server.shutdown()
    server.server_close()


def get(service, path):
    with urllib.request.urlopen(service.base_url + path, timeout=5) as resp:
        return json.loads(resp.read().decode())


def test_look_endpoint_starts_a_left_look(service):
    reply = get(service, "/look?dir=left")

    assert reply["ok"] is True
    assert get(service, "/status")["side_look"]["mode"] != "idle"


def test_look_endpoint_rejects_nonsense(service):
    assert get(service, "/look?dir=sideways")["ok"] is False


def test_look_endpoint_refused_in_standby(service):
    service.standby_mode = True

    assert get(service, "/look?dir=left")["ok"] is False
    service.standby_mode = False


def test_status_reports_side_look_and_the_servo_the_arduino_confirmed(service):
    status = get(service, "/status")

    assert "side_look" in status and "advice" in status["side_look"]
    assert "servo_acked" in status


def test_slider_command_is_held_by_the_follower_not_sent_around_it(service):
    get(service, "/cmd?c=SERVO:70")

    assert "SERVO:70" not in service.sent          # the camera loop sends it, once, in order
    assert service.line_follower.side_look.angle(time.time()) == 70


def test_dashboard_page_has_the_look_buttons_and_the_warning_chip(service):
    with urllib.request.urlopen(service.base_url + "/", timeout=5) as resp:
        page = resp.read().decode("utf-8")

    for needle in ("lookSide('left')", "lookSide('check')", "lookSide('right')",
                   'id="side-look-chip"', "fetch('/look?dir='"):
        assert needle in page


def test_look_angles_match_the_dashboards_own_left_and_right_presets(service):
    look = service.line_follower.side_look

    assert (look.left_angle, look.right_angle) == (130, 50)


def test_the_service_runs_the_follower_without_tag_search_sweeps(service):
    assert service.line_follower.servo_search is False


def test_arduino_servo_acknowledgement_is_recorded(service):
    service._note_ack("ACK:SERVO:123")

    assert service.servo_acked == 123 and service.last_ack == "ACK:SERVO:123"
    service._note_ack("ACK:STOP")
    assert service.servo_acked == 123 and service.last_ack == "ACK:STOP"


# --- /cmd only passes commands the car understands, and the reflex stop knows slides ---------

@pytest.mark.parametrize("command", ["FORWARD", "BACKWARD", "LEFT", "RIGHT", "SLIDE_LEFT",
                                     "SLIDE_RIGHT", "HARD_LEFT", "HARD_RIGHT", "STOP"])
def test_cmd_passes_every_drive_command_the_dashboard_uses(service, command):
    reply = get(service, f"/cmd?c={command}")

    assert reply["ok"] is True
    assert service.sent == [command]


@pytest.mark.parametrize("command", ["FOO", "AUTO:1", "AUTO:0", "forward", "FORWARD;STOP", "SERVO", "x" * 40])
def test_cmd_refuses_anything_else_so_it_never_reaches_the_arduino(service, command):
    reply = get(service, "/cmd?c=" + urllib.parse.quote(command))

    assert reply["ok"] is False
    assert service.sent == []


def test_a_bad_servo_angle_is_refused_and_nothing_is_sent(service):
    assert get(service, "/cmd?c=SERVO:abc")["ok"] is False
    assert service.sent == []


@pytest.mark.parametrize("moving", ["FORWARD", "SLIDE_LEFT", "SLIDE_RIGHT"])
def test_reflex_stop_fires_for_every_forward_moving_command_in_auto(service, moving):
    service.auto_running = True
    service.last_motion_cmd = moving

    service._on_dist(10)

    assert service.sent == ["STOP"]


@pytest.mark.parametrize("command", ["LEFT", "RIGHT", "BACKWARD", "HARD_LEFT", "STOP"])
def test_reflex_stop_does_not_fire_when_not_driving_toward_the_obstacle(service, command):
    service.auto_running = True
    service.last_motion_cmd = command

    service._on_dist(10)

    assert service.sent == []


def test_reflex_stop_is_for_autonomous_mode_only(service):
    service.auto_running = False
    service.last_motion_cmd = "FORWARD"

    service._on_dist(10)

    assert service.sent == []
