"""The dashboard must never drive the car when the operator is working the camera servo.

Real-browser tests (Chrome via Playwright) against the real dashboard page. A fake
car sits on the wire and records every command the page sends, so each test states
exactly what may and may not reach the motors.

Bug that prompted these: a page-wide key handler turned the arrow keys into driving
commands and cancelled the key. A range slider is adjusted with those same keys, so
nudging the camera slider with the arrows pivoted the car and left the servo alone.
"""

import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

pytest.importorskip("playwright.sync_api")
from playwright.sync_api import sync_playwright  # noqa: E402

from test_side_look import service  # noqa: E402,F401  (fixture: the real service, fake car)

DRIVE = {"FORWARD", "BACKWARD", "LEFT", "RIGHT", "HARD_LEFT", "HARD_RIGHT", "SLIDE_LEFT", "SLIDE_RIGHT"}


@pytest.fixture(scope="module")
def browser():
    with sync_playwright() as p:
        try:
            chrome = p.chromium.launch(channel="chrome", headless=True)
        except Exception as exc:  # pragma: no cover - no Chrome on this machine
            pytest.skip(f"Chrome not available for browser tests: {exc}")
        yield chrome
        chrome.close()


@pytest.fixture()
def page(browser, service):
    pg = browser.new_page(viewport={"width": 1280, "height": 2200})   # tall: every control is on screen
    # The live camera feed never finishes loading (it is an endless MJPEG stream), so
    # the browser's "load" event never fires. The controls do not need it.
    pg.route("**/stream*", lambda route: route.abort())
    pg.goto(service.base_url + "/", wait_until="domcontentloaded")
    pg.wait_for_selector("#servo-slider")
    pg.wait_for_timeout(400)          # let the first status poll settle
    service.sent.clear()
    yield pg
    pg.close()


def settle(page, ms=300):
    page.wait_for_timeout(ms)         # the slider sends after an 80 ms debounce


# --- the bug -------------------------------------------------------------------

@pytest.mark.parametrize("key", ["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown"])
def test_arrow_keys_on_the_camera_slider_never_drive_the_car(page, service, key):
    page.focus("#servo-slider")

    for _ in range(3):
        page.keyboard.press(key)
    settle(page)

    settle(page)
    assert service.sent == [] or all(c.startswith("SERVO") for c in service.sent), service.sent
    settle(page)
    assert not set(service.sent) & (DRIVE | {"STOP"})


def test_arrow_keys_on_the_slider_move_the_servo_instead(page, service):
    page.focus("#servo-slider")

    for _ in range(3):
        page.keyboard.press("ArrowLeft")
    settle(page)

    assert page.input_value("#servo-slider") == "87"
    assert service.line_follower.side_look.angle(time.time()) == 87


def test_right_arrow_on_the_slider_raises_the_angle(page, service):
    page.focus("#servo-slider")
    for _ in range(5):
        page.keyboard.press("ArrowRight")
    settle(page)

    assert page.input_value("#servo-slider") == "95"
    assert service.line_follower.side_look.angle(time.time()) == 95


def test_typing_in_the_route_box_does_not_drive_or_stop_the_car(page, service):
    page.fill("#route-input-list", "")
    page.focus("#route-input-list")

    page.keyboard.type("w a s d x 1, 2")

    settle(page)
    assert service.sent == []
    assert page.input_value("#route-input-list") == "w a s d x 1, 2"   # spaces are typed, not swallowed


# --- driving must keep working ---------------------------------------------------

def test_wasd_still_drives_and_stops_when_nothing_is_focused(page, service):
    page.keyboard.press("w")
    page.keyboard.press("a")

    settle(page)
    assert service.sent == ["FORWARD", "STOP", "LEFT", "STOP"]


def test_arrow_keys_still_drive_when_a_normal_button_has_focus(page, service):
    page.focus("#btn-w")
    page.keyboard.press("ArrowLeft")

    settle(page)
    assert service.sent == ["LEFT", "STOP"]


def test_wasd_letters_still_drive_even_with_the_slider_focused(page, service):
    page.focus("#servo-slider")
    page.keyboard.press("a")

    settle(page)
    assert service.sent == ["LEFT", "STOP"]       # only the arrow keys belong to the slider


def test_space_still_stops_the_car(page, service):
    page.keyboard.press("Space")

    settle(page)
    assert "STOP" in service.sent


# --- the car must not be left moving by a lost release ---------------------------

def test_releasing_the_mouse_away_from_the_drive_button_still_stops_the_car(page, service):
    box = page.locator("#btn-w").bounding_box()
    page.mouse.move(box["x"] + 10, box["y"] + 10)
    page.mouse.down()
    page.mouse.move(5, 5)                           # slide off the button
    page.mouse.up()                                 # released over nothing

    settle(page)
    assert service.sent == ["FORWARD", "STOP"]


def test_a_normal_click_on_a_drive_button_sends_one_stop_not_two(page, service):
    box = page.locator("#btn-w").bounding_box()
    page.mouse.move(box["x"] + 10, box["y"] + 10)
    page.mouse.down()
    page.mouse.up()

    settle(page)
    assert service.sent == ["FORWARD", "STOP"]


def test_a_cancelled_touch_on_a_drive_button_stops_the_car(page, service):
    page.dispatch_event("#btn-w", "touchstart")
    page.dispatch_event("#btn-w", "touchcancel")

    settle(page)
    assert service.sent == ["FORWARD", "STOP"]


def test_losing_window_focus_while_a_key_is_held_stops_the_car(page, service):
    page.keyboard.down("w")
    page.evaluate("window.dispatchEvent(new Event('blur'))")   # the key-up now never arrives
    settle(page)

    assert service.sent == ["FORWARD", "STOP"]       # the STOP came from losing focus, not from a key-up

    page.keyboard.up("w")                            # a late key-up must not add a second STOP
    settle(page)
    assert service.sent == ["FORWARD", "STOP"]


def test_hiding_the_tab_while_driving_stops_the_car(page, service):
    page.keyboard.down("s")
    page.evaluate("Object.defineProperty(document, 'hidden', {value: true, configurable: true});"
                  "document.dispatchEvent(new Event('visibilitychange'))")

    settle(page)
    assert service.sent == ["BACKWARD", "STOP"]


def test_blur_with_nothing_pressed_sends_nothing(page, service):
    page.evaluate("window.dispatchEvent(new Event('blur'))")

    settle(page)
    assert service.sent == []


# --- servo and look controls never reach the motors --------------------------------

@pytest.mark.parametrize("selector", [
    "text=50° Right", "text=90° Center", "text=130° Left",
    "text=◀ Look left", "text=Look right ▶", "text=↔ Check sides",
])
def test_servo_and_look_buttons_never_send_a_drive_command(page, service, selector):
    page.click(selector)
    settle(page, 500)

    settle(page)
    assert not set(service.sent) & (DRIVE | {"STOP"}), service.sent
    assert service.line_follower.side_look.active


# --- steer while moving: forward + left/right = a gentle arc, not an in-place spin --------

def held(page, service, downs, ups):
    for key in downs:
        page.keyboard.down(key)
    for key in ups:
        page.keyboard.up(key)
    settle(page)
    return service.sent


def test_forward_then_left_arcs_left_and_straightens_when_left_is_released(page, service):
    sent = held(page, service, ["w", "a"], ["a", "w"])

    assert sent == ["FORWARD", "SLIDE_LEFT", "FORWARD", "STOP"]


def test_forward_then_right_arcs_right_and_straightens_when_right_is_released(page, service):
    sent = held(page, service, ["w", "d"], ["d", "w"])

    assert sent == ["FORWARD", "SLIDE_RIGHT", "FORWARD", "STOP"]


def test_arrow_keys_arc_too(page, service):
    sent = held(page, service, ["ArrowUp", "ArrowLeft"], ["ArrowLeft", "ArrowUp"])

    assert sent == ["FORWARD", "SLIDE_LEFT", "FORWARD", "STOP"]


def test_left_pressed_before_forward_still_arcs_once_forward_joins(page, service):
    sent = held(page, service, ["a", "w"], ["w", "a"])

    assert sent == ["LEFT", "SLIDE_LEFT", "LEFT", "STOP"]


def test_left_and_right_together_while_forward_cancel_out_and_go_straight(page, service):
    sent = held(page, service, ["w", "a", "d"], [])

    assert sent == ["FORWARD", "SLIDE_LEFT", "FORWARD"]


def test_left_or_right_alone_still_pivots_in_place(page, service):
    assert held(page, service, ["a"], ["a"]) == ["LEFT", "STOP"]
    service.sent.clear()
    assert held(page, service, ["d"], ["d"]) == ["RIGHT", "STOP"]


def test_reversing_has_no_arc_so_steering_keys_change_nothing(page, service):
    sent = held(page, service, ["s", "a", "d"], ["d", "a", "s"])

    assert sent == ["BACKWARD", "STOP"]


def test_forward_and_back_together_stop_and_forward_resumes_when_back_is_released(page, service):
    sent = held(page, service, ["w", "s"], ["s"])

    assert sent == ["FORWARD", "STOP", "FORWARD"]


def test_the_on_screen_slide_buttons_still_work(page, service):
    box = page.locator("text=↙ Slide Left").bounding_box()
    page.mouse.move(box["x"] + 10, box["y"] + 10)
    page.mouse.down()
    page.mouse.up()
    settle(page)

    assert service.sent == ["SLIDE_LEFT", "STOP"]


# --- the slider and the status poll must not fight each other ----------------------------

def test_the_slider_shows_the_angle_a_look_is_commanding(page, service):
    page.click("text=◀ Look left")
    page.wait_for_timeout(1200)                      # a couple of status polls

    assert page.input_value("#servo-slider") == "130"


def test_the_slider_is_not_snapped_back_while_the_operator_is_using_it(page, service):
    page.focus("#servo-slider")
    for _ in range(4):
        page.keyboard.press("ArrowRight")
        page.wait_for_timeout(150)                   # slower than a burst: several polls land meanwhile

    assert page.input_value("#servo-slider") == "94"
