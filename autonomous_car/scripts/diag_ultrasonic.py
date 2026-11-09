"""HC-SR04 wiring diagnostic for TRIG=BCM 24, ECHO=BCM 25.

Run ON the Pi with rccar.service stopped (the service holds the GPIO pins):

    ./venv/bin/python3 scripts/diag_ultrasonic.py [--hold-seconds 12]

Phase A runs immediately with nothing special in front of the sensor.
Phase B starts after --hold-seconds: hold a flat object (a book) about 20 cm
in front of the sensor before then.

What the numbers mean:
* idle test  - ECHO is read with the internal pull-down, then pull-up. A pin
  that is actually driven by the sensor stays LOW in both. A pin that follows
  the pull is floating: sensor unpowered or ECHO wire not connected.
* shots      - each TRIG pulse is classified:
    no_rise    ECHO never went high (no sensor reply, or high level below the
               Pi's ~1.8 V threshold because of a wrong voltage divider)
    max_range  ECHO high for >= 30 ms: the sensor works but heard no echo
               (nothing within range, or the surface absorbs/deflects sound)
    stuck_high ECHO still high at the end of the window
    ok         pulse width converted to centimetres
"""

import argparse
import subprocess
import time

import RPi.GPIO as GPIO

TRIG = 24
ECHO = 25
WINDOW_SEC = 0.045
SPEED_CM_PER_SEC = 34300.0


def idle_high_fraction(pull: int, samples: int = 300) -> float:
    GPIO.setup(ECHO, GPIO.IN, pull_up_down=pull)
    time.sleep(0.05)
    return sum(GPIO.input(ECHO) for _ in range(samples)) / samples


def shot() -> tuple:
    """One TRIG pulse. Returns (high_before_trigger, rise_s, width_s)."""
    high_before = bool(GPIO.input(ECHO))
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start = time.perf_counter()
    rise = None
    while True:
        now = time.perf_counter()
        if now - start > WINDOW_SEC:
            return high_before, (rise - start) if rise else None, None
        level = GPIO.input(ECHO)
        if rise is None:
            if level:
                rise = now
        elif not level:
            return high_before, rise - start, now - rise


def run_phase(label: str, count: int) -> None:
    counts = {"no_rise": 0, "max_range": 0, "stuck_high": 0, "ok": 0}
    distances = []
    rises_ms = []
    high_before_trigger = 0
    for _ in range(count):
        high_before, rise, width = shot()
        high_before_trigger += high_before
        if rise is None:
            counts["no_rise"] += 1
        else:
            rises_ms.append(rise * 1000.0)
            if width is None:
                counts["stuck_high"] += 1
            elif width >= 0.030:
                counts["max_range"] += 1
            else:
                counts["ok"] += 1
                distances.append(width * SPEED_CM_PER_SEC / 2.0)
        time.sleep(0.06)
    print(f"[{label}] {count} shots: {counts}")
    print(f"[{label}] ECHO already high before trigger: {high_before_trigger}/{count}")
    if rises_ms:
        print(f"[{label}] echo rise delay: {min(rises_ms):.2f}-{max(rises_ms):.2f} ms after trigger")
    if distances:
        distances.sort()
        print(f"[{label}] distance cm: min {distances[0]:.1f}  median "
              f"{distances[len(distances) // 2]:.1f}  max {distances[-1]:.1f}")


def main() -> None:
    global TRIG, ECHO
    parser = argparse.ArgumentParser()
    parser.add_argument("--hold-seconds", type=float, default=12.0)
    parser.add_argument("--shots", type=int, default=25)
    parser.add_argument("--trig", type=int, default=TRIG)
    parser.add_argument("--echo", type=int, default=ECHO, help="swap --trig/--echo to test reversed wiring")
    args = parser.parse_args()
    TRIG, ECHO = args.trig, args.echo
    print(f"[pins] TRIG=BCM{TRIG} ECHO=BCM{ECHO}")

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(TRIG, GPIO.OUT, initial=False)

    low_pull = idle_high_fraction(GPIO.PUD_DOWN)
    high_pull = idle_high_fraction(GPIO.PUD_UP)
    print(f"[idle] ECHO high with pull-down: {low_pull:.0%}   with pull-up: {high_pull:.0%}")
    if low_pull < 0.05 and high_pull > 0.95:
        print("[idle] -> ECHO follows the pull: NOT DRIVEN (sensor unpowered or ECHO wire loose)")
    elif low_pull > 0.5:
        print("[idle] -> ECHO stuck HIGH even with pull-down (short to 3.3V/5V, or wrong pin)")
    elif low_pull < 0.05 and high_pull < 0.05:
        print("[idle] -> ECHO actively driven low: a powered sensor is connected")
    else:
        print("[idle] -> ECHO unstable/noisy: loose wire or interference")
    GPIO.setup(ECHO, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

    run_phase("A: nothing placed", args.shots)
    print(f"... hold an object ~20 cm in front of the sensor now ({args.hold_seconds:.0f} s) ...", flush=True)
    time.sleep(args.hold_seconds)
    run_phase("B: object at ~20 cm", args.shots)

    try:
        out = subprocess.run(["pinctrl", "get", "24,25"], capture_output=True, text=True, timeout=5)
        print("[pinctrl]", (out.stdout or out.stderr).strip().replace("\n", " | "))
    except (OSError, subprocess.SubprocessError):
        pass
    GPIO.cleanup([TRIG, ECHO])


if __name__ == "__main__":
    main()
