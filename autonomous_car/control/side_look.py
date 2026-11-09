"""
control/side_look.py - fixed camera positions and a side obstacle warning.

The camera servo has three jobs and no others: sit at centre, hold an angle the
operator picked on the slider, or turn to a fixed left/right angle on command so
the car can check what is beside it. There is no sweep that hunts for a path.

One object owns the servo. The follower asks ``angle()`` every frame; the
dashboard only calls ``request()`` / ``hold()``. Nothing else sends servo
commands, so the slider and the camera loop can no longer fight each other.

When the camera is turned to a side, ``update()`` looks for an obstacle there
and follows it (``ObstacleWatch``). While an obstacle stays it keeps the camera
on it, up to ``observe_max_s``; when it leaves, the warning clears and the
camera returns to centre. ``status()`` reports a warning and an advice
(``HOLD`` / ``CLEAR_TO_MOVE`` / ``UNKNOWN``). It only advises; it never drives.
"""

import threading
from typing import Any, Dict, List, Optional

import numpy as np

try:
    from vision.side_obstacle import BLOCKED, ObstacleWatch, find_side_obstacle
except ImportError:
    from autonomous_car.vision.side_obstacle import BLOCKED, ObstacleWatch, find_side_obstacle

SERVO_MIN, SERVO_MAX = 30, 150      # the dashboard slider range; beyond it the mount may stall


class SideLook:
    """Owns the camera servo: centre, manual hold, or a fixed look left / right."""

    def __init__(
        self,
        left_angle: int = 130,     # same as the dashboard's "130 Left" preset
        right_angle: int = 50,     # same as the dashboard's "50 Right" preset
        centre_angle: int = 90,
        settle_s: float = 0.6,
        sample_s: float = 1.2,
        observe_max_s: float = 10.0,
        hold_max_s: float = 20.0,
        report_ttl_s: float = 30.0,
        clear_after_s: float = 1.0,
    ) -> None:
        self.left_angle = int(left_angle)
        self.right_angle = int(right_angle)
        self.centre_angle = int(centre_angle)
        self.settle_s = settle_s
        self.sample_s = sample_s
        self.observe_max_s = observe_max_s
        self.hold_max_s = hold_max_s
        self.report_ttl_s = report_ttl_s
        self.clear_after_s = clear_after_s
        self._lock = threading.RLock()
        self._mode = "idle"            # idle | look | check | return | manual
        self._side: Optional[str] = None
        self._queue: List[str] = []
        self._target = self.centre_angle
        self._t_target = 0.0
        self._watch = {s: ObstacleWatch(clear_after_s) for s in ("left", "right")}
        self._reports: Dict[str, Optional[Dict[str, Any]]] = {"left": None, "right": None}

    # ── commands (called from the dashboard thread) ─────────────────────────
    def request(self, direction: str, now: float) -> bool:
        direction = str(direction).strip().lower()
        with self._lock:
            if direction in ("centre", "center"):
                self._go_home(now)
            elif direction in ("left", "right"):
                self._start_side(direction, now, "look")
            elif direction == "check":
                self._queue = ["right"]
                self._start_side("left", now, "check")
            else:
                return False
            return True

    def hold(self, angle: int, now: float) -> None:
        """Manual slider: hold this angle (clamped) until ``hold_max_s`` or a centre command."""
        with self._lock:
            self._mode = "manual"
            self._side = None
            self._queue = []
            self._target = int(max(SERVO_MIN, min(SERVO_MAX, angle)))
            self._t_target = now

    # ── what the follower asks every frame ──────────────────────────────────
    @property
    def active(self) -> bool:
        with self._lock:
            return self._mode != "idle"

    def angle(self, now: float) -> Optional[int]:
        """Angle to command, or None when the servo has no special job (stay at centre)."""
        with self._lock:
            return None if self._mode == "idle" else self._target

    def update(self, now: float, frame: Optional[np.ndarray],
               floor_mean: Optional[np.ndarray], floor_std: Optional[np.ndarray]) -> None:
        with self._lock:
            if self._mode == "manual":
                if now - self._t_target >= self.hold_max_s:
                    self._go_home(now)
            elif self._mode == "return":
                if now - self._t_target >= self.settle_s:
                    self._mode = "idle"
            elif self._mode in ("look", "check") and self._side:
                self._look_step(now, frame, floor_mean, floor_std)

    # ── internals (lock held) ───────────────────────────────────────────────
    def _go_home(self, now: float) -> None:
        self._mode = "return"
        self._side = None
        self._queue = []
        self._target = self.centre_angle
        self._t_target = now

    def _start_side(self, side: str, now: float, mode: str) -> None:
        self._mode = mode
        self._side = side
        self._target = self.left_angle if side == "left" else self.right_angle
        self._t_target = now
        self._watch[side] = ObstacleWatch(self.clear_after_s)

    def _look_step(self, now, frame, floor_mean, floor_std) -> None:
        side = self._side
        elapsed = now - self._t_target
        if elapsed < self.settle_s:
            return                                  # the camera is still moving
        if frame is not None:
            obs = find_side_obstacle(frame, floor_mean, floor_std)
            seen = self._watch[side].update(now, obs)
            self._reports[side] = {
                "status": seen.status, "present": obs.present,
                "area": round(obs.area_frac, 3), "cx": round(obs.cx, 2), "base": round(obs.base, 2),
                "seen_s": round(seen.seen_s, 1), "closing_per_s": round(seen.closing_per_s, 3),
                "low_confidence": obs.from_band, "t": now,
            }
        report = self._reports[side]
        blocked = bool(report and report["status"] in BLOCKED)
        sampled = elapsed >= self.settle_s + self.sample_s
        if self._mode == "check":
            if sampled:
                if self._queue:
                    self._start_side(self._queue.pop(0), now, "check")
                else:
                    self._go_home(now)
        elif elapsed >= self.observe_max_s or (sampled and not blocked):
            self._go_home(now)

    # ── report ──────────────────────────────────────────────────────────────
    def status(self, now: float) -> Dict[str, Any]:
        with self._lock:
            sides: Dict[str, Dict[str, Any]] = {}
            for side, report in self._reports.items():
                if report is None or now - report["t"] > self.report_ttl_s:
                    sides[side] = {"status": "unknown"}
                else:
                    sides[side] = {**{k: v for k, v in report.items() if k != "t"},
                                   "age_s": round(now - report["t"], 1)}
            blocked = [s for s, v in sides.items() if v["status"] in BLOCKED]
            trusted_clear = [s for s, v in sides.items()
                             if v["status"] == "clear" and not v.get("low_confidence")]
            warning = None
            if blocked:
                detail = f" ({sides[blocked[0]]['status']})" if len(blocked) == 1 else ""
                warning = "OBSTACLE ON " + " AND ".join(s.upper() for s in blocked) + detail
            advice = "HOLD" if blocked else ("CLEAR_TO_MOVE" if trusted_clear else "UNKNOWN")
            return {
                "mode": self._mode,
                "angle": None if self._mode == "idle" else self._target,
                "active": self._mode != "idle",
                "sides": sides,
                "warning": warning,
                "advice": advice,
            }
