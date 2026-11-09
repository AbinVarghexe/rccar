"""
control/obstacle_avoider.py — Sonar-driven reactive obstacle avoidance.

The HC-SR04 is fixed to the chassis and only sees straight ahead, so the *body*
is the scanner: when something is in the way the car brakes, pivots in short
pulses toward the more open side until the sonar reads clear, then bypasses.

    CLEAR / CAUTION --(d <= stop)--> BRAKE --> TURN --(clear)--> BYPASS --> CLEAR
                                               |  timeout: flip side, then BACKUP,
                                               |  then give up --> STUCK (latched stop)

Safety properties:
  * Stale or missing sonar data means STOP (SENSOR_FAULT), never "assume clear".
  * Every state is time-bounded, so the car cannot spin or freeze forever.
  * Repeated brakes inside a window latch STUCK instead of looping.

Pure Python with no OpenCV/serial dependencies so it can be unit-tested offline.
"""

from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Deque, List, Optional, Tuple

MIN_VALID_CM = 2.0      # HC-SR04 blind zone; smaller readings are noise
MAX_VALID_CM = 400.0    # firmware reports 400 on echo timeout (nothing in range)


def trusted_reading(
    dist_cm: Optional[float], dist_ts: Optional[float]
) -> Tuple[Optional[float], Optional[float]]:
    """Only pass a sonar distance on when it carries its read timestamp.

    ``ObstacleAvoider`` stamps a distance that has no timestamp with "now", so
    a bare number looks freshly measured on every call and can never go stale.
    A genuine DIST line always comes with a timestamp; anything without one
    (a placeholder, a simulation, a value left over after a reset) must read
    as "no sonar data", which the avoider turns into SENSOR_FAULT / STOP.
    """
    if dist_cm is None or dist_ts is None:
        return None, None
    return dist_cm, dist_ts


class AvoidState(str, Enum):
    CLEAR = "CLEAR"
    CAUTION = "CAUTION"
    BRAKE = "BRAKE"
    TURN = "TURN"
    BACKUP = "BACKUP"
    BYPASS = "BYPASS"
    STUCK = "STUCK"
    SENSOR_FAULT = "SENSOR_FAULT"


# Compact integer for telemetry/dashboards (0 = not avoiding).
_PHASE = {
    AvoidState.CLEAR: 0, AvoidState.CAUTION: 0, AvoidState.BRAKE: 1,
    AvoidState.TURN: 2, AvoidState.BACKUP: 3, AvoidState.BYPASS: 4,
    AvoidState.STUCK: 5, AvoidState.SENSOR_FAULT: 6,
}


@dataclass(frozen=True)
class AvoidConfig:
    stop_cm: float = 35.0            # brake when the filtered range drops to this
    caution_cm: float = 70.0         # creep (pulsed forward) inside this range
    clear_cm: float = 60.0           # a turn ends when the range stays above this
    sensor_max_age_s: float = 0.5    # older readings are not trusted
    brake_s: float = 0.30            # settle time before choosing a side
    turn_on_s: float = 0.10          # pivot pulse length (full-speed motors)
    turn_off_s: float = 0.14         # pause so the sonar reads a stationary scene
    turn_timeout_s: float = 3.0      # per direction; the flipped direction gets 2x
    backup_s: float = 0.5            # short blind reverse (no rear sensor)
    bypass_s: float = 1.0            # drive on after the turn to get past the obstacle
    creep_on_s: float = 0.12
    creep_off_s: float = 0.10
    clear_readings: int = 3          # consecutive fresh clear readings to end a turn
    max_attempts: int = 3            # brakes allowed inside attempt_window_s
    attempt_window_s: float = 8.0
    side_margin: float = 0.08        # clearance difference below which sides are "equal"
    preferred_side_s: float = 10.0   # keep the last side for this long when undecided


@dataclass(frozen=True)
class AvoidDecision:
    state: AvoidState
    cmd: Optional[str]               # None -> navigation keeps control
    note: str
    gate_forward: bool = True        # False -> hold FORWARD this instant (creep)

    @property
    def phase(self) -> int:
        return _PHASE[self.state]


class ObstacleAvoider:
    def __init__(self, config: Optional[AvoidConfig] = None) -> None:
        self.cfg = config or AvoidConfig()
        self._readings: Deque[float] = deque(maxlen=3)
        self.reset()

    def reset(self) -> None:
        self._readings.clear()
        self._state = AvoidState.CLEAR
        self._t0 = 0.0
        self._last_ts: Optional[float] = None
        self._clear_count = 0
        self._dir = "RIGHT"
        self._flipped = False
        self._backed_up = False
        self._attempts: List[float] = []
        self._last_dir: Optional[str] = None
        self._last_dir_time = -1e9

    @property
    def state(self) -> AvoidState:
        return self._state

    @property
    def is_avoiding(self) -> bool:
        return self._state not in (AvoidState.CLEAR, AvoidState.CAUTION)

    # ── sensor handling ──────────────────────────────────────────────────────
    def _ingest(self, now: float, dist_cm: Optional[float], dist_ts: Optional[float]) -> None:
        if dist_cm is None or dist_cm < MIN_VALID_CM:
            return
        ts = now if dist_ts is None else dist_ts
        if self._last_ts is not None and ts == self._last_ts:
            return  # same reading seen again; don't double count
        self._last_ts = ts
        value = min(float(dist_cm), MAX_VALID_CM)
        self._readings.append(value)
        self._clear_count = self._clear_count + 1 if value >= self.cfg.clear_cm else 0

    def _filtered(self) -> float:
        # Median of 3 rejects single-echo spikes; with fewer samples stay conservative.
        vals = sorted(self._readings)
        return vals[len(vals) // 2] if len(vals) == 3 else vals[0]

    # ── state transitions ────────────────────────────────────────────────────
    def _begin_brake(self, now: float) -> None:
        cfg = self.cfg
        self._attempts = [t for t in self._attempts if now - t <= cfg.attempt_window_s]
        self._attempts.append(now)
        if len(self._attempts) > cfg.max_attempts:
            self._state = AvoidState.STUCK
            self._clear_count = 0
            return
        self._state = AvoidState.BRAKE
        self._t0 = now

    def _choose_side(self, now: float, left: float, right: float) -> str:
        diff = left - right
        if abs(diff) < self.cfg.side_margin:
            if self._last_dir and now - self._last_dir_time < self.cfg.preferred_side_s:
                return self._last_dir
            return "RIGHT"
        return "LEFT" if diff > 0 else "RIGHT"

    def _begin_turn(self, now: float, direction: str, flipped: bool) -> None:
        self._state = AvoidState.TURN
        self._t0 = now
        self._dir = direction
        self._flipped = flipped
        self._clear_count = 0
        self._last_dir = direction
        self._last_dir_time = now

    # ── main entry ───────────────────────────────────────────────────────────
    def update(
        self,
        now: float,
        dist_cm: Optional[float],
        dist_ts: Optional[float] = None,
        left_clearance: float = 0.5,
        right_clearance: float = 0.5,
    ) -> AvoidDecision:
        cfg = self.cfg
        self._ingest(now, dist_cm, dist_ts)

        age = float("inf") if self._last_ts is None else now - self._last_ts
        if not self._readings or age > cfg.sensor_max_age_s:
            return AvoidDecision(AvoidState.SENSOR_FAULT, "STOP", "SONAR STALE: HOLDING")

        d = self._filtered()
        st = self._state

        if st in (AvoidState.CLEAR, AvoidState.CAUTION):
            if d <= cfg.stop_cm:
                self._begin_brake(now)
                return self._stop_decision(f"OBSTACLE {d:.0f}cm: BRAKING")
            if d <= cfg.caution_cm:
                self._state = AvoidState.CAUTION
                period = cfg.creep_on_s + cfg.creep_off_s
                gate = (now % period) < cfg.creep_on_s
                return AvoidDecision(AvoidState.CAUTION, None, f"CAUTION {d:.0f}cm", gate)
            self._state = AvoidState.CLEAR
            return AvoidDecision(AvoidState.CLEAR, None, "CLEAR")

        if st == AvoidState.BRAKE:
            if now - self._t0 < cfg.brake_s:
                return AvoidDecision(st, "STOP", f"BRAKING {d:.0f}cm")
            side = self._choose_side(now, left_clearance, right_clearance)
            self._begin_turn(now, side, flipped=False)
            st = self._state  # fall through to TURN

        if st == AvoidState.TURN:
            if self._clear_count >= cfg.clear_readings:
                self._state = AvoidState.BYPASS
                self._t0 = now
                return AvoidDecision(AvoidState.BYPASS, "FORWARD", "PATH CLEAR: BYPASS")
            t = now - self._t0
            limit = cfg.turn_timeout_s * (2.0 if self._flipped else 1.0)
            if t > limit:
                if not self._flipped:
                    other = "LEFT" if self._dir == "RIGHT" else "RIGHT"
                    self._begin_turn(now, other, flipped=True)
                    return AvoidDecision(AvoidState.TURN, "STOP", f"NO GAP: TRY {other}")
                if not self._backed_up:
                    self._backed_up = True
                    self._state = AvoidState.BACKUP
                    self._t0 = now
                    return AvoidDecision(AvoidState.BACKUP, "BACKWARD", "BOXED IN: BACKING UP")
                self._state = AvoidState.STUCK
                self._clear_count = 0
                return self._stop_decision("BLOCKED ON ALL SIDES")
            phase = t % (cfg.turn_on_s + cfg.turn_off_s)
            cmd = self._dir if phase < cfg.turn_on_s else "STOP"
            return AvoidDecision(AvoidState.TURN, cmd, f"TURNING {self._dir} ({d:.0f}cm)")

        if st == AvoidState.BACKUP:
            if now - self._t0 < cfg.backup_s:
                return AvoidDecision(st, "BACKWARD", "BACKING UP")
            other = "LEFT" if self._dir == "RIGHT" else "RIGHT"
            self._begin_turn(now, other, flipped=True)
            return AvoidDecision(AvoidState.TURN, "STOP", f"RETRY {other}")

        if st == AvoidState.BYPASS:
            if d <= cfg.stop_cm:
                self._backed_up = False
                self._begin_brake(now)
                return self._stop_decision(f"OBSTACLE {d:.0f}cm: BRAKING")
            if now - self._t0 < cfg.bypass_s:
                return AvoidDecision(st, "FORWARD", "BYPASSING")
            self._state = AvoidState.CLEAR
            self._backed_up = False
            return AvoidDecision(AvoidState.CLEAR, None, "RESUMING")

        # STUCK: latched until the way ahead really opens up (or reset()).
        if self._clear_count >= cfg.clear_readings:
            self._attempts.clear()
            self._backed_up = False
            self._state = AvoidState.CLEAR
            return AvoidDecision(AvoidState.CLEAR, None, "UNBLOCKED")
        return AvoidDecision(AvoidState.STUCK, "STOP", "STUCK: REMOVE OBSTACLE")

    def _stop_decision(self, note: str) -> AvoidDecision:
        return AvoidDecision(self._state, "STOP", note)
