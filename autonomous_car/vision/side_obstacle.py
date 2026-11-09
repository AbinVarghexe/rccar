"""
vision/side_obstacle.py - spot an obstacle beside the car and watch it.

Used with the camera turned to a fixed left or right angle. Two parts:

* ``find_side_obstacle`` marks everything in the lower part of the view that
  does not look like the floor and keeps the biggest blob. Thin things such as
  lane tape are removed. The floor colour comes from the follower's own floor
  model; only if there is none does it fall back to the frame's bottom rows,
  which cannot see a wall that fills the bottom of the view (``from_band``).
* ``ObstacleWatch`` follows that one obstacle over time and says whether it is
  standing still, getting closer, sliding sideways, or gone.

Pure numpy / OpenCV, no hardware, so it can be tested offline.
"""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import cv2
import numpy as np

ROI_TOP_FRAC = 0.40              # judge the lower 60% of the image
WORK_W, WORK_H = 160, 58         # size the lower part is shrunk to
FLOOR_DIST_SQ = 8.0              # same floor test as the follower (LAB distance squared)
MIN_FLOOR_STD = np.array([10.0, 5.0, 5.0], np.float32)
MIN_AREA_FRAC = 0.02             # a blob must cover this share of the lower view
BAND_ROWS = 14                   # fallback floor model: the bottom rows of the view


@dataclass(frozen=True)
class SideObstacle:
    present: bool
    area_frac: float = 0.0
    cx: float = 0.5              # 0..1 across the image
    base: float = 0.0            # 0..1 image row of the obstacle's lowest pixel (1 = at the car)
    from_band: bool = False      # floor model came from the frame itself (less reliable)


def find_side_obstacle(
    bgr: Optional[np.ndarray],
    floor_mean: Optional[np.ndarray] = None,
    floor_std: Optional[np.ndarray] = None,
) -> SideObstacle:
    """Biggest non-floor blob in the lower view, or ``present=False``."""
    if bgr is None or bgr.size == 0:
        return SideObstacle(False)
    height = bgr.shape[0]
    roi = bgr[int(height * ROI_TOP_FRAC):]
    if roi.size == 0:
        return SideObstacle(False)
    small = cv2.resize(roi, (WORK_W, WORK_H), interpolation=cv2.INTER_AREA)
    lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB).astype(np.float32)

    from_band = floor_mean is None or floor_std is None
    if from_band:
        band = lab[-BAND_ROWS:].reshape(-1, 3)
        mean = np.median(band, axis=0)
        std = 1.4826 * np.median(np.abs(band - mean), axis=0)
    else:
        mean = np.asarray(floor_mean, np.float32)
        std = np.asarray(floor_std, np.float32)
    std = np.maximum(std, MIN_FLOOR_STD)

    non_floor = (np.sum(((lab - mean) / std) ** 2, axis=2) > FLOOR_DIST_SQ).astype(np.uint8)
    non_floor = cv2.morphologyEx(non_floor, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, _labels, stats, centroids = cv2.connectedComponentsWithStats(non_floor, connectivity=8)
    if count <= 1:
        return SideObstacle(False, from_band=from_band)

    best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area_frac = float(stats[best, cv2.CC_STAT_AREA]) / float(WORK_W * WORK_H)
    lowest = (stats[best, cv2.CC_STAT_TOP] + stats[best, cv2.CC_STAT_HEIGHT]) / float(WORK_H)
    # A blob under the entry size is reported with its geometry but present=False, so
    # ObstacleWatch can keep following an obstacle that is already confirmed.
    return SideObstacle(
        area_frac >= MIN_AREA_FRAC, area_frac, float(centroids[best][0]) / WORK_W,
        ROI_TOP_FRAC + lowest * (1.0 - ROI_TOP_FRAC), from_band,
    )


@dataclass(frozen=True)
class Watch:
    status: str                  # unknown | clear | obstacle | approaching | moving
    seen_s: float = 0.0          # how long this obstacle has been observed
    closing_per_s: float = 0.0   # how fast its base moves toward the car (image rows / s)
    lateral_per_s: float = 0.0   # how fast it slides across the view (image widths / s)
    cleared: bool = False        # True on the one update where it turned from obstacle to clear


BLOCKED = ("obstacle", "approaching", "moving")


class ObstacleWatch:
    """Follow one obstacle over time (static, closing in, moving, or gone).

    A warning needs ``CONFIRM_SAMPLES`` frames in a row with the obstacle present,
    so one noisy frame (or a blob flickering around the entry size) never warns.
    Once confirmed, the obstacle is kept while its blob stays above ``EXIT_AREA``,
    a little under the entry size, so it does not drop out when it dips below.
    """

    HISTORY_S = 1.5
    MIN_SPAN_S = 0.4
    CONFIRM_SAMPLES = 3
    EXIT_AREA = 0.01

    def __init__(self, clear_after_s: float = 1.0, approach_per_s: float = 0.05,
                 lateral_per_s: float = 0.10) -> None:
        self.clear_after_s = clear_after_s
        self.approach_per_s = approach_per_s
        self.lateral_threshold = lateral_per_s
        self._history: Deque[Tuple[float, float, float]] = deque()
        self._first_seen: Optional[float] = None
        self._absent_since: Optional[float] = None
        self._streak = 0
        self._status = "unknown"

    def _rates(self) -> Tuple[float, float]:
        if len(self._history) < 3:
            return 0.0, 0.0
        t = np.array([h[0] for h in self._history])
        if t[-1] - t[0] < self.MIN_SPAN_S:
            return 0.0, 0.0
        closing = float(np.polyfit(t, [h[2] for h in self._history], 1)[0])
        lateral = float(np.polyfit(t, [h[1] for h in self._history], 1)[0])
        return closing, lateral

    def update(self, now: float, obs: SideObstacle) -> Watch:
        confirmed = self._status in BLOCKED
        present = obs.present or (confirmed and obs.area_frac >= self.EXIT_AREA)
        if present:
            self._absent_since = None
            self._streak += 1
            if self._first_seen is None:
                self._first_seen = now
            self._history.append((now, obs.cx, obs.base))
            while self._history and now - self._history[0][0] > self.HISTORY_S:
                self._history.popleft()
            if not confirmed and self._streak < self.CONFIRM_SAMPLES:
                self._status = "clear"                # a candidate, not yet an obstacle
                return Watch("clear")
            closing, lateral = self._rates()
            self._status = ("approaching" if closing >= self.approach_per_s
                            else "moving" if abs(lateral) >= self.lateral_threshold
                            else "obstacle")
            return Watch(self._status, now - self._first_seen, closing, lateral)

        self._streak = 0
        if not confirmed:                            # nothing confirmed: forget any candidate
            self._first_seen = self._absent_since = None
            self._history.clear()
            self._status = "clear"
            return Watch("clear")
        if self._absent_since is None:
            self._absent_since = now
        if now - self._absent_since < self.clear_after_s:   # a blink is not 'gone'
            closing, lateral = self._rates()
            return Watch(self._status, now - self._first_seen, closing, lateral)
        seen = now - self._first_seen
        self._first_seen = self._absent_since = None
        self._history.clear()
        was_blocked = self._status != "clear"
        self._status = "clear"
        return Watch("clear", seen, cleared=was_blocked)
