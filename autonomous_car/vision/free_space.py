"""
vision/free_space.py — camera free-space ranging, self-calibrated by the sonar.

The camera sees the whole scene but has no idea how far things are; the sonar knows
the range but only along one narrow beam. Fusing them gives the car both:

  1. Floor segmentation splits the view into N_BANDS vertical bands. In each band the
     lowest obstacle pixel (its contact point with the floor) is found.
  2. For a flat floor, the inverse distance of a floor point is linear in its image row:
     1/Z = a*row + b. Whenever the sonar and the centre band both see the same obstacle,
     that pair (row, cm) updates a and b by recursive least squares. After a handful of
     obstacles the camera can range every band, not just the one the sonar points at.
  3. choose_heading() steers toward the goal (or lane) through the nearest direction
     that is actually free, which is the "better route" around distant obstacles.

Pure numpy; no OpenCV or hardware imports so it can be tested offline.
"""

from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

N_BANDS = 7
HFOV_DEG = 62.2
ROI_TOP_FRAC = 0.45          # the floor grid covers the bottom 55% of the image
MAX_RANGE_CM = 400.0
BAND_BLOCK_FRAC = 0.5        # a row is "blocked" when this share of the band is non-floor
MIN_RUN_ROWS = 3             # consecutive blocked rows that make an obstacle (rejects lane lines)

BAND_ANGLES_DEG: Tuple[float, ...] = tuple(
    (i + 0.5) / N_BANDS * HFOV_DEG - HFOV_DEG / 2.0 for i in range(N_BANDS)
)
CENTER_BAND = N_BANDS // 2


@dataclass(frozen=True)
class FreeSpace:
    ranges_cm: Tuple[float, ...]        # per band, MAX_RANGE_CM when nothing is seen
    base_rows: Tuple[Optional[float], ...]  # image row fraction (0..1) of each obstacle base
    calibrated: bool

    @property
    def forward_cm(self) -> float:
        """Nearest obstacle in the three central bands (what the car would hit going straight)."""
        mid = self.ranges_cm[CENTER_BAND - 1:CENTER_BAND + 2]
        return float(min(mid))

    def side_clearance(self) -> Tuple[float, float]:
        """(left, right) clearance in 0..1, from the mean range of the outer bands."""
        left = float(np.mean(self.ranges_cm[:CENTER_BAND - 1])) / 200.0
        right = float(np.mean(self.ranges_cm[CENTER_BAND + 2:])) / 200.0
        return min(1.0, left), min(1.0, right)


class RangeModel:
    """Online fit of 1/Z = a*row + b (row = image row as a fraction of frame height)."""

    FORGET = 0.97
    MIN_SAMPLES = 10
    MIN_ROW_SPAN = 0.05

    def __init__(self) -> None:
        # Prior: floor point at the bottom of the image ~18 cm away, at the ROI top ~180 cm.
        z_bottom, z_top = 18.0, 180.0
        a = (1.0 / z_bottom - 1.0 / z_top) / (1.0 - ROI_TOP_FRAC)
        b = 1.0 / z_bottom - a
        self._theta = np.array([a, b], dtype=np.float64)
        self._P = np.eye(2) * 1e3
        self._n = 0
        self._row_min = 1.0
        self._row_max = 0.0

    @property
    def calibrated(self) -> bool:
        return (self._n >= self.MIN_SAMPLES
                and (self._row_max - self._row_min) >= self.MIN_ROW_SPAN
                and self._theta[0] > 0)

    def add(self, row_frac: float, z_cm: float) -> None:
        if not (5.0 <= z_cm <= 250.0):
            return
        x = np.array([row_frac, 1.0])
        y = 1.0 / z_cm
        Px = self._P @ x
        k = Px / (self.FORGET + x @ Px)
        self._theta = self._theta + k * (y - x @ self._theta)
        self._P = (self._P - np.outer(k, Px)) / self.FORGET
        self._n += 1
        self._row_min = min(self._row_min, row_frac)
        self._row_max = max(self._row_max, row_frac)

    def range_cm(self, row_frac: float) -> float:
        inv = float(self._theta[0] * row_frac + self._theta[1])
        if inv <= 1.0 / MAX_RANGE_CM:
            return MAX_RANGE_CM
        return min(MAX_RANGE_CM, 1.0 / inv)


class FreeSpaceAnalyzer:
    def __init__(self) -> None:
        self.model = RangeModel()
        self._last_sonar: Optional[float] = None

    @staticmethod
    def _band_bases(is_floor: np.ndarray) -> Tuple[Optional[int], ...]:
        """Per band: grid row of the lowest obstacle pixel, or None if the band is clear."""
        rows, cols = is_floor.shape
        edges = np.linspace(0, cols, N_BANDS + 1).astype(int)
        bases = []
        for i in range(N_BANDS):
            blocked = (~is_floor[:, edges[i]:edges[i + 1]]).mean(axis=1) > BAND_BLOCK_FRAC
            base = None
            for r in range(rows - 1, MIN_RUN_ROWS - 2, -1):     # scan from the bumper upward
                if blocked[r - MIN_RUN_ROWS + 1:r + 1].all():
                    base = r
                    break
            bases.append(base)
        return tuple(bases)

    @staticmethod
    def _row_frac(grid_row: int, rows: int) -> float:
        return ROI_TOP_FRAC + (grid_row + 0.5) / rows * (1.0 - ROI_TOP_FRAC)

    def update(self, is_floor: np.ndarray, sonar_cm: Optional[float] = None) -> FreeSpace:
        """is_floor: bool grid (rows x cols) covering the bottom of the image."""
        rows = is_floor.shape[0]
        bases = self._band_bases(is_floor)
        base_rows = tuple(None if b is None else self._row_frac(b, rows) for b in bases)

        # Learn from the sonar only when it is steady and the centre band agrees an obstacle is there.
        steady = (sonar_cm is not None and self._last_sonar is not None
                  and abs(sonar_cm - self._last_sonar) < 6.0)
        centre = bases[CENTER_BAND]
        if (steady and centre is not None and 3 <= centre < rows - 3
                and sonar_cm is not None and 15.0 <= sonar_cm <= 200.0):
            self.model.add(base_rows[CENTER_BAND], sonar_cm)  # type: ignore[arg-type]
        self._last_sonar = sonar_cm

        ranges = tuple(MAX_RANGE_CM if r is None else self.model.range_cm(r) for r in base_rows)
        return FreeSpace(ranges, base_rows, self.model.calibrated)


def choose_heading(
    ranges_cm: Sequence[float],
    desired_deg: float = 0.0,
    clear_cm: float = 60.0,
) -> Optional[float]:
    """
    Pick the heading (degrees, + = right) closest to desired_deg whose band, and the
    neighbours the car body also sweeps, are all farther than clear_cm.
    Returns None when every direction is blocked.
    """
    n = len(ranges_cm)
    best: Optional[float] = None
    best_cost = float("inf")
    for i in range(n):
        lo, hi = max(0, i - 1), min(n - 1, i + 1)
        if min(ranges_cm[lo:hi + 1]) < clear_cm:
            continue
        cost = abs(BAND_ANGLES_DEG[i] - desired_deg)
        if cost < best_cost:
            best, best_cost = BAND_ANGLES_DEG[i], cost
    if best is None:
        return None
    # If the desired heading itself is inside the chosen free band, keep it (no needless wobble).
    i = int(np.argmin([abs(a - best) for a in BAND_ANGLES_DEG]))
    half = HFOV_DEG / N_BANDS / 2.0
    if abs(desired_deg - BAND_ANGLES_DEG[i]) <= half:
        return desired_deg
    return best
