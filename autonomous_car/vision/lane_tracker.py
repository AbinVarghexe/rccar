"""
vision/lane_tracker.py - lane keeping for a two-lane track made of three tapes.

Track: a solid white tape on each outer edge and a dashed tape down the
centre. The car follows the lane it is in, may cross the dashed centre to
change lanes, and must never cross a solid outer line.

What differs from the classic ``LaneDetector``:

* Every white line is traced bottom-to-top with sliding windows and fitted
  with a quadratic, so bends are followed instead of smeared into one peak.
* Solid versus dashed is measured (how much of the line is actually lit), so
  a dashed centre can no longer lose to the stronger far edge.
* The nearest line on each side of the car bounds the lane; the lane width is
  learned from the track itself, no centimetres needed.
* Only solid outer lines are hard limits. The dashed centre is never a wall.
* Steering uses the lane centre a little way ahead as well as right at the
  bumper, so the car starts turning before the bend arrives.

The result is a ``LaneResult`` with extra fields, so the follower is unchanged.
"""

import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np

try:
    from vision.lane_detector import Direction, LaneDetector, LaneResult
except ImportError:
    from autonomous_car.vision.lane_detector import Direction, LaneDetector, LaneResult

BEV_W, BEV_H = 400, 240          # bird's-eye plane size
NEAR_ROW = 225                   # row measured "at the bumper"
LOOK_ROW = 120                   # row measured "ahead"
NEAR_WEIGHT = 0.6                # error = 0.6 * near + 0.4 * ahead
TAPE_KERNEL = 21                 # top-hat size: wider than the tape, narrower than glare
TAPE_MIN_CONTRAST = 35           # tape must stand this far above its surroundings (V)
TAPE_MIN_VALUE = 100             # ... and be at least this bright
TAPE_MAX_SATURATION = 90         # white / cream tape, not coloured objects
MIN_SPAN = 80                    # a line spans at least this many rows (rejects streaks, glare)
MIN_WINDOW_HITS = 5              # tape shows up in at least this many of the 8 windows
INLIER_PX = 6                    # pixels within this of the first fit are the line
MIN_INLIER_FRACTION = 0.7
MAX_FIT_RMS = 4.5                # px; tape hugs a smooth curve, a chain of specks does not
WIDTH_HISTORY = 15               # lane width = median of this many recent observations
MIN_WIDTH_SAMPLES = 3            # observations before the width is trusted as a prior
WIDTH_TOLERANCE = 0.3            # a neighbour pair within 30% of the lane width is a lane
SEED_TOP = 150                   # histogram band used to find line starts
SEED_SMOOTH = 7
SEED_MIN_MEAN = 12.0             # smoothed lit rows needed for a line start
SEED_MIN_SEP = 24
MAX_SEEDS = 4
N_WINDOWS = 8
WINDOW_H = BEV_H // N_WINDOWS
WINDOW_MARGIN = 14
MIN_WINDOW_PIXELS = 18
COVER_HALF_BAND = 4
MIN_COVERAGE = 0.2
SOLID_COVERAGE = 0.78            # >= solid tape, below = dashed tape
TRACK_MATCH_PX = 25
TRACK_MAX_AGE = 10
DUPLICATE_PX = 14
WIDTH_RANGE = (40.0, 220.0)
STRADDLE_RATIO = 1.5             # two solids this many lane widths apart = both outer edges


@dataclass
class _Line:
    poly: np.ndarray             # x = poly[0]*y^2 + poly[1]*y + poly[2]
    x_near: float
    y_top: int
    coverage: float
    kind: str = "solid"


@dataclass
class _Lane:
    centre: Optional[np.ndarray] = None
    left: Optional[_Line] = None
    right: Optional[_Line] = None
    hard_left: Optional[_Line] = None
    hard_right: Optional[_Line] = None
    side: Optional[str] = None
    width: float = 0.0
    y_top: int = BEV_H


@dataclass
class _Steer:
    error: float = 0.0
    look_ahead: float = 0.0
    centre_x: Optional[int] = None
    direction: Direction = Direction.NO_LANE
    suggested: float = 0.0
    left_hit: bool = False
    right_hit: bool = False
    lo: float = -1.0
    hi: float = 1.0
    warning: Optional[str] = None


def constrain_side_clearance(left_clear: float, right_clear: float, lane: LaneResult) -> Tuple[float, float]:
    """Zero the clearance of a side that is bounded by a solid outer line, so an
    obstacle detour can only turn toward the centre / the other lane."""
    blocked = getattr(lane, "blocked_sides", ())
    return (0.0 if "left" in blocked else left_clear, 0.0 if "right" in blocked else right_clear)


class ThreeLineLaneDetector(LaneDetector):
    """Lane detector for solid-edge / dashed-centre / solid-edge tape."""

    def __init__(
        self,
        deadzone_pixels: int = 15,
        vehicle_half_width_px: int = 45,
        boundary_margin_px: int = 18,
        kp: float = 0.0075,
        kd: float = 0.0025,
        solid_coverage: float = SOLID_COVERAGE,
    ) -> None:
        super().__init__(deadzone_pixels, vehicle_half_width_px, boundary_margin_px, kp, kd)
        self.solid_coverage = solid_coverage
        self._tracks: List[dict] = []
        self._widths: Deque[float] = deque(maxlen=WIDTH_HISTORY)
        self._width_learned = False
        self._lane_side: Optional[str] = None

    # ── line finding ─────────────────────────────────────────────────────────
    @staticmethod
    def _white_mask(warped: np.ndarray) -> np.ndarray:
        """Tape = thin, bright, low-saturation structure that stands out from its floor.

        A fixed brightness cut-off (the classic detector's V >= 140) lights up the
        whole frame on a bright floor and misses tape in a dim room. A top-hat
        keeps only structures narrower than ``TAPE_KERNEL`` that are brighter than
        their surroundings, so floor brightness does not matter and a glare patch
        wider than the tape is removed.
        """
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        value = hsv[..., 2]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (TAPE_KERNEL, TAPE_KERNEL))
        contrast = cv2.morphologyEx(value, cv2.MORPH_TOPHAT, kernel)
        tape = ((contrast >= TAPE_MIN_CONTRAST) & (value >= TAPE_MIN_VALUE)
                & (hsv[..., 1] <= TAPE_MAX_SATURATION)).astype(np.uint8)
        return cv2.morphologyEx(tape, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)) > 0

    @staticmethod
    def _seeds(binary: np.ndarray) -> List[int]:
        band = binary[SEED_TOP:].sum(axis=0).astype(np.float32)
        work = np.convolve(band, np.ones(SEED_SMOOTH, np.float32) / SEED_SMOOTH, mode="same")
        seeds: List[int] = []
        while len(seeds) < MAX_SEEDS:
            x = int(np.argmax(work))
            if work[x] < SEED_MIN_MEAN:
                break
            seeds.append(x)
            work[max(0, x - SEED_MIN_SEP): x + SEED_MIN_SEP + 1] = 0
        return seeds

    @staticmethod
    def _trace(binary: np.ndarray, seed_x: int) -> Optional[Tuple[np.ndarray, int]]:
        """Follow a line upward; returns (quadratic coefficients, top row)."""
        x, dx, hits = float(seed_x), 0.0, 0
        xs_all: List[np.ndarray] = []
        ys_all: List[np.ndarray] = []
        for i in range(N_WINDOWS):
            y1 = BEV_H - i * WINDOW_H
            y0 = y1 - WINDOW_H
            # Search where the line should be if it keeps its direction; on a bend
            # the margin then only has to cover the change in slope.
            centre = x + dx
            lo, hi = int(max(0, centre - WINDOW_MARGIN)), int(min(BEV_W, centre + WINDOW_MARGIN + 1))
            if hi <= lo:
                break
            ys, xs = np.nonzero(binary[y0:y1, lo:hi])
            if xs.size >= MIN_WINDOW_PIXELS:
                new_x = lo + float(xs.mean())
                dx, x = new_x - x, new_x
                hits += 1
                xs_all.append(xs + lo)
                ys_all.append(ys + y0)
            else:
                x = centre  # a gap in a dashed line: coast along the last direction
        if hits < MIN_WINDOW_HITS:
            return None
        px, py = np.concatenate(xs_all), np.concatenate(ys_all)
        span = int(py.max() - py.min())
        if span < MIN_SPAN:
            return None
        degree = 2 if span >= 90 else 1
        coef = np.polyfit(py, px, degree)
        # A speck inside a search window drags the average; refit on the pixels that
        # agree with the first fit, and insist that most of them do.
        inliers = np.abs(px - np.polyval(coef, py)) <= INLIER_PX
        if inliers.mean() < MIN_INLIER_FRACTION:
            return None
        coef = np.polyfit(py[inliers], px[inliers], degree)
        residual = px[inliers] - np.polyval(coef, py[inliers])
        if float(np.sqrt(np.mean(residual ** 2))) > MAX_FIT_RMS:
            return None
        poly = np.zeros(3)
        poly[3 - len(coef):] = coef
        return poly, int(py.min())

    @staticmethod
    def _coverage(binary: np.ndarray, poly: np.ndarray) -> float:
        """Fraction of near-field rows with tape under the fitted line."""
        rows = np.arange(SEED_TOP, BEV_H)
        cols = np.rint(np.polyval(poly, rows)).astype(int)
        inside = (cols >= 0) & (cols < BEV_W)
        if not inside.any():
            return 0.0
        rows, cols = rows[inside], cols[inside]
        cums = np.cumsum(binary, axis=1, dtype=np.int32)
        hi = np.clip(cols + COVER_HALF_BAND, 0, BEV_W - 1)
        lo = cols - COVER_HALF_BAND - 1
        below = np.where(lo >= 0, cums[rows, np.clip(lo, 0, BEV_W - 1)], 0)
        return float(np.mean((cums[rows, hi] - below) > 0))

    def _find_lines(self, binary: np.ndarray) -> List[_Line]:
        lines: List[_Line] = []
        for seed in self._seeds(binary):
            traced = self._trace(binary, seed)
            if traced is None:
                continue
            poly, y_top = traced
            coverage = self._coverage(binary, poly)
            if coverage >= MIN_COVERAGE:
                lines.append(_Line(poly, float(np.polyval(poly, NEAR_ROW)), y_top, coverage))
        lines.sort(key=lambda ln: ln.x_near)
        unique: List[_Line] = []
        for ln in lines:
            if unique and ln.x_near - unique[-1].x_near < DUPLICATE_PX:
                if ln.coverage > unique[-1].coverage:
                    unique[-1] = ln
            else:
                unique.append(ln)
        return unique

    def _classify(self, lines: List[_Line]) -> List[_Line]:
        """Solid or dashed, smoothed over frames so a dash gap cannot flip it."""
        previous = self._tracks
        used: set = set()
        fresh: List[dict] = []
        for ln in lines:
            near = [(abs(t["x"] - ln.x_near), i) for i, t in enumerate(previous)
                    if i not in used and abs(t["x"] - ln.x_near) <= TRACK_MATCH_PX]
            if near:
                _, idx = min(near)
                used.add(idx)
                cover = 0.6 * previous[idx]["cov"] + 0.4 * ln.coverage
            else:
                cover = ln.coverage
            ln.kind = "solid" if cover >= self.solid_coverage else "dashed"
            fresh.append({"x": ln.x_near, "cov": cover, "age": 0})
        for i, t in enumerate(previous):
            if i not in used and t["age"] < TRACK_MAX_AGE:
                fresh.append({**t, "age": t["age"] + 1})
        self._tracks = fresh
        return lines

    # ── lane model ───────────────────────────────────────────────────────────
    def _learn_width(self, width: float) -> None:
        """Lane width = median of recent solid/dashed pairs, so one stray line cannot
        poison it. Every in-range observation counts, even one the current width
        would reject, otherwise a bad first reading could never be corrected."""
        if not WIDTH_RANGE[0] <= width <= WIDTH_RANGE[1]:
            return
        self._widths.append(width)
        self.nominal_lane_width = float(np.median(self._widths))
        self.nominal_half_width = int(round(self.nominal_lane_width / 2))
        self._width_learned = True

    def _shifted(self, line: _Line, dx: float) -> np.ndarray:
        shifted = line.poly.copy()
        shifted[2] += dx
        return shifted

    def _best_width_match(
        self, left: List[_Line], right: List[_Line]
    ) -> Optional[Tuple[_Line, _Line]]:
        """Once the lane width is known, prefer the solid/dashed pair that matches it
        over merely the nearest lines, so a stray line next to the car cannot win."""
        if len(self._widths) < MIN_WIDTH_SAMPLES:
            return None
        nominal = self.nominal_lane_width
        scored = [(abs(r.x_near - l.x_near - nominal) / nominal, i, j)
                  for i, l in enumerate(left) for j, r in enumerate(right)
                  if {l.kind, r.kind} == {"solid", "dashed"}]
        if not scored:
            return None
        score, i, j = min(scored)
        return (left[i], right[j]) if score <= WIDTH_TOLERANCE else None

    def _resolve_lane(self, lines: List[_Line]) -> _Lane:
        car = self.bev_center_x
        left = sorted((ln for ln in lines if ln.x_near < car), key=lambda ln: -ln.x_near)
        right = sorted((ln for ln in lines if ln.x_near >= car), key=lambda ln: ln.x_near)
        lft = left[0] if left else None   # nearest first
        rgt = right[0] if right else None
        matched = self._best_width_match(left[:3], right[:3])
        if matched is not None:
            lft, rgt = matched
        if lft and rgt:
            lane = self._lane_from_pair(lft, rgt, car)
            if lane is not None:
                return lane
            lft, rgt = (lft, None) if car - lft.x_near <= rgt.x_near - car else (None, rgt)
        if lft:
            return self._lane_from_one(lft, towards=+1)
        if rgt:
            return self._lane_from_one(rgt, towards=-1)
        return _Lane()

    def _lane_from_pair(self, lft: _Line, rgt: _Line, car: int) -> Optional[_Lane]:
        width = rgt.x_near - lft.x_near
        y_top = min(lft.y_top, rgt.y_top)
        if {lft.kind, rgt.kind} == {"solid", "dashed"}:
            self._learn_width(width)
            if width > STRADDLE_RATIO * self.nominal_lane_width:
                return None  # not neighbours: judge by the nearer line alone
            side = "left" if rgt.kind == "dashed" else "right"
            return _Lane((lft.poly + rgt.poly) / 2, lft, rgt,
                         lft if lft.kind == "solid" else None,
                         rgt if rgt.kind == "solid" else None, side, width, y_top)
        if lft.kind == rgt.kind == "solid":
            straddle = (not self._width_learned) or width > STRADDLE_RATIO * self.nominal_lane_width
            if straddle:
                if not self._width_learned:
                    self._learn_width(width / 2)
                side = self._lane_side or ("left" if car < (lft.x_near + rgt.x_near) / 2 else "right")
                half = self.nominal_lane_width / 2
                centre = self._shifted(lft, +half) if side == "left" else self._shifted(rgt, -half)
                return _Lane(centre, lft, rgt, lft, rgt, side, self.nominal_lane_width, y_top)
            return _Lane((lft.poly + rgt.poly) / 2, lft, rgt, lft, rgt, self._lane_side, width, y_top)
        return _Lane((lft.poly + rgt.poly) / 2, lft, rgt, None, None, self._lane_side, width, y_top)

    def _lane_from_one(self, line: _Line, towards: int) -> _Lane:
        """One line visible; the lane lies ``towards`` (+1 right, -1 left) of it."""
        centre = self._shifted(line, towards * self.nominal_lane_width / 2)
        if towards > 0:   # line is on the car's left
            side = "left" if line.kind == "solid" else "right"
            return _Lane(centre, line, None, line if line.kind == "solid" else None, None,
                         side, self.nominal_lane_width, line.y_top)
        side = "right" if line.kind == "solid" else "left"
        return _Lane(centre, None, line, None, line if line.kind == "solid" else None,
                     side, self.nominal_lane_width, line.y_top)

    # ── steering and limits ──────────────────────────────────────────────────
    def _gate(self, lane: _Lane, demand: float) -> Tuple[bool, bool, float, float, Optional[str]]:
        """Limits from solid outer lines only; the dashed centre never blocks.

        Near a solid line, steering toward it is capped at what the lane itself
        asks for (``demand``, the follower's error/100). Outside influences
        (free space, tag homing, the neural pilot) cannot push the car over the
        line, but on a bend toward that line the lane can still steer into it.
        """
        car = self.bev_center_x
        half = min(self.vehicle_half_width_px, 0.3 * self.nominal_lane_width)
        margin = min(self.boundary_margin_px, 0.12 * self.nominal_lane_width)
        left_hit = lane.hard_left is not None and (car - half) - lane.hard_left.x_near <= margin
        right_hit = lane.hard_right is not None and lane.hard_right.x_near - (car + half) <= margin
        warning = "LEFT" if left_hit else ("RIGHT" if right_hit else None)
        lo = min(0.0, demand) if left_hit else -1.0
        hi = max(0.0, demand) if right_hit else 1.0
        return left_hit, right_hit, lo, hi, warning

    def _blocked_sides(self, lane: _Lane) -> Tuple[str, ...]:
        sides = set()
        if lane.hard_left is not None or lane.side == "left":
            sides.add("left")
        if lane.hard_right is not None or lane.side == "right":
            sides.add("right")
        return tuple(sorted(sides))

    # ── main entry ───────────────────────────────────────────────────────────
    def _steering(self, lane: _Lane) -> _Steer:
        """Lane-centre error (near + ahead), limits, and the PD steering value."""
        if lane.centre is None:
            self._prev_error = 0.0
            return _Steer()
        car = self.bev_center_x
        near = float(np.polyval(lane.centre, NEAR_ROW)) - car
        ahead = float(np.polyval(lane.centre, max(LOOK_ROW, lane.y_top + 5))) - car
        error = NEAR_WEIGHT * near + (1.0 - NEAR_WEIGHT) * ahead
        left_hit, right_hit, lo, hi, warning = self._gate(lane, float(np.clip(error / 100.0, -1.0, 1.0)))
        now = time.time()
        dt = max(0.01, now - self._last_time)
        self._last_time = now
        raw = self.kp * error + self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        direction = (Direction.STRAIGHT if abs(error) <= self.deadzone_pixels
                     else Direction.LEFT if error < 0 else Direction.RIGHT)
        return _Steer(
            error=error, look_ahead=ahead,
            centre_x=int(round(float(np.polyval(lane.centre, int(BEV_H * 0.75))))),
            direction=direction, suggested=max(lo, min(hi, raw)),
            left_hit=left_hit, right_hit=right_hit, lo=lo, hi=hi, warning=warning,
        )

    def detect_from_bgr(self, bgr_frame: np.ndarray) -> LaneResult:
        if bgr_frame is None or bgr_frame.size == 0:
            return LaneResult()
        self.fps_counter.update()
        orig_h, orig_w = bgr_frame.shape[:2]
        frame = bgr_frame if (orig_w, orig_h) == (BEV_W, BEV_H) else cv2.resize(
            bgr_frame, (BEV_W, BEV_H), interpolation=cv2.INTER_NEAREST)
        warped = cv2.warpPerspective(frame, self.M, (BEV_W, BEV_H), flags=cv2.INTER_LINEAR)

        lines = self._classify(self._find_lines(self._white_mask(warped)))
        lane = self._resolve_lane(lines)
        if lane.side is not None:
            self._lane_side = lane.side
        steer = self._steering(lane)
        annotated, birdseye = self._render(frame, warped, lines, lane, steer.warning)
        if (orig_w, orig_h) != (BEV_W, BEV_H):
            annotated = cv2.resize(annotated, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
        return self._result(lane, steer, annotated, birdseye, orig_w, orig_h)

    def _result(self, lane: _Lane, steer: _Steer, annotated: np.ndarray,
                birdseye: np.ndarray, orig_w: int, orig_h: int) -> LaneResult:
        found = lane.centre is not None
        scale_x = orig_w / float(BEV_W)
        return LaneResult(
            lane_center_x=None if steer.centre_x is None else int(steer.centre_x * scale_x),
            lane_center_y=int(orig_h * 0.75) if found else None,
            image_center_x=orig_w // 2,
            error=round(steer.error, 2),
            direction=steer.direction,
            contour_found=found,
            annotated_frame=annotated,
            birdseye_frame=birdseye,
            left_line_x=None if lane.left is None else int(lane.left.x_near * scale_x),
            right_line_x=None if lane.right is None else int(lane.right.x_near * scale_x),
            lane_width=int(lane.width) if found else None,
            lane_detected=found,
            is_left_boundary_violated=steer.left_hit,
            is_right_boundary_violated=steer.right_hit,
            departure_warning=steer.warning,
            suggested_steering=round(float(steer.suggested), 3),
            corrective_steering=0.5 if steer.left_hit else (-0.5 if steer.right_hit else 0.0),
            min_allowed_steering=steer.lo,
            max_allowed_steering=steer.hi,
            lane_side=lane.side,
            left_kind=None if lane.left is None else lane.left.kind,
            right_kind=None if lane.right is None else lane.right.kind,
            look_ahead_error=round(steer.look_ahead, 2),
            blocked_sides=self._blocked_sides(lane) if found else (),
        )

    # ── drawing ──────────────────────────────────────────────────────────────
    def _render(self, frame, warped, lines, lane, warning) -> Tuple[np.ndarray, np.ndarray]:
        rows = np.arange(0, BEV_H, 6)
        birdseye = warped.copy()
        cv2.line(birdseye, (self.bev_center_x, 0), (self.bev_center_x, BEV_H), (255, 120, 0), 1)
        for ln in lines:
            xs = np.polyval(ln.poly, rows)
            pts = np.stack([xs, rows], axis=1)[rows >= ln.y_top].astype(np.int32)
            color = (255, 255, 0) if ln.kind == "solid" else (255, 0, 255)
            if len(pts) > 1:
                cv2.polylines(birdseye, [pts], False, color, 2)
        annotated = frame.copy()
        if lane.centre is not None:
            half = lane.width / 2 if lane.width else self.nominal_lane_width / 2
            xs = np.polyval(lane.centre, rows)
            centre_pts = np.stack([xs, rows], axis=1).astype(np.int32)
            cv2.polylines(birdseye, [centre_pts], False, (0, 255, 0), 2)
            corridor = np.zeros_like(frame)
            poly = np.concatenate([np.stack([xs - half, rows], axis=1),
                                   np.stack([xs + half, rows], axis=1)[::-1]]).astype(np.int32)
            cv2.fillPoly(corridor, [poly], (40, 220, 40))
            annotated = cv2.addWeighted(
                annotated, 1.0, cv2.warpPerspective(corridor, self.Minv, (BEV_W, BEV_H)), 0.35, 0)
        cv2.polylines(annotated, [np.int32(self.src_points)], True, (140, 100, 255), 1)
        if warning:
            cv2.rectangle(annotated, (8, 6), (280, 32), (0, 0, 240), -1)
            cv2.putText(annotated, f"OUTER LINE: {warning}!", (12, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return annotated, birdseye
