"""
Per-stage latency tracking for the pilot control loop.

The loop used to report one total latency number, which cannot say whether
capture, inference, the safety gate or the motor write is the slow part.
``StageTimer`` keeps a bounded rolling window per stage so the dashboard and
the drive log can show where the time goes.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from contextlib import contextmanager
from typing import Deque, Dict, Iterator


class StageTimer:
    """Thread-safe rolling latency statistics keyed by stage name."""

    def __init__(self, window: int = 100) -> None:
        self._window = max(2, int(window))
        self._samples: Dict[str, Deque[float]] = {}
        self._lock = threading.Lock()

    def record(self, stage: str, seconds: float) -> None:
        """Add one measurement; non-finite or negative values are ignored."""
        if not math.isfinite(seconds) or seconds < 0.0:
            return
        with self._lock:
            samples = self._samples.get(stage)
            if samples is None:
                samples = deque(maxlen=self._window)
                self._samples[stage] = samples
            samples.append(seconds)

    @contextmanager
    def stage(self, name: str) -> Iterator[None]:
        """Time a block; the sample is recorded even if the block raises."""
        started = time.perf_counter()
        try:
            yield
        finally:
            self.record(name, time.perf_counter() - started)

    def last_ms(self) -> Dict[str, float]:
        """Most recent sample per stage, in milliseconds."""
        with self._lock:
            return {
                name: round(samples[-1] * 1000.0, 2)
                for name, samples in self._samples.items()
                if samples
            }

    def snapshot(self) -> Dict[str, Dict[str, float]]:
        """avg / p95 / max / last (ms) and sample count for every stage."""
        with self._lock:
            copied = {name: list(samples) for name, samples in self._samples.items()}
        result: Dict[str, Dict[str, float]] = {}
        for name, values in copied.items():
            if not values:
                continue
            ordered = sorted(values)
            p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
            result[name] = {
                "avg": round(sum(values) / len(values) * 1000.0, 2),
                "p95": round(ordered[p95_index] * 1000.0, 2),
                "max": round(ordered[-1] * 1000.0, 2),
                "last": round(values[-1] * 1000.0, 2),
                "count": len(values),
            }
        return result
