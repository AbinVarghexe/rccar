"""
FPS Counter Utility for monitoring execution frequency and video frame rates.
"""

import time
from collections import deque
from typing import Optional

class FPSCounter:
    """
    Computes frames-per-second using a rolling window of frame timestamps.
    """
    def __init__(self, window_size: int = 30) -> None:
        self.window_size: int = window_size
        self._timestamps: deque = deque(maxlen=window_size)
        self._start_time: Optional[float] = None
        self._last_time: Optional[float] = None
        self._total_frames: int = 0

    def start(self) -> "FPSCounter":
        """Start the timer."""
        self._start_time = time.perf_counter()
        self._last_time = self._start_time
        self._timestamps.clear()
        self._total_frames = 0
        return self

    def update(self) -> None:
        """Call on every frame update."""
        now = time.perf_counter()
        if self._start_time is None:
            self.start()
            return
        
        self._timestamps.append(now)
        self._last_time = now
        self._total_frames += 1

    def get_fps(self) -> float:
        """Calculate the average FPS over the configured rolling window."""
        if len(self._timestamps) < 2:
            return 0.0
        elapsed = self._timestamps[-1] - self._timestamps[0]
        if elapsed <= 0:
            return 0.0
        return (len(self._timestamps) - 1) / elapsed

    @property
    def total_frames(self) -> int:
        return self._total_frames
