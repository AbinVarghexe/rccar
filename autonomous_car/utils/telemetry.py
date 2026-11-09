"""
Drive telemetry logger for autonomous runs.

The manual-driving recorder only captures human labels. This logger records
what the pilot actually did while driving itself (model output, the command
that survived the safety gate, sonar, frame age, stage timings) so a run can
be explained afterwards and replayed offline with ``utils.session_replay``.

Rows go to ``telemetry.jsonl``. A JPEG is stored for every Nth row only, to
keep SD-card wear and Raspberry Pi CPU cost low. Logging stops by itself when
free disk space drops below ``min_free_mb``.
"""

from __future__ import annotations

import json
import logging
import shutil
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover - OpenCV is supplied by Raspberry Pi OS
    cv2 = None

logger = logging.getLogger(__name__)

DISK_CHECK_EVERY_ROWS = 100


class TelemetryLogger:
    """Append-only JSONL drive log with sparse frame capture."""

    def __init__(
        self,
        base_dir: Path,
        frame_every: int = 5,
        jpeg_quality: int = 80,
        min_free_mb: int = 200,
    ) -> None:
        self.base_dir = Path(base_dir)
        self.frame_every = max(0, int(frame_every))
        self.jpeg_quality = int(jpeg_quality)
        self.min_free_bytes = max(0, int(min_free_mb)) * 1024 * 1024
        self._lock = threading.RLock()
        self._file = None
        self._session_dir: Optional[Path] = None
        self._rows = 0
        self._images = 0
        self._stop_reason = ""

    @property
    def active(self) -> bool:
        with self._lock:
            return self._file is not None

    def start(self) -> Dict[str, Any]:
        with self._lock:
            if self._file is not None:
                return self.status()
            self.base_dir.mkdir(parents=True, exist_ok=True)
            if self._disk_low():
                self._stop_reason = "disk space below limit"
                logger.warning("[TELEMETRY] Not starting: %s", self._stop_reason)
                return self.status()
            name = time.strftime("drive-%Y%m%d-%H%M%S")
            session = self.base_dir / name
            suffix = 1
            while session.exists():
                session = self.base_dir / f"{name}-{suffix}"
                suffix += 1
            session.mkdir(parents=True)
            self._session_dir = session
            self._file = open(session / "telemetry.jsonl", "a", encoding="utf-8", newline="")
            self._rows = 0
            self._images = 0
            self._stop_reason = ""
            return self.status()

    def log(self, record: Dict[str, Any], frame: Optional[np.ndarray] = None) -> bool:
        """Append one row. Returns False when inactive or the write failed."""
        with self._lock:
            if self._file is None or self._session_dir is None:
                return False
            row = dict(record)
            row.setdefault("timestamp", time.time())
            try:
                if (
                    frame is not None
                    and cv2 is not None
                    and self.frame_every > 0
                    and self._rows % self.frame_every == 0
                ):
                    image_name = f"frame-{self._rows:06d}.jpg"
                    ok, encoded = cv2.imencode(
                        ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                    )
                    if ok:
                        (self._session_dir / image_name).write_bytes(encoded.tobytes())
                        row["image"] = image_name
                        self._images += 1
                self._file.write(json.dumps(row, default=_json_default) + "\n")
                self._file.flush()
                self._rows += 1
            except OSError as exc:
                self._stop_reason = f"write failed: {exc}"
                logger.error("[TELEMETRY] %s; logging stopped", self._stop_reason)
                self._close()
                return False
            if self._rows % DISK_CHECK_EVERY_ROWS == 0 and self._disk_low():
                self._stop_reason = "disk space below limit"
                logger.warning("[TELEMETRY] Stopping: %s", self._stop_reason)
                self._close()
            return True

    def stop(self) -> Dict[str, Any]:
        with self._lock:
            self._close()
            return self.status()

    def status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "logging": self._file is not None,
                "rows": self._rows,
                "images": self._images,
                "session": str(self._session_dir) if self._session_dir else None,
                "stopped_reason": self._stop_reason or None,
            }

    def _close(self) -> None:
        if self._file is not None:
            self._file.close()
        self._file = None

    def _disk_low(self) -> bool:
        if self.min_free_bytes <= 0:
            return False
        try:
            return shutil.disk_usage(self.base_dir).free < self.min_free_bytes
        except OSError:
            return False


def _json_default(value: Any) -> Any:
    """Serialise numpy scalars that sneak into a telemetry row."""
    if isinstance(value, np.generic):
        return value.item()
    return str(value)
