import json
import time
import logging
import threading
from typing import Optional, List, Dict, Any
import requests

from autonomous_car.ai.classes import DecisionResult, ActionEnum, DetectionResult, BoundingBox

logger = logging.getLogger(__name__)

class ServerAIClient:
    """
    Connects to the Edge AI Perception Server (e.g., self-driving-ai at port 8000).
    Receives remote high-power GPU detections and driving decisions.
    """

    def __init__(self, base_url: str = "http://127.0.0.1:8000", timeout: float = 0.5):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.is_connected = False
        self.server_latency_ms = 0.0

        self._latest_decision: Optional[DecisionResult] = None
        self._latest_detections: List[DetectionResult] = []
        self._last_update_time: float = 0.0

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._session = requests.Session()

    def update_server_url(self, new_url: str) -> None:
        """Update server base URL at runtime."""
        with self._lock:
            self.base_url = new_url.rstrip("/")
            self.is_connected = False
        logger.info(f"[SERVER AI CLIENT] Updated server URL to: {self.base_url}")

    def start(self) -> None:
        """Starts background poll thread to monitor server and fetch decisions."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True, name="server-ai-client")
        self._thread.start()
        logger.info(f"[SERVER AI CLIENT] Started monitoring Edge AI Server at {self.base_url}")

    def stop(self) -> None:
        self._running = False
        if self._session:
            try:
                self._session.close()
            except Exception:
                pass

    def _poll_loop(self) -> None:
        while self._running:
            try:
                t0 = time.time()
                url = f"{self.base_url}/decision"
                resp = self._session.get(url, timeout=self.timeout)
                elapsed_ms = (time.time() - t0) * 1000.0

                if resp.status_code == 200:
                    data = resp.json()
                    action_str = data.get("action", "FORWARD").upper()
                    reason = data.get("reason", "Edge server decision")
                    ts = float(data.get("timestamp", time.time()))

                    action = getattr(ActionEnum, action_str, ActionEnum.FORWARD)

                    with self._lock:
                        self.is_connected = True
                        self.server_latency_ms = round(elapsed_ms, 1)
                        self._last_update_time = time.time()
                        self._latest_decision = DecisionResult(
                            action=action,
                            reason=f"[Server Model] {reason}",
                            timestamp=ts,
                            hazard_detected=(action in (ActionEnum.STOP, ActionEnum.WAIT)),
                            source="server"
                        )
                else:
                    with self._lock:
                        self.is_connected = False
            except Exception:
                with self._lock:
                    self.is_connected = False

            # Poll frequency ~15-20 Hz
            time.sleep(0.06)

    def get_latest_decision(self, max_age_sec: float = 0.8) -> Optional[DecisionResult]:
        """Returns the latest server decision if it is fresh; returns None if stale or disconnected."""
        with self._lock:
            if not self.is_connected or self._latest_decision is None:
                return None
            if (time.time() - self._last_update_time) > max_age_sec:
                return None
            return self._latest_decision

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "server_url": self.base_url,
                "connected": self.is_connected,
                "latency_ms": self.server_latency_ms,
                "last_update": round(time.time() - self._last_update_time, 2) if self._last_update_time > 0 else -1
            }
