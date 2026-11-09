"""Optional paid vision-decision client with a local safety boundary.

The endpoint is OpenAI-compatible (``/v1/chat/completions``). The cloud
model is advisory only: the autonomous driver applies local collision and
camera-health gates before accepting its action.
"""

import base64
import json
import logging
import threading
import time
from typing import Any, Dict, Optional

import cv2
import requests

from ai.classes import ActionEnum, DecisionResult

logger = logging.getLogger(__name__)


class PaidDecisionClient:
    def __init__(
        self,
        endpoint: str = "",
        api_key: str = "",
        model: str = "gpt-4o-mini",
        interval_sec: float = 1.0,
        timeout_sec: float = 4.0,
    ) -> None:
        self.endpoint = endpoint.rstrip("/")
        self.api_key = api_key
        self.model = model
        self.interval_sec = max(0.2, interval_sec)
        self.timeout_sec = max(0.5, timeout_sec)
        self._latest: Optional[DecisionResult] = None
        self._last_update = 0.0
        self._last_error = ""
        self._last_cost = 0.0
        self._pending: Optional[bytes] = None
        self._context: Dict[str, Any] = {}
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._session = requests.Session()

    @property
    def enabled(self) -> bool:
        return bool(self.endpoint and self.api_key)

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="paid-ai-client")
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        self._session.close()

    def submit_frame(self, frame, context: Dict[str, Any]) -> None:
        if not self.enabled or frame is None:
            return
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 55])
        if not ok:
            return
        with self._lock:
            self._pending = encoded.tobytes()
            self._context = dict(context)

    def get_latest_decision(self, max_age_sec: float = 3.0) -> Optional[DecisionResult]:
        with self._lock:
            if not self._latest or time.time() - self._last_update > max_age_sec:
                return None
            return self._latest

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "enabled": self.enabled,
                "connected": bool(self._latest and time.time() - self._last_update <= 3.0),
                "model": self.model,
                "last_update": round(time.time() - self._last_update, 2) if self._last_update else -1,
                "latency_ms": self._last_cost,
                "last_error": self._last_error,
            }

    def _loop(self) -> None:
        while self._running:
            started = time.monotonic()
            with self._lock:
                image = self._pending
                context = dict(self._context)
                self._pending = None
            if image:
                try:
                    decision = self._request(image, context)
                    with self._lock:
                        self._latest = decision
                        self._last_update = time.time()
                        self._last_error = ""
                        self._last_cost = round((time.monotonic() - started) * 1000.0, 1)
                except Exception as exc:
                    with self._lock:
                        self._last_error = str(exc)
                    logger.warning("[PAID AI] decision request failed: %s", exc)
            time.sleep(max(0.02, self.interval_sec - (time.monotonic() - started)))

    def _request(self, image: bytes, context: Dict[str, Any]) -> DecisionResult:
        prompt = (
            "You are a cautious RC-car driving policy. Return JSON only with keys "
            "action (FORWARD, LEFT, RIGHT, STOP, WAIT), reason, confidence. "
            "Choose STOP when uncertain or an obstacle is likely. Never invent sensor data. "
            f"Local perception context: {json.dumps(context, separators=(',', ':'))}"
        )
        payload = {
            "model": self.model,
            "temperature": 0,
            "max_tokens": 120,
            "response_format": {"type": "json_object"},
            "messages": [{
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url", "image_url": {
                        "url": "data:image/jpeg;base64," + base64.b64encode(image).decode("ascii")
                    }},
                ],
            }],
        }
        response = self._session.post(
            self.endpoint + "/v1/chat/completions",
            headers={"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"},
            json=payload,
            timeout=self.timeout_sec,
        )
        response.raise_for_status()
        data = response.json()
        content = data["choices"][0]["message"]["content"]
        parsed = json.loads(content) if isinstance(content, str) else content
        action_name = str(parsed.get("action", "STOP")).upper()
        action = getattr(ActionEnum, action_name, ActionEnum.STOP)
        return DecisionResult(
            action=action,
            reason=f"[Paid AI] {str(parsed.get('reason', 'Cloud decision'))[:240]}",
            hazard_detected=action in (ActionEnum.STOP, ActionEnum.WAIT),
            source="paid_ai",
        )
