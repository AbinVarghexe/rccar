"""
webrtc_stream.py — low-bandwidth WebRTC (H.264) camera stream for the dashboard.

MJPEG sends every frame as a standalone JPEG, which is heavy on Wi-Fi and stutters
when the link hiccups. WebRTC sends a compressed H.264 stream with congestion
control, so it stays smooth at a fraction of the bandwidth. The HUD overlay is already
drawn into the frames, so it comes along for free.

Everything here is optional: if aiortc is missing, AVAILABLE is False and the
dashboard keeps using MJPEG. The server runs its own asyncio loop in a daemon
thread so it never touches the camera / control threads.
"""

import asyncio
import fractions
import threading
import time
from typing import Callable, Dict, Optional

import numpy as np

try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
    from aiortc.codecs import h264 as _h264
    from aiortc.mediastreams import MediaStreamError
    from aiortc.rtcrtpsender import RTCRtpSender
    from av import VideoFrame
    AVAILABLE = True
except Exception:  # aiortc / av not installed
    AVAILABLE = False

VIDEO_CLOCK_RATE = 90000
MAX_PEERS = 2                    # one driver + one spare; older peers are dropped
TARGET_BITRATE = 600_000         # 400x240 looks clean at this rate
FrameSource = Callable[[str], Optional[np.ndarray]]


if AVAILABLE:
    class FrameTrack(VideoStreamTrack):
        """Pulls the newest BGR frame of one dashboard view at a fixed frame rate."""

        def __init__(self, get_frame: FrameSource, view: str, fps: float) -> None:
            super().__init__()
            self._get_frame = get_frame
            self._view = view
            self._period = 1.0 / fps
            self._t0 = time.monotonic()
            self._next = self._t0
            self._blank = np.zeros((240, 400, 3), dtype=np.uint8)

        async def recv(self) -> "VideoFrame":
            if self.readyState != "live":
                raise MediaStreamError
            wait = self._next - time.monotonic()
            if wait > 0:
                await asyncio.sleep(wait)
            self._next = max(self._next + self._period, time.monotonic() - self._period)

            img = self._get_frame(self._view)
            frame = VideoFrame.from_ndarray(img if img is not None else self._blank, format="bgr24")
            frame.pts = int((time.monotonic() - self._t0) * VIDEO_CLOCK_RATE)
            frame.time_base = fractions.Fraction(1, VIDEO_CLOCK_RATE)
            return frame


class WebRTCStreamer:
    def __init__(self, get_frame: FrameSource, fps: float = 15.0) -> None:
        self._get_frame = get_frame
        self._fps = fps
        self._peers: list = []
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ready = threading.Event()
        if not AVAILABLE:
            return
        try:  # the default 1 Mbit/s is wasteful for a 400x240 picture
            _h264.DEFAULT_BITRATE = TARGET_BITRATE
            _h264.MAX_BITRATE = TARGET_BITRATE * 2
        except Exception:
            pass
        threading.Thread(target=self._run_loop, daemon=True, name="webrtc-loop").start()
        self._ready.wait(2.0)

    @property
    def peer_count(self) -> int:
        return len(self._peers)

    def _run_loop(self) -> None:
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._ready.set()
        self._loop.run_forever()

    def offer(self, sdp: str, sdp_type: str, view: str, timeout: float = 10.0) -> Dict[str, str]:
        """Blocking helper for the HTTP thread: returns the SDP answer as a dict."""
        if not AVAILABLE or self._loop is None:
            raise RuntimeError("WebRTC unavailable")
        fut = asyncio.run_coroutine_threadsafe(self._handle_offer(sdp, sdp_type, view), self._loop)
        return fut.result(timeout)

    async def _handle_offer(self, sdp: str, sdp_type: str, view: str) -> Dict[str, str]:
        while len(self._peers) >= MAX_PEERS:
            await self._drop(self._peers[0])

        pc = RTCPeerConnection()
        self._peers.append(pc)

        @pc.on("connectionstatechange")
        async def _on_state() -> None:
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await self._drop(pc)

        sender = pc.addTrack(FrameTrack(self._get_frame, view, self._fps))
        # Prefer H.264 (cheapest to decode on phones/laptops); the browser may still pick another.
        try:
            caps = RTCRtpSender.getCapabilities("video").codecs
            h264 = [c for c in caps if c.mimeType == "video/H264"]
            rest = [c for c in caps if c.mimeType != "video/H264"]
            for t in pc.getTransceivers():
                if t.sender == sender and h264:
                    t.setCodecPreferences(h264 + rest)
        except Exception:
            pass

        await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=sdp_type))
        await pc.setLocalDescription(await pc.createAnswer())
        return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

    async def _drop(self, pc) -> None:
        if pc in self._peers:
            self._peers.remove(pc)
        try:
            await pc.close()
        except Exception:
            pass
