"""
Camera Module for Raspberry Pi 3 CSI Camera (OV5647 / Pi Camera v1).
Uses Picamera2 with direct hardware JPEG capture for non-blocking, fluid streaming.
Supports: JPEG photo capture, H264 video recording alongside live preview.
"""

import io
import os
import time
import logging
import threading
from datetime import datetime
from typing import Optional

logger = logging.getLogger(__name__)

USE_PICAMERA2 = False

try:
    from picamera2 import Picamera2
    USE_PICAMERA2 = True
except ImportError:
    logger.warning("Picamera2 not available. Camera disabled.")

# Media storage directory on the Pi
MEDIA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "media")


def _ensure_media_dir():
    os.makedirs(MEDIA_DIR, exist_ok=True)


class Camera:
    """
    Raspberry Pi CSI Camera — OV5647 @ 640x480.
    Captures JPEG frames via Picamera2 hardware encoder.
    Supports concurrent photo capture + H264 video recording.
    """

    def __init__(self, width: int = 640, height: int = 480) -> None:
        self.width:  int  = width
        self.height: int  = height

        self._camera:    Optional[object] = None
        self._lock       = threading.Lock()
        self._frame:     Optional[bytes]  = None
        self._running:   bool             = False
        self._thread:    Optional[threading.Thread] = None
        self.is_initialized: bool         = False

        # Recording state
        self._recording:    bool = False
        self._record_path:  Optional[str] = None
        self._record_lock   = threading.Lock()
        self._record_thread: Optional[threading.Thread] = None
        self._record_frames = []
        self._stop_record   = threading.Event()

        _ensure_media_dir()

    # ------------------------------------------------------------------ #
    def start(self) -> "Camera":
        """Initialize Picamera2 and start background capture thread."""
        if not USE_PICAMERA2:
            logger.warning("Picamera2 not installed. Camera disabled.")
            return self

        try:
            cam_info = Picamera2.global_camera_info()
        except Exception as e:
            logger.error(f"Could not query cameras: {e}")
            cam_info = []

        if not cam_info:
            logger.error("No cameras detected by libcamera.")
            return self

        print(f"[CAMERA] Detected {len(cam_info)} camera(s): {cam_info[0].get('Model', 'OV5647')}")

        try:
            self._camera = Picamera2(0)
            config = self._camera.create_video_configuration(
                main={"size": (self.width, self.height)}
            )
            self._camera.configure(config)
            self._camera.start()
            time.sleep(0.8)   # sensor warm-up

            self.is_initialized = True
            self._running = True
            self._thread = threading.Thread(
                target=self._capture_loop, daemon=True, name="camera-capture"
            )
            self._thread.start()
            print(f"[CAMERA READY] OV5647 @ {self.width}x{self.height} JPEG streaming active")

        except Exception as e:
            logger.error(f"Camera init failed: {e}")
            if self._camera:
                try:
                    self._camera.close()
                except Exception:
                    pass
            self._camera = None

        return self

    # ------------------------------------------------------------------ #
    def _capture_loop(self) -> None:
        """Continuously captures JPEG frames from Picamera2."""
        while self._running and self.is_initialized and self._camera:
            try:
                bio = io.BytesIO()
                self._camera.capture_file(bio, format="jpeg")
                jpeg_bytes = bio.getvalue()
                if jpeg_bytes:
                    with self._lock:
                        self._frame = jpeg_bytes
                    # Feed recording buffer if active
                    with self._record_lock:
                        if self._recording:
                            self._record_frames.append(jpeg_bytes)
            except Exception as e:
                logger.error(f"Frame capture error: {e}")
                time.sleep(0.1)
            time.sleep(0.02)   # ~40 fps cap

    # ------------------------------------------------------------------ #
    def get_frame(self) -> Optional[bytes]:
        """Returns the latest JPEG frame (thread-safe)."""
        with self._lock:
            return self._frame

    def generate_mjpeg(self):
        """Generator for MJPEG multipart streaming."""
        while self.is_initialized:
            frame = self.get_frame()
            if frame:
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + frame
                    + b"\r\n"
                )
            time.sleep(0.033)

    # ------------------------------------------------------------------ #
    #  PHOTO CAPTURE
    # ------------------------------------------------------------------ #
    def capture_photo(self) -> Optional[str]:
        """
        Save the current frame as a JPEG photo.
        Returns the filename (not full path) on success, None on failure.
        """
        frame = self.get_frame()
        if not frame:
            logger.error("No frame available for photo capture.")
            return None

        ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"photo_{ts}.jpg"
        path     = os.path.join(MEDIA_DIR, filename)

        try:
            with open(path, "wb") as f:
                f.write(frame)
            size_kb = len(frame) // 1024
            print(f"[PHOTO] Saved: {filename} ({size_kb} KB)")
            return filename
        except Exception as e:
            logger.error(f"Photo save failed: {e}")
            return None

    # ------------------------------------------------------------------ #
    #  VIDEO RECORDING  (MJPEG frames → .mjpeg file)
    # ------------------------------------------------------------------ #
    def start_recording(self) -> Optional[str]:
        """
        Start saving frames to a video file.
        Returns the filename or None if already recording / camera not ready.
        """
        with self._record_lock:
            if self._recording:
                return None
            if not self.is_initialized:
                return None

            ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"video_{ts}.mjpeg"
            self._record_path   = os.path.join(MEDIA_DIR, filename)
            self._recording     = True
            self._record_frames = []

        self._stop_record.clear()
        print(f"[VIDEO] Recording started → {filename}")
        return filename

    def stop_recording(self) -> Optional[str]:
        """
        Stop recording and flush frames to disk.
        Returns the filename or None.
        """
        with self._record_lock:
            if not self._recording:
                return None
            self._recording = False
            frames           = list(self._record_frames)
            self._record_frames = []
            path             = self._record_path

        if not path or not frames:
            return None

        filename = os.path.basename(path)

        # Write MJPEG file in background so response is instant
        def _flush():
            try:
                with open(path, "wb") as f:
                    for jpg in frames:
                        # MJPEG boundary
                        f.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                        f.write(jpg)
                        f.write(b"\r\n")
                size_mb = os.path.getsize(path) / (1024 * 1024)
                print(f"[VIDEO] Saved {filename}: {len(frames)} frames, {size_mb:.1f} MB")
            except Exception as e:
                logger.error(f"Video save failed: {e}")

        threading.Thread(target=_flush, daemon=True).start()
        return filename

    @property
    def is_recording(self) -> bool:
        with self._record_lock:
            return self._recording

    # ------------------------------------------------------------------ #
    def list_media(self) -> list:
        """List all captured media files with metadata."""
        files = []
        try:
            for fname in sorted(os.listdir(MEDIA_DIR), reverse=True):
                if fname.startswith(("photo_", "video_")):
                    fpath = os.path.join(MEDIA_DIR, fname)
                    stat  = os.stat(fpath)
                    files.append({
                        "name":    fname,
                        "type":    "photo" if fname.startswith("photo_") else "video",
                        "size":    stat.st_size,
                        "size_kb": round(stat.st_size / 1024, 1),
                        "mtime":   stat.st_mtime,
                    })
        except Exception as e:
            logger.error(f"Media list error: {e}")
        return files[:30]   # Return last 30 items

    # ------------------------------------------------------------------ #
    def stop(self) -> None:
        """Stop capture and release camera."""
        if self._recording:
            self.stop_recording()
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        if self._camera and self.is_initialized:
            try:
                self._camera.stop()
                self._camera.close()
            except Exception:
                pass
            self.is_initialized = False
            print("[CAMERA] Stopped.")
