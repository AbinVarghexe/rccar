"""
Local Laptop Live Camera Receiver & Processor.
Connects to the Raspberry Pi 3 live video feed (http://10.252.39.181:5000/api/frame or /video_feed)
and provides real-time OpenCV frames on your laptop for computer vision & autonomous processing.
"""

import cv2
import numpy as np
import urllib.request
import time
from typing import Optional, Generator

class RemoteCameraClient:
    """
    Client for receiving live video frames from the Raspberry Pi 3 RC Car.
    """

    def __init__(self, pi_ip: str = "10.252.39.181", port: int = 5000) -> None:
        self.pi_ip: str = pi_ip
        self.port: int = port
        self.frame_url: str = f"http://{pi_ip}:{port}/api/frame"
        self.stream_url: str = f"http://{pi_ip}:{port}/video_feed"

    def get_latest_frame(self) -> Optional[np.ndarray]:
        """
        Fetches the single latest JPEG frame from the Pi over HTTP
        and decodes it into an OpenCV BGR numpy matrix.
        """
        try:
            url = f"{self.frame_url}?t={int(time.time() * 1000)}"
            req = urllib.request.urlopen(url, timeout=2)
            img_np = np.asarray(bytearray(req.read()), dtype=np.uint8)
            if len(img_np) > 0:
                frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
                return frame
        except Exception as e:
            print(f"[RECEIVER WARNING] Frame fetch error: {e}")
        return None

    def stream_frames(self) -> Generator[np.ndarray, None, None]:
        """
        Generator yielding continuous OpenCV BGR numpy frames for processing loop.
        """
        while True:
            frame = self.get_latest_frame()
            if frame is not None:
                yield frame
            time.sleep(0.03)  # ~30 FPS


def main():
    print("=======================================================================")
    print(" LAPTOP LIVE CAMERA STREAM RECEIVER & PROCESSOR")
    print(" Connecting to Pi at http://10.252.39.181:5000/api/frame...")
    print(" Press 'q' on the OpenCV window to exit.")
    print("=======================================================================")

    client = RemoteCameraClient(pi_ip="10.252.39.181", port=5000)

    for frame in client.stream_frames():
        # --- YOUR LOCAL COMPUTER VISION & AUTONOMOUS LOGIC HERE ---
        # e.g., cv2.Canny(frame, 100, 200) for lane detection
        # e.g., AI Object Detection / YOLO / TensorRT / HSV Color thresholding
        
        # Draw overlay showing receiver is active
        cv2.putText(
            frame, "LIVE PI CAM - LAPTOP RECEIVER ACTIVE", (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
        )

        cv2.imshow("RC Car Live Input (Laptop Processing Window)", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
