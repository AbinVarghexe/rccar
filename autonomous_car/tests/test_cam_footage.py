import sys
import os
import time
import cv2
import numpy as np

# Ensure autonomous_car is on path
curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
sys.path.insert(0, parent_dir)

from vision.camera import Camera, MEDIA_DIR

def run_tests():
    print("=" * 60)
    print("TESTING CAMERA INPUT AND FOOTAGE PIPELINE")
    print("=" * 60)

    # 1. Probe Hardware Webcams on host
    print("\n[STEP 1] Probing Host Physical Cameras via OpenCV...")
    found_hardware_cams = []
    for idx in range(3):
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW) if sys.platform == 'win32' else cv2.VideoCapture(idx)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                found_hardware_cams.append(idx)
                print(f"  -> Hardware Camera DETECTED at index {idx}: resolution {frame.shape[1]}x{frame.shape[0]}")
            cap.release()
        else:
            cap.release()

    if not found_hardware_cams:
        print("  -> No physical webcam detected. System will verify synthetic generator fallback.")

    # 2. Test Camera Class Initialization and Input
    print("\n[STEP 2] Initializing Camera System...")
    cam = Camera(width=640, height=480)
    cam.start()

    print(f"  -> Active Backend: {cam._backend}")
    print(f"  -> Initialized: {cam.is_initialized}")
    print(f"  -> Thread running: {cam._running}")

    # Wait for frame capture loop to generate first frame
    time.sleep(1.0)
    frame_bytes = cam.get_frame()

    if frame_bytes is None or len(frame_bytes) == 0:
        print("  [FAIL] No frame bytes captured!")
        cam.stop()
        return False
    
    print(f"  [PASS] Frame captured: {len(frame_bytes)} bytes")
    
    # Verify JPEG decoding
    img = cv2.imdecode(np.frombuffer(frame_bytes, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        print("  [FAIL] Failed to decode JPEG frame!")
        cam.stop()
        return False

    h, w, c = img.shape
    print(f"  [PASS] Decoded image shape: {w}x{h}, channels: {c}")

    # 3. Test Alignment Transforms
    print("\n[STEP 3] Testing Camera Transforms (Rotation & Flips)...")
    cam.set_alignment(flip_h=True, flip_v=False, rotation=90)
    time.sleep(0.3)
    t_frame_bytes = cam.get_frame()
    t_img = cv2.imdecode(np.frombuffer(t_frame_bytes, np.uint8), cv2.IMREAD_COLOR)
    print(f"  -> Rotated 90 deg shape: {t_img.shape[1]}x{t_img.shape[0]}")
    assert t_img.shape[0] == 640 and t_img.shape[1] == 480, "Rotation 90 failed to transpose dimensions"
    print("  [PASS] Alignment transform applied correctly.")

    # Reset alignment
    cam.set_alignment(flip_h=False, flip_v=False, rotation=0)
    time.sleep(0.2)

    # 4. Test MJPEG Stream Generator
    print("\n[STEP 4] Testing MJPEG Live Stream Generator...")
    gen = cam.generate_mjpeg()
    chunk = next(gen)
    assert b"--frame" in chunk and b"image/jpeg" in chunk, "MJPEG chunk header invalid"
    print(f"  [PASS] MJPEG stream generation operational ({len(chunk)} bytes in chunk)")

    # 5. Test Photo Footage Capture
    print("\n[STEP 5] Testing Photo Footage Capture...")
    photo_file = cam.capture_photo()
    if not photo_file:
        print("  [FAIL] Photo capture failed!")
        cam.stop()
        return False

    photo_path = os.path.join(MEDIA_DIR, photo_file)
    assert os.path.exists(photo_path), f"Photo file not found on disk: {photo_path}"
    photo_size = os.path.getsize(photo_path)
    print(f"  [PASS] Photo captured and saved: {photo_file} ({photo_size} bytes)")

    # 6. Test Video Footage Recording
    print("\n[STEP 6] Testing Video Footage Recording...")
    video_file = cam.start_recording()
    if not video_file:
        print("  [FAIL] Video recording failed to start!")
        cam.stop()
        return False

    print(f"  -> Recording in progress for 2.5 seconds to: {video_file}...")
    assert cam.is_recording is True, "cam.is_recording should be True"
    time.sleep(2.5)

    stopped_file = cam.stop_recording()
    assert stopped_file == video_file, "Stopped filename mismatch"
    assert cam.is_recording is False, "cam.is_recording should be False"

    # Wait for async disk flush
    time.sleep(1.0)
    video_path = os.path.join(MEDIA_DIR, video_file)
    assert os.path.exists(video_path), f"Video file not found on disk: {video_path}"
    video_size = os.path.getsize(video_path)
    print(f"  [PASS] Video footage saved: {video_file} ({video_size} bytes)")

    # 7. Test Media List
    print("\n[STEP 7] Testing Media Listing Index...")
    media_list = cam.list_media()
    print(f"  -> Found {len(media_list)} media items:")
    for item in media_list:
        print(f"     - {item['name']} ({item['type']}, {item['size_kb']} KB)")

    found_names = [m["name"] for m in media_list]
    assert photo_file in found_names, "Photo not listed in media_list"
    assert video_file in found_names, "Video not listed in media_list"
    print("  [PASS] Media list reflects captured footage.")

    # 8. Clean up camera
    cam.stop()
    print("\n[STEP 8] Camera stopped successfully.")
    print("=" * 60)
    print("ALL CAMERA INPUT AND FOOTAGE TESTS PASSED!")
    print("=" * 60)
    return True

if __name__ == "__main__":
    success = run_tests()
    if not success:
        sys.exit(1)
