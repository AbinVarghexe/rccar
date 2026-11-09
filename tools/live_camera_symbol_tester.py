"""
tools/live_camera_symbol_tester.py — Interactive Inbuilt Webcam Symbol & Perception Tester.

Features:
- Connects directly to the inbuilt laptop/PC camera (cv2.VideoCapture(0)).
- Detects all ArUco Waypoint Symbols (Tags #0, #1, #2, #3, #4, #5) in real-time with 3D pose estimation.
- Detects Stop Signs (red octagons).
- Evaluates Sanoguzhan LKA Lane Tracking and perspective overlay.
- Visual HUD displays distance, bearing, and navigation recommendations.
- Interactive controls:
    [0-5] : Set target ArUco waypoint tag
    [H]   : Toggle between 3D HUD and Bird's-Eye Perspective
    [S]   : Save a test snapshot to disk
    [Q]   : Quit test
"""

import os
import sys
import time
import cv2
import numpy as np

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
for p in [curr_dir, parent_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from autonomous_car.control.line_follower import LineFollower, StopSignDetector
from autonomous_car.navigation.aruco_navigator import ArucoNavigator
from autonomous_car.vision.lane_detector import LaneDetector

def main():
    print("=" * 70)
    print("INBUILT CAMERA SYMBOL TESTER — Autonomous RC Car Perception Suite")
    print("=" * 70)
    print("Initializing inbuilt webcam (device 0)...")
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Could not open inbuilt webcam (device 0).")
        print("Please ensure your camera is not in use by another application.")
        return
        
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Initialize perception engines
    line_follower = LineFollower(marker_size_cm=10.0)
    line_follower.start()
    
    view_mode = "hud"  # 'hud' or 'birdseye'
    target_tag = 0
    line_follower.set_aruco_target(target_tag)
    
    print("\n[OK] Camera running! Show any ArUco tag, printed PDF, or stop sign to the camera.")
    print("Keys: [0-5] Set target tag | [H] Toggle HUD / Bird's Eye | [S] Save snapshot | [Q] Quit\n")
    
    fps_time = time.time()
    frame_count = 0
    fps = 0.0
    
    window_name = "Autonomous Car — Inbuilt Camera Symbol Tester"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[WARN] Failed to grab frame.")
            time.sleep(0.05)
            continue
            
        frame_count += 1
        now = time.time()
        if (now - fps_time) >= 1.0:
            fps = frame_count / (now - fps_time)
            frame_count = 0
            fps_time = now
            
        # Standardize for vehicle processing pipeline
        h, w = frame.shape[:2]
        proc_frame = cv2.resize(frame, (400, 240), interpolation=cv2.INTER_LINEAR)
        
        # Process through LineFollower
        cmd, servo_cmd, hud_frame, telem = line_follower.process_frame(proc_frame, current_distance_cm=50.0)
        
        if view_mode == "birdseye" and line_follower.latest_birdseye is not None:
            display_frame = line_follower.latest_birdseye.copy()
            view_label = "VIEW: SANOGUZHAN BIRD'S-EYE"
        else:
            display_frame = hud_frame.copy()
            view_label = "VIEW: 3D PERCEPTION HUD"
            
        # Draw Tester Overlay Top Banner
        cv2.rectangle(display_frame, (0, 0), (display_frame.shape[1], 28), (20, 20, 20), -1)
        cv2.putText(display_frame, f"INBUILT CAM | {fps:.1f} FPS | {view_label} | TARGET: TAG #{target_tag}",
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1, cv2.LINE_AA)
        
        # Draw Detected Symbols Summary on Bottom
        tags = telem.get("aruco_tags", [])
        if tags:
            tag_str = ", ".join([f"#{t['id']} ({t['dist_cm']:.0f}cm, {t['bearing_deg']:+.0f}°)" for t in tags])
            cv2.rectangle(display_frame, (0, display_frame.shape[0] - 22), (display_frame.shape[1], display_frame.shape[0]), (15, 60, 15), -1)
            cv2.putText(display_frame, f"DETECTED ARUCO: {tag_str}",
                        (8, display_frame.shape[0] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (50, 255, 50), 1, cv2.LINE_AA)
        else:
            cv2.rectangle(display_frame, (0, display_frame.shape[0] - 22), (display_frame.shape[1], display_frame.shape[0]), (30, 30, 30), -1)
            cv2.putText(display_frame, "NO ARUCO SYMBOL DETECTED — Present Tag #0-#5 to camera",
                        (8, display_frame.shape[0] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1, cv2.LINE_AA)
            
        # Enlarge for clear view on laptop screen
        large_display = cv2.resize(display_frame, (800, 480), interpolation=cv2.INTER_NEAREST)
        cv2.imshow(window_name, large_display)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('h'):
            view_mode = "birdseye" if view_mode == "hud" else "hud"
            print(f"[SWITCH] Mode: {view_mode}")
        elif key in [ord('0'), ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
            target_tag = int(chr(key))
            line_follower.set_aruco_target(target_tag)
            print(f"[TARGET] Switched target waypoint to Tag #{target_tag}")
        elif key == ord('s'):
            snap_path = os.path.join(curr_dir, f"snapshot_tag_test_{int(time.time())}.png")
            cv2.imwrite(snap_path, display_frame)
            print(f"[SNAPSHOT] Saved snapshot to: {snap_path}")
            
    cap.release()
    cv2.destroyAllWindows()
    print("[EXIT] Symbol tester terminated.")

if __name__ == "__main__":
    main()
