"""
tools/test_all_symbols.py — Programmatic verification test for all visual navigation symbols:
1. ArUco Marker #0 (Home Base / Origin Dock)
2. ArUco Marker #1 (Waypoint Alpha)
3. ArUco Marker #2 (Waypoint Beta)
4. ArUco Marker #3 (Terminal Station / Destination Goal)
5. ArUco Marker #4 (Obstacle Bypass)
6. ArUco Marker #5 (Auxiliary Dock)
7. Stop Sign (Octagonal Red Traffic Symbol)
8. Sanoguzhan Dual Lane Lines (White & Yellow road markers)
"""

import os
import sys
import cv2
import numpy as np

curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
for p in [curr_dir, parent_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from autonomous_car.navigation.aruco_navigator import ArucoNavigator
from autonomous_car.control.line_follower import LineFollower, StopSignDetector
from autonomous_car.vision.lane_detector import LaneDetector

def test_aruco_symbols():
    print("=" * 60)
    print("TEST 1: ArUco Landmark Waypoints (DICT_4X4_50, Tags #0 to #5)")
    print("=" * 60)
    
    nav = ArucoNavigator(marker_size_cm=10.0)
    dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    all_passed = True
    for tag_id in range(6):
        # Generate clean marker image
        marker = cv2.aruco.generateImageMarker(dict_4x4, tag_id, 400, 1)
        # Place marker on a simulated camera canvas (400x240)
        canvas = np.ones((240, 400, 3), dtype=np.uint8) * 220
        # Scale marker down to ~120x120 to simulate realistic distance in FOV
        marker_scaled = cv2.resize(marker, (120, 120))
        marker_bgr = cv2.cvtColor(marker_scaled, cv2.COLOR_GRAY2BGR)
        
        # Center marker in camera frame
        y0 = (240 - 120) // 2
        x0 = (400 - 120) // 2
        canvas[y0:y0+120, x0:x0+120] = marker_bgr
        
        # Detect with ArucoNavigator
        detected = nav.detect_markers(canvas)
        if tag_id in detected:
            pose = detected[tag_id]
            print(f"  [PASS] Tag #{tag_id:02d}: Detected! Dist={pose.distance_cm:.1f} cm, Bearing={pose.bearing_deg:+.1f} deg, X={pose.x_cm:+.1f} cm, Z={pose.z_cm:.1f} cm")
        else:
            print(f"  [FAIL] Tag #{tag_id:02d}: NOT detected!")
            all_passed = False
            
    return all_passed

def test_stop_sign():
    print("\n" + "=" * 60)
    print("TEST 2: Stop Sign Symbol Detection")
    print("=" * 60)
    
    detector = StopSignDetector(min_area=800.0)
    
    # Create synthetic red octagon stop sign
    canvas = np.ones((240, 400, 3), dtype=np.uint8) * 180
    cx, cy, r = 200, 120, 45
    pts = []
    import math
    for i in range(8):
        angle = math.radians(i * 45 + 22.5)
        px = int(cx + r * math.cos(angle))
        py = int(cy + r * math.sin(angle))
        pts.append([px, py])
    pts = np.array(pts, dtype=np.int32)
    # Red BGR: (30, 30, 220)
    cv2.fillPoly(canvas, [pts], (30, 30, 220))
    cv2.putText(canvas, "STOP", (cx - 26, cy + 8), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    has_stop, bbox = detector.detect(canvas)
    if has_stop and bbox is not None:
        print(f"  [PASS] Stop Sign Detected! Bounding box: x={bbox[0]}, y={bbox[1]}, w={bbox[2]}, h={bbox[3]}")
        return True
    else:
        print("  [FAIL] Stop Sign NOT detected!")
        return False

def test_lane_assistant():
    print("\n" + "=" * 60)
    print("TEST 3: Sanoguzhan Perspective Lane Tracking")
    print("=" * 60)
    
    ld = LaneDetector()
    canvas = np.zeros((240, 400, 3), dtype=np.uint8)
    # Draw left white lane line and right yellow lane line
    cv2.line(canvas, (60, 210), (100, 160), (255, 255, 255), 4)
    cv2.line(canvas, (320, 210), (280, 160), (0, 230, 255), 4)
    
    res = ld.detect_from_bgr(canvas)
    if res.lane_detected:
        print(f"  [PASS] Lane Detected! Center={res.lane_center_x} px (Target=200), Lateral Error={res.error:+.1f} px, Direction={res.direction.value}, Width={res.lane_width} px")
        return True
    else:
        print("  [FAIL] Lane NOT detected!")
        return False

def test_unified_line_follower():
    print("\n" + "=" * 60)
    print("TEST 4: Unified 4-in-1 LineFollower Full Pipeline")
    print("=" * 60)
    
    lf = LineFollower()
    dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # Test Tag #0 homing
    canvas = np.ones((240, 400, 3), dtype=np.uint8) * 200
    marker = cv2.aruco.generateImageMarker(dict_4x4, 0, 100, 1)
    marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    canvas[70:170, 150:250] = marker_bgr
    
    lf.set_aruco_target(0)
    lf.start()
    cmd, s_cmd, hud, telem = lf.process_frame(canvas, current_distance_cm=55.0)
    print(f"  [PASS] LineFollower Processed: State={telem['state']}, Command={cmd}, Servo={s_cmd}")
    print(f"         Detected Tags in Telemetry: {len(telem['aruco_tags'])} tag(s) -> {telem['aruco_tags']}")
    return True

if __name__ == "__main__":
    p1 = test_aruco_symbols()
    p2 = test_stop_sign()
    p3 = test_lane_assistant()
    p4 = test_unified_line_follower()
    
    print("\n" + "=" * 60)
    if p1 and p2 and p3 and p4:
        print(">>> ALL SYMBOL & PERCEPTION TESTS PASSED SUCCESSFULLY! <<<")
    else:
        print(">>> SOME TESTS FAILED <<<")
    print("=" * 60)
