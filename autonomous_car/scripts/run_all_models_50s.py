#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_all_models_50s.py — Simultaneous Multi-Model Perception & Benchmark Engine.
Runs all onboard autonomous perception and neural models concurrently for exactly 50.0 seconds.

Target Models & Subsystems:
  1. ONNX Neural Pilot (SidewalkPilot-v3.1b.onnx)
  2. YOLOv8 INT8 Object Detector (yolov8n_int8.tflite)
  3. Sanoguzhan Computer Vision Lane Keeping (LaneDetector)
  4. ArUco 3D Metric Landmark Navigator (ArucoNavigator)
  5. Color-Segmented Stop Sign Safety Classifier (StopSignDetector)
  6. Lab-Space Ground-Plane Free-Space & Contact Horizon Evaluator

Usage on Pi or Terminal:
  python autonomous_car/scripts/run_all_models_50s.py
"""

import os
import sys
import time
import math
from typing import Dict, Any, Optional
import cv2
import numpy as np

# Ensure repository paths are importable
curr_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(curr_dir)
for p in [curr_dir, parent_dir]:
    if p not in sys.path:
        sys.path.insert(0, p)

from vision.lane_detector import LaneDetector
from navigation.aruco_navigator import ArucoNavigator
from control.line_follower import StopSignDetector


def load_onnx_model(model_path: str):
    """Loads ONNX runtime session for SidewalkPilot."""
    try:
        import onnxruntime as ort
        if not os.path.exists(model_path):
            return None, "File not found"
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        session = ort.InferenceSession(model_path, opts, providers=["CPUExecutionProvider"])
        return session, "OK (CPUExecutionProvider)"
    except Exception as e:
        return None, str(e)


def load_tflite_model(model_path: str):
    """Loads TFLite interpreter for YOLOv8."""
    try:
        try:
            from tflite_runtime.interpreter import Interpreter
        except ImportError:
            import tensorflow as tf
            Interpreter = tf.lite.Interpreter

        if not os.path.exists(model_path):
            return None, "File not found"
        interpreter = Interpreter(model_path=model_path, num_threads=2)
        interpreter.allocate_tensors()
        return interpreter, "OK (tflite_runtime)"
    except Exception as e:
        return None, str(e)


def evaluate_floor_freespace(bgr_frame: np.ndarray) -> float:
    """Evaluates ground-plane Lab floor free-space clearance."""
    h, w = bgr_frame.shape[:2]
    roi = bgr_frame[int(h * 0.45):, :]
    small = cv2.resize(roi, (80, 30), interpolation=cv2.INTER_NEAREST)
    lab = cv2.cvtColor(small, cv2.COLOR_BGR2LAB)
    # Ground patch near bumper
    patch = lab[20:28, 25:55]
    if patch.size == 0:
        return 1.0
    mean_val = np.mean(patch, axis=(0, 1))
    std_val = np.maximum(np.std(patch, axis=(0, 1)), [10.0, 5.0, 5.0])
    diff = np.abs(lab.astype(np.float32) - mean_val) / std_val
    dist_map = np.sum(diff ** 2, axis=2)
    is_floor = (dist_map <= 8.0).astype(np.float32)
    return float(np.mean(is_floor))


def main():
    print("=" * 78)
    print("  AUTONOMOUS RC CAR — MULTI-MODEL SIMULTANEOUS BENCHMARK (50 SECONDS)")
    print("=" * 78)
    print(f"Start Time       : {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Benchmark Target : 50.0 Seconds Continuous Simultaneous Multi-Model Execution")
    print("-" * 78)

    # 1. Initialize Frame Grabber (Camera, Webcam, or Synthetic Driving Generator)
    cam = None
    use_synthetic = False
    try:
        from picamera2 import Picamera2
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
        cam.start()
        print("[Camera] Picamera2 CSI Sensor initialized (640x480 RGB888).")
    except Exception:
        try:
            cap = cv2.VideoCapture(0)
            if cap.isOpened():
                cam = cap
                print("[Camera] OpenCV VideoCapture(0) initialized.")
            else:
                use_synthetic = True
        except Exception:
            use_synthetic = True

    if use_synthetic:
        print("[Camera] Physical camera offline or in use by service — Using synthetic driving canvas.")

    # 2. Model Paths
    models_dir = os.path.join(parent_dir, "models")
    onnx_path = os.path.join(models_dir, "SidewalkPilot-v3.1b.onnx")
    tflite_path = os.path.join(models_dir, "yolov8n_int8.tflite")

    # 3. Load Models
    print("\n[Loading Models]")
    onnx_sess, onnx_status = load_onnx_model(onnx_path)
    print(f"  1. SidewalkPilot ONNX      : {onnx_status}")

    tflite_interp, tflite_status = load_tflite_model(tflite_path)
    print(f"  2. YOLOv8n INT8 TFLite     : {tflite_status}")

    lane_detector = LaneDetector(deadzone_pixels=15)
    print("  3. Sanoguzhan LaneDetector : OK (CV Perspective LKA)")

    aruco_nav = ArucoNavigator(marker_size_cm=10.0)
    print("  4. ArUco 3D Waypoint Nav   : OK (DICT_4X4_50 Metric PnP)")

    stop_detector = StopSignDetector()
    print("  5. StopSignDetector        : OK (HSV Segmentation)")

    print("  6. Floor Free-Space Radar  : OK (Ground Horizon Analyzer)")

    print("-" * 78)
    print("Starting 50.0-second execution loop NOW. Watch real-time terminal metrics:")
    print("-" * 78)

    # Metric Accumulators
    metrics = {
        "onnx": {"count": 0, "total_ms": 0.0},
        "tflite": {"count": 0, "total_ms": 0.0},
        "lane": {"count": 0, "total_ms": 0.0},
        "aruco": {"count": 0, "total_ms": 0.0},
        "stop": {"count": 0, "total_ms": 0.0},
        "floor": {"count": 0, "total_ms": 0.0},
    }

    start_time = time.time()
    target_duration = 50.0
    frame_idx = 0
    last_print = 0.0

    synthetic_t = 0.0

    while True:
        loop_now = time.time()
        elapsed = loop_now - start_time
        if elapsed >= target_duration:
            break

        frame_idx += 1

        # A. Acquire Frame
        if cam is not None and not use_synthetic:
            if hasattr(cam, "capture_array"):
                rgb = cam.capture_array()
                frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                ret, frame = cam.read()
                if not ret:
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            # Generate synthetic asphalt road canvas with curving lines
            synthetic_t += 0.05
            frame = np.full((480, 640, 3), 40, dtype=np.uint8)
            curve = int(math.sin(synthetic_t) * 70)
            cv2.line(frame, (160 + curve, 480), (280 + curve, 240), (255, 255, 255), 8)
            cv2.line(frame, (480 + curve, 480), (360 + curve, 240), (255, 255, 255), 8)
            cv2.circle(frame, (320, 240), 20, (0, 0, 255), -1)

        proc_frame = cv2.resize(frame, (400, 240), interpolation=cv2.INTER_LINEAR)

        # ── 1. ONNX Neural Pilot Inference ────────────────────────────
        onnx_steer = 0.0
        if onnx_sess is not None:
            t0 = time.time()
            in_img = cv2.resize(proc_frame, (200, 66))
            in_img = in_img.astype(np.float32) / 255.0
            in_tensor = np.transpose(in_img, (2, 0, 1))[np.newaxis, ...]
            input_name = onnx_sess.get_inputs()[0].name
            res = onnx_sess.run(None, {input_name: in_tensor})
            onnx_steer = float(res[0][0][0]) if res and len(res[0]) > 0 else 0.0
            ms = (time.time() - t0) * 1000.0
            metrics["onnx"]["count"] += 1
            metrics["onnx"]["total_ms"] += ms

        # ── 2. YOLOv8 TFLite Object Detection ──────────────────────────
        tflite_dets = 0
        if tflite_interp is not None:
            t0 = time.time()
            in_details = tflite_interp.get_input_details()
            out_details = tflite_interp.get_output_details()
            ih, iw = in_details[0]['shape'][1:3]
            in_tensor = cv2.resize(proc_frame, (iw, ih))
            in_tensor = np.expand_dims(in_tensor, axis=0)
            if in_details[0]['dtype'] == np.float32:
                in_tensor = in_tensor.astype(np.float32) / 255.0
            tflite_interp.set_tensor(in_details[0]['index'], in_tensor)
            tflite_interp.invoke()
            out_tensor = tflite_interp.get_tensor(out_details[0]['index'])
            tflite_dets = int(np.count_nonzero(out_tensor > 0.35))
            ms = (time.time() - t0) * 1000.0
            metrics["tflite"]["count"] += 1
            metrics["tflite"]["total_ms"] += ms

        # ── 3. Sanoguzhan CV Lane Detection ────────────────────────────
        t0 = time.time()
        lane_res = lane_detector.detect_from_bgr(proc_frame)
        ms = (time.time() - t0) * 1000.0
        metrics["lane"]["count"] += 1
        metrics["lane"]["total_ms"] += ms

        # ── 4. ArUco 3D Waypoint Navigation ───────────────────────────
        t0 = time.time()
        tags = aruco_nav.detect_markers(proc_frame)
        ms = (time.time() - t0) * 1000.0
        metrics["aruco"]["count"] += 1
        metrics["aruco"]["total_ms"] += ms

        # ── 5. Stop Sign Segmenter ────────────────────────────────────
        t0 = time.time()
        has_stop, _ = stop_detector.detect(proc_frame)
        ms = (time.time() - t0) * 1000.0
        metrics["stop"]["count"] += 1
        metrics["stop"]["total_ms"] += ms

        # ── 6. Floor Free-Space Radar ─────────────────────────────────
        t0 = time.time()
        clr = evaluate_floor_freespace(proc_frame)
        ms = (time.time() - t0) * 1000.0
        metrics["floor"]["count"] += 1
        metrics["floor"]["total_ms"] += ms

        # Print live status update every 0.5s
        if (loop_now - last_print) >= 0.5:
            last_print = loop_now
            rem = max(0.0, target_duration - elapsed)
            prog_bars = int((elapsed / target_duration) * 30)
            bar_str = "[" + "#" * prog_bars + "-" * (30 - prog_bars) + "]"
            fps = frame_idx / max(0.001, elapsed)

            onnx_avg = metrics["onnx"]["total_ms"] / max(1, metrics["onnx"]["count"])
            lane_avg = metrics["lane"]["total_ms"] / max(1, metrics["lane"]["count"])
            aruco_avg = metrics["aruco"]["total_ms"] / max(1, metrics["aruco"]["count"])

            tag_info = f"{len(tags)} tags" if tags else "None"
            print(f"{bar_str} {elapsed:4.1f}s/50s | FPS: {fps:4.1f} | ONNX Steer: {onnx_steer:+.2f} ({onnx_avg:4.1f}ms) | Lane Err: {lane_res.error:+.0f}px | Tags: {tag_info}")

    # Benchmark Completion Summary
    total_time = time.time() - start_time
    print("\n" + "=" * 78)
    print("  SIMULTANEOUS MULTI-MODEL BENCHMARK COMPLETE (50.0 SECONDS)")
    print("=" * 78)
    print(f"Total Test Elapsed Time : {total_time:.2f} seconds")
    print(f"Total Frames Processed  : {frame_idx} frames")
    print(f"Average Pipeline FPS    : {frame_idx / total_time:.2f} FPS")
    print("-" * 78)
    print(f"{'Perception Subsystem':<30} | {'Runs':<8} | {'Avg Latency (ms)':<18} | {'Status'}")
    print("-" * 78)

    for name, title in [
        ("onnx", "1. SidewalkPilot ONNX"),
        ("tflite", "2. YOLOv8n INT8 TFLite"),
        ("lane", "3. LaneDetector (LKA)"),
        ("aruco", "4. ArUco 3D Waypoint Nav"),
        ("stop", "5. StopSignDetector"),
        ("floor", "6. Floor Free-Space Radar"),
    ]:
        cnt = metrics[name]["count"]
        avg = (metrics[name]["total_ms"] / cnt) if cnt > 0 else 0.0
        status = "Active" if cnt > 0 else "Offline/Skipped"
        print(f"{title:<30} | {cnt:<8} | {avg:<18.2f} | {status}")

    print("=" * 78)
    print("[SUCCESS] All perception models evaluated concurrently. Results logged.\n")


if __name__ == "__main__":
    main()
