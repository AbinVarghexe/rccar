# Autonomous RC Car: Architecture Audit and Offline Rebuild Plan

Date: 2026-10-26

This document records the current evidence from the local checkout, the deployed Raspberry Pi, and the Raspberry Pi/LiteRT documentation reviewed during the audit. The first low-risk hardening changes are implemented locally and have not been deployed to the Pi; physical wiring and stationary tests still gate deployment.

## Executive finding

The car is not failing because the Raspberry Pi cannot run an offline model. The Pi is running Debian 13 on a 64-bit Raspberry Pi kernel, the OV5647 is discovered by `rpicam-hello`, Picamera2 starts successfully, GPIO is active, and LiteRT loads the INT8 YOLO model.

The current system is failing because too many partially overlapping architectures were merged into one control loop. The active runtime mixes a generic COCO detector, heuristic edge-density obstacle detection, lane assist, camera-panning search, dead-reckoning, monocular visual odometry/SLAM, optional server AI, optional paid AI, and a dashboard. They do not share one calibrated world model or one authoritative safety contract.

The replacement should be a small deterministic offline controller in which:

1. Ultrasonic distance and camera freshness can only reduce motion authority.
2. Free-space/lane geometry is the primary navigation signal.
3. YOLO is an advisory detector for semantic classes, not the primary collision sensor.
4. YOLO inference runs at a bounded low rate, with temporal confirmation.
5. A single planner produces one typed motion command.
6. A single safety arbiter validates every command immediately before GPIO output.
7. SLAM, GPS, cloud/server AI, paid AI, and waypoint claims are disabled until the car has encoders/IMU and a validated localization requirement.

## Evidence collected

### Raspberry Pi runtime

- `rccar.service` is active and has been running since 10:05:56 IST.
- Pi kernel: `6.18.34+rpt-rpi-v8`, `aarch64`; OS: Debian GNU/Linux 13 (trixie).
- Venv Python: 3.13.5, not the Python 3.11 described by the handover brief.
- Installed in the venv: NumPy, OpenCV, Flask, `RPi.GPIO`, Picamera2, and `ai_edge_litert`.
- Not installed: `tflite_runtime`, TensorFlow, Ultralytics.
- OV5647 is detected at 640x480/30 FPS by the native camera stack.
- Before the local hardening pass, current service status reported roughly 2.7 FPS and 382.8 ms latency while autonomy was disabled; the offline standby path now skips heavy perception when no autonomous task is active.
- Current service logs confirm Picamera2 starts and the INT8 LiteRT model loads.
- The model input is `[1, 320, 320, 3]` `int8`; the model output is `[1, 84, 2100]` `int8`.
- A current camera-frame smoke test measured approximately 106 ms for the model invoke alone, before corridor analysis, lane detection, SLAM, JPEG/HUD work, and Python post-processing.
- Current status returned an `obstacle` box from the heuristic corridor path, while the YOLO smoke test on the same current frame produced no useful detection at the configured confidence of 0.22.

### Local validation

- Python bytecode compilation passes.
- The full local suite now passes 121 tests, including regressions for destination priority, obstacle-stop priority, and invalid ultrasonic data.
- Tests also expose path-scan errors caused by invalid/non-image frame data reaching OpenCV in simulation paths.
- The checkout is heavily dirty and contains many untracked additions. Existing work must be preserved while the rebuild is staged.

## Critical defects

### 1. Competing architectures are active in one repository

The actual service entry point is:

`main.py` -> `hardware.motors` + `hardware.servo` + `vision.camera` + `navigation.autonomous_driver` + `sensors.ultrasonic` + `dashboard.app`.

The repository also contains competing, partially or fully unused systems:

- `control/*` and `hardware/*` motor/control layers;
- `ai/*` and `perception/*` detection layers;
- multiple navigation planners and state machines;
- `safety/safety_controller.py` and `control/safety_supervisor.py`;
- `slam/*`, behavioral cloning, server AI, paid AI, and a separate `self-driving-ai` server;
- a large `backup/obstacle_lane_stack` tree.

This makes it difficult to know which code is authoritative and lets tests pass against components that the Pi service never uses.

### 2. The control loop is overloaded and ignores its own rate configuration

`navigation/autonomous_driver.py` starts a background loop at approximately 35--40 Hz, but the enabled path still combines YOLO, corridor scanning, lane detection, traffic rules, risk analysis, object tracking, mapping, and ORB-based visual odometry. The local hardening pass now bounds YOLO through `yolo_inference_interval_sec` and skips the perception stack during manual standby; the remaining work is to move the rest of perception into separate bounded workers. The observed pre-fix result was about 2.7 FPS and 382.8 ms loop latency.

The loop also runs the expensive perception stack while `autonomous.enabled` is false. Manual mode should not spend most of the Pi CPU on a control decision that is not allowed to actuate motors.

### 3. YOLO preprocessing is not geometry-safe

`ai/tflite_detector.py` resizes the 640x480 camera image directly to 320x320. This changes the aspect ratio and makes object geometry vertically inconsistent. The output boxes are then scaled independently back to 640x480 without undoing a letterbox transform. A detector can still produce boxes, but their position and size are not trustworthy enough to drive collision decisions.

The local hardening pass now uses aspect-ratio-preserving letterbox preprocessing and carries the exact scale/padding metadata through post-processing. Model metadata and quantization parameters should still be read from the interpreter rather than relying on hard-coded assumptions.

### 4. Generic COCO YOLO is being asked to solve the wrong problem

The deployed model is a generic YOLOv8n COCO model. COCO does not provide a reliable `wall`, `floor`, `barrier`, or `free-space` representation for a low camera on a small indoor car. The code compensates with edge-density and colour heuristics, but those heuristics currently create the most visible obstacle decisions.

YOLO should be limited to semantic advisories such as person, bicycle, car, truck, animal, and stop sign. Collision avoidance should primarily use measured range plus a calibrated, temporally filtered free-space/ground geometry pipeline. If semantic detection is needed for the actual course, collect a small course-specific dataset and export a purpose-built INT8 model.

### 5. The promised safety contract is not actually wired through

`control/safety_supervisor.py` can reject stale or close ultrasonic readings. The local hardening pass now retains and passes the timestamped front `SingleSensorData` through `navigation/autonomous_driver.py::_execute_motor_action()`, and rejects invalid sensor samples instead of treating them as a clear path.

The live driver only keeps `_front_distance_cm` and `_rear_distance_cm` scalars. It must retain the complete timestamped sensor sample and pass the forward sample to the arbiter for every commanded motion.

### 6. Safety thresholds and hardware declarations disagree

The supplied `USER_MANUAL.md` is now the wiring authority. Its intended mapping is:

- L298N: GPIO17/18 -> IN1/IN2 and GPIO22/23 -> IN3/IN4;
- SG90 signal: GPIO12, with servo power from 5V and a shared ground;
- HC-SR04: GPIO24 -> TRIG and GPIO25 -> ECHO through the documented 1 kΩ/2 kΩ divider;
- common ground between the Pi, L298N, sensor divider, and servo supply.

This mapping matches the active `config.py` and the Pi `pinctrl` observation (GPIO24 output, GPIO25 input, and motor GPIOs configured as outputs). The older handover/AGENTS descriptions of GPIO23/24 sonar wiring and a rear sensor are superseded. Software still cannot prove the physical wires, the common ground, or the divider, so these must be checked before motor or autonomous tests.

The manual itself has two stale software claims that should not be used as runtime evidence: it says the HC-SR04 is disabled while the deployed Pi currently initializes it as enabled, and it describes GPIO12 PWM as hardware PWM even though the current RPi.GPIO implementation is software PWM. Its advertised 17.2 FPS/58 ms figure also does not match the observed Pi service behavior; the measured LiteRT invoke was about 106 ms and the full pre-fix loop was about 2.7 FPS.

The owner has now confirmed that the HC-SR04 is physically attached. A read-only check through the running dashboard confirmed `enabled=True`, GPIO24/25, an active service, and a live front reading of 82.2 cm. The divider, common ground, and sensor mounting should still be visually checked before motion testing.

Distance thresholds also differ between `SensorConfig`, `SafetyConfig`, the autonomous driver, and the safety supervisor. There must be one source of truth, with a sensor fault treated as `UNKNOWN` rather than silently equivalent to clear space.

### 7. Odometry, SLAM, GPS, and 3D target claims exceed the hardware

The car has no wheel encoders or IMU in the declared hardware. The code still exposes dead-reckoning, monocular VO/SLAM, GPS coordinates, 3D ground projection, and world-anchored destinations. These are estimates, not measured localization. They should not be used to claim destination arrival or to choose a path on an uncalibrated vehicle.

For the current hardware, autonomy should be reactive and local: follow a visible course or choose a short-horizon free-space direction, while reporting pose as `UNKNOWN` unless measured by an added sensor.

### 8. Servo and motor timing documentation overstates determinism

The code labels `RPi.GPIO.PWM()` as hardware PWM. RPi.GPIO's documented `PWM` API is software PWM; the RPi.GPIO project explicitly warns that it is not suitable for real-time/timing-critical work and does not provide hardware PWM through that API. GPIO12 is hardware-PWM-capable on the Pi, but the current implementation does not use a hardware PWM driver. Servo and motor timing should be treated as non-real-time until migrated to an appropriate GPIO/PWM backend.

### 9. Network/cloud code remains in the offline path

The active driver previously created and started `ServerAIClient` even in `local_ai` mode. The local hardening pass now starts it only for explicitly selected server modes. The `self-driving-ai` directory is a separate Ultralytics/FastAPI server architecture and is not a Pi-offline dependency. Paid AI, server AI, and ground-station heartbeat logic add failure modes and state transitions that are irrelevant to an offline car.

The offline profile should not open network AI clients at all. The dashboard can remain a local telemetry/manual-control surface, but no network response may be required for motion.

## Replacement architecture

### Runtime ownership

Use one runtime with these components only:

```text
Picamera2 latest-frame buffer -----> Perception @ 5-10 Hz ----+
                                                            |
HC-SR04 sensor worker @ 10 Hz ---> SensorState -------------+--> ReactivePlanner
                                                            |
Dashboard/manual commands --------> CommandSource ---------+
                                                            |
                                                            v
                                                  SafetyArbiter @ 10-20 Hz
                                                            |
                                                            v
                                                    MotorDriver + watchdog
```

Suggested modules:

- `runtime.py`: lifecycle and bounded worker loops;
- `camera.py`: Picamera2 capture only, one latest frame, explicit freshness;
- `sensors/ultrasonic.py`: timestamped filtered readings and health state;
- `perception/free_space.py`: calibrated ground ROI, lane/track geometry, left/center/right clearance;
- `perception/semantic_detector.py`: optional LiteRT model at 1--3 Hz, letterboxed and temporally smoothed;
- `navigation/reactive_planner.py`: hysteretic cruise/slow/stop/reverse/turn states;
- `control/safety_arbiter.py`: one final command gate;
- `hardware/motors.py` and a real PWM backend;
- `dashboard/app.py`: telemetry and manual override only.

### Decision policy for the current hardware

1. Stop if the camera is stale, the sensor is in a fault state, the emergency latch is set, or the forward range is below the hard stop threshold.
2. Slow when range is inside the warning band or free-space confidence is low.
3. Follow the course/free-space steering vector only when perception confidence is stable across multiple frames.
4. If the center is blocked, stop, take a bounded pan scan, choose the side with the higher measured/visual clearance, and execute a timed low-speed turn.
5. If both sides are unknown or blocked, hold stop. Do not guess.
6. Do not reverse unless a rear range sensor is installed and valid.
7. A semantic detector can add a stop or slow advisory, never remove a local safety stop.

### Performance budget

- Capture: 20--30 FPS, latest-frame only.
- Free-space/lane perception: 8--12 Hz at 320x240 or a small ROI.
- YOLO: 1--3 Hz, only when enabled and only as advisory.
- Planner/safety/motor command: 10--20 Hz.
- Dashboard JPEG/HUD: separate from the control loop.
- No ORB-SLAM, server AI, paid AI, or repeated JPEG encoding in the motion-critical path.

## Staged implementation plan

### Stage 0: Freeze and instrument

- Keep autonomy disabled by default.
- Add a startup self-test report: camera backend, model input/output signature, sensor health, GPIO mapping, motor calibration, and loop rates.
- Add a replay mode that consumes recorded frames and never touches GPIO.
- Add a command trace so every motor command records source, sensor state, and arbiter result.

### Stage 1: Make the existing runtime safe and measurable

- Gate expensive perception when autonomy is disabled.
- Pass timestamped ultrasonic data into the safety supervisor.
- Enforce one set of thresholds.
- Use the configured detector interval and expose actual inference rate.
- Fix path-scan frame handling and remove direct motor writes from scan code.
- Add tests for stale sensor, missing sensor, stale camera, and obstacle hysteresis.

### Stage 2: Replace the decision stack

- Introduce `ReactivePlanner` with explicit states and transition hysteresis.
- Make free-space geometry the navigation signal.
- Run semantic YOLO asynchronously at a low rate with temporal confirmation.
- Remove server/paid AI from the offline profile.

### Stage 3: Hardware calibration

- Verify the physical pinout against the active configuration.
- Verify each motor direction with the vehicle lifted, then calibrate trim at low speed.
- Calibrate camera orientation, camera height, pitch, field of view, and ground ROI.
- Measure HC-SR04 repeatability and define a fault policy.
- Use a deterministic PWM backend for the servo if jitter remains.

### Stage 4: Controlled deployment

- Run replay tests and dry-run tests locally.
- Push only the staged files using `push_to_pi.py --diff`.
- Restart the service with autonomy disabled.
- Perform stationary sensor/camera tests on the Pi.
- Test motors with wheels lifted and an operator at the emergency stop.
- Test autonomous motion at the lowest speed in a bounded course.

## Acceptance criteria

The rebuild is not ready for autonomous motion until all of these are true:

- The service starts offline with no server, cloud, or paid-AI process.
- The Pi reports the camera and sensor health explicitly.
- A stale or invalid sensor cannot be interpreted as clear path.
- Every motor command passes through the safety arbiter.
- Autonomy disabled means no autonomous motor commands and no full perception loop.
- Control loop stays within its target period under worst-case perception load.
- A replay suite covers clear path, false edge clutter, center obstacle, left/right opening, sensor timeout, stale camera, dark frame, and emergency stop.
- Manual and autonomous command ownership is mutually exclusive and visible in telemetry.
- The car stops and remains stopped when uncertainty is high.

### Implemented local hardening

- Manual standby skips the expensive autonomy pipeline.
- Local mode does not start a server-AI monitoring thread.
- Detector boxes use letterbox-aware geometry.
- Lane-assist corrections no longer override a confirmed obstacle stop or an active destination.
- YOLO inference is bounded to a configurable advisory cadence (0.35 seconds by default).
- Invalid ultrasonic samples now stop commanded motion.
- The camera exposes a latest-frame `capture_frame()` compatibility method for scan helpers.

## Sources reviewed

- Raspberry Pi camera software documentation: https://www.raspberrypi.com/documentation/computers/camera_software.html
- Picamera2 manual: https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
- LiteRT/TFLite Python Interpreter API: https://ai.google.dev/edge/api/tflite/python/tf/lite/Interpreter
- Ultralytics export and INT8/LiteRT guidance: https://docs.ultralytics.com/modes/export
- Raspberry Pi GPIO documentation: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
- RPi.GPIO PWM documentation and timing limitations: https://sourceforge.net/p/raspberry-gpio-python/wiki/PWM/
