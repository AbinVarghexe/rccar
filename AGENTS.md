# AGENTS.md — Autonomous RC Car Agent & Deployment Instructions

> **Operational Directive for AI Agents and Developers**  
> This file defines the mandatory protocol for managing, updating, and testing the Autonomous RC Car software stack on the physical Raspberry Pi.

---

## 1. Hardware & Remote SSH Connection Details

Agents connecting to the Raspberry Pi must use the following credentials and connection parameters:

| Parameter | Value |
| :--- | :--- |
| **Default Host IP** | `10.91.16.78` *(Subnet sweep via `autonomous_car/scripts/scan_network.py`)* |
| **SSH Port** | `22` |
| **SSH Username** | `alfin` |
| **SSH Password** | `123456` |
| **Remote Base Directory** | `/home/alfin/autonomous_car` |
| **Python Virtual Environment** | `/home/alfin/autonomous_car/venv/bin/python3` |
| **System Service** | `rccar.service` |
| **Live Web Dashboard** | `http://10.91.16.78:5000` |

---

## 2. Core Protocol: Direct Board Push via SSH (NO GitHub Pulls)

> [!IMPORTANT]
> **DO NOT fetch code from GitHub on the Raspberry Pi during active development.**  
> Never run `git pull origin master` or curl raw files from GitHub on the Pi for local iterative changes. All updates must be pushed **directly from the local workspace to the Pi over SSH/SFTP**.

### Why Direct Push?
1. Allows instant testing of uncommitted edits, rapid hotfixes, and experimental hardware tuning.
2. Prevents polluting Git history with micro-commits solely meant to transfer code to the car.
3. Eliminates delays, GitHub rate limits, and network latency on the Pi.

### How to Deploy:
Use the dedicated deployment tool [`push_to_pi.py`](file:///d:/DEV/Minor%20Project/rccar/autonomous_car/scripts/push_to_pi.py):

```bash
# Push only modified files (fast diff) and restart the service
python autonomous_car/scripts/push_to_pi.py --diff --restart

# Push all codebase files (full sync)
python autonomous_car/scripts/push_to_pi.py

# Push specific files
python autonomous_car/scripts/push_to_pi.py --files autonomous_car/hardware/motors.py autonomous_car/dashboard/app.py --restart

# Check service status on the Pi
python autonomous_car/scripts/push_to_pi.py --status
```

---

## 3. Deployment Tracking & Manifest

Every push to the Raspberry Pi is tracked locally in [`.pi_deploy_manifest.json`](file:///d:/DEV/Minor%20Project/rccar/.pi_deploy_manifest.json).

### What is Recorded:
- **File Relative Path**: Path relative to `autonomous_car/`
- **MD5 Checksum**: Hash of the file content at time of push
- **File Size**: Size in bytes
- **Timestamp**: ISO 8601 deployment time
- **Remote Target Path**: Absolute destination on the Pi (`/home/alfin/autonomous_car/...`)
- **Last Deploy Metadata**: Commit SHA, Git branch, total files pushed

This ensures agents can inspect the exact state of what exists on the vehicle hardware at any time.

---

## 4. Audit Log & Git Status Recording

All deployments automatically log their full context to [`pi_deployment.log`](file:///d:/DEV/Minor%20Project/rccar/pi_deployment.log).

### Captured Metrics in Every Push:
1. **Timestamp**: `YYYY-MM-DD HH:MM:SS`
2. **Current Git Branch**: (e.g. `master`)
3. **Current HEAD Commit**: Full 40-character SHA and short hash
4. **Git Dirty State**: Whether there are uncommitted changes
5. **Git Porcelain Status**: Exact list of modified (`M`), added (`A`), or untracked (`??`) files
6. **Transferred Files**: List of uploaded files and byte sizes
7. **Remote Execution Results**: Status of `rccar.service` restart or remote commands

---

## 5. Remote Execution & Service Management

Agents can execute remote commands on the Pi using [`remote_exec.py`](file:///d:/DEV/Minor%20Project/rccar/autonomous_car/scripts/remote_exec.py) or direct Paramiko SSH calls:

```bash
# Check service status and recent journal logs
python autonomous_car/scripts/remote_exec.py "systemctl status rccar.service --no-pager"

# View recent service logs
python autonomous_car/scripts/remote_exec.py "journalctl -u rccar.service -n 30 --no-pager"

# Restart the service manually
python autonomous_car/scripts/remote_exec.py "echo '123456' | sudo -S systemctl restart rccar.service"

# Run a motor pin diagnostic directly on the Pi
python autonomous_car/scripts/remote_exec.py "/home/alfin/autonomous_car/venv/bin/python3 /home/alfin/autonomous_car/scripts/test_motor_pins.py"
```

---

## 6. Hardware Pin Configuration Reference

When modifying control, motor, or sensor code, verify against the vehicle's physical pinout:

- **L298N Motor Driver (BCM GPIO)**:
  - `IN1`: GPIO 17
  - `IN2`: GPIO 18 (Default; GPIO 27 is the alternate legacy fallback)
  - `IN3`: GPIO 22
  - `IN4`: GPIO 23
- **Camera Tilt/Pan Servo**:
  - `Servo PWM`: GPIO 12 (Hardware PWM0 / Physical Pin 32)
- **Ultrasonic Distance Sensor**:
  - `TRIG`: GPIO 24
  - `ECHO`: GPIO 25 (with resistor voltage divider)
- **Camera**:
  - Raspberry Pi CSI Camera Module (OV5647 / IMX219 via `/dev/video0` or `libcamera`)

---

## 7. Mandatory CodeRabbit Automation & GitHub Verification Protocol

> [!IMPORTANT]
> **CodeRabbit Verification Requirement**  
> Every project, push, and Pull Request must run CodeRabbit automation before code is integrated into the production branches (`master` / `main`).

1. **Repository Configuration**: `.coderabbit.yaml` must always be present at the repository root with `auto_review: enabled: true`, path-specific safety instructions (hardware GPIO checks, inference constraints, navigation thresholds), and noise filters.
2. **Pull Request Review Workflow**:
   - For all major changes, push to a feature branch and open a Pull Request.
   - CodeRabbit automatically executes code review, architectural walkthrough, and automated checks.
   - Address any critical or high-severity issues flagged by CodeRabbit prior to merging.
3. **Automated Verification Pipeline**:
   - GitHub Actions CI workflow (`.github/workflows/verify.yml`) compiles Python files and executes linting/test checks.
   - CodeRabbit's `github-checks` tool ingests test results directly into the review thread.

