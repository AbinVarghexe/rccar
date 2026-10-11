#!/bin/bash
# ==============================================================================
# Autonomous RC Car - Pi Software Stack Updater & Service Restarter
# ==============================================================================

set -e
echo "=== RC CAR DASHBOARD & CAMERA UPDATE SCRIPT ==="

# 1. Locate project root directory
CANDIDATE_DIRS=(
  "$(pwd)"
  "$HOME/autonomous_car"
  "$HOME/rccar/autonomous_car"
  "/home/alfin/autonomous_car"
  "/home/alfin/rccar/autonomous_car"
)

TARGET_DIR=""
for d in "${CANDIDATE_DIRS[@]}"; do
  if [ -d "$d" ] && [ -f "$d/main.py" ] && [ -d "$d/dashboard" ]; then
    TARGET_DIR="$d"
    break
  fi
done

if [ -z "$TARGET_DIR" ]; then
  echo "[!] Could not auto-detect autonomous_car folder. Using current dir $(pwd)"
  TARGET_DIR="$(pwd)"
fi

echo "[*] Target autonomous_car directory: $TARGET_DIR"
cd "$TARGET_DIR"

# 2. Stop any running rccar service or old python processes on port 5000 / 5001
echo "[*] Stopping existing services and processes on ports 5000 / 5001..."
sudo systemctl stop rccar.service 2>/dev/null || true
sudo fuser -k 5000/tcp 2>/dev/null || true
sudo fuser -k 5001/tcp 2>/dev/null || true
pkill -f "dashboard/app.py" 2>/dev/null || true
pkill -f "main.py" 2>/dev/null || true

# 3. Pull latest changes if git repository
if [ -d ".git" ] || [ -d "../.git" ]; then
  echo "[*] Updating via git..."
  git fetch origin master 2>/dev/null || git fetch origin 2>/dev/null || true
  git reset --hard origin/master 2>/dev/null || git pull origin master || true
fi

# 4. In case the folder was copied without .git, download the critical camera & dashboard files directly from GitHub
echo "[*] Ensuring critical camera and dashboard files are latest from GitHub..."
mkdir -p dashboard vision tests
curl -sSL "https://raw.githubusercontent.com/AbinVarghexe/rccar/master/autonomous_car/dashboard/app.py" -o "$TARGET_DIR/dashboard/app.py"
curl -sSL "https://raw.githubusercontent.com/AbinVarghexe/rccar/master/autonomous_car/dashboard/template.html" -o "$TARGET_DIR/dashboard/template.html"
curl -sSL "https://raw.githubusercontent.com/AbinVarghexe/rccar/master/autonomous_car/vision/camera.py" -o "$TARGET_DIR/vision/camera.py"
curl -sSL "https://raw.githubusercontent.com/AbinVarghexe/rccar/master/autonomous_car/config.py" -o "$TARGET_DIR/config.py"

# 5. Enable system-site-packages in virtualenv if present (critical for picamera2 apt package)
if [ -f "$TARGET_DIR/venv/pyvenv.cfg" ]; then
  echo "[*] Enabling system-site-packages in virtualenv..."
  sed -i 's/include-system-site-packages = false/include-system-site-packages = true/g' "$TARGET_DIR/venv/pyvenv.cfg" || true
fi

# 6. Load Raspberry Pi CSI V4L2 kernel driver (required for OV5647 CSI under OpenCV /dev/video0)
echo "[*] Loading bcm2835-v4l2 camera kernel module..."
sudo modprobe bcm2835-v4l2 2>/dev/null || true
if [ -f /etc/modules ] && ! grep -q "bcm2835-v4l2" /etc/modules; then
  echo "bcm2835-v4l2" | sudo tee -a /etc/modules >/dev/null 2>&1 || true
fi

# 7. Check camera hardware detection
echo ""
echo "=== CAMERA HARDWARE DIAGNOSTICS ==="
if command -v vcgencmd &>/dev/null; then
  echo -n "[vcgencmd]: "
  vcgencmd get_camera || true
fi
if command -v libcamera-hello &>/dev/null; then
  echo "[libcamera]:"
  libcamera-hello --list-cameras 2>/dev/null || true
elif command -v rpicam-hello &>/dev/null; then
  echo "[rpicam]:"
  rpicam-hello --list-cameras 2>/dev/null || true
fi
echo "[Video Nodes]:"
ls -l /dev/video* 2>/dev/null || echo "  [NOTE] No /dev/video* nodes active. If CSI camera is attached, check ribbon cable."
echo "===================================="
echo ""

# 8. Restart systemd service if installed, or start in background
if systemctl list-unit-files 2>/dev/null | grep -q "rccar.service"; then
  echo "[*] Restarting rccar.service..."
  sudo systemctl daemon-reload
  sudo systemctl restart rccar.service
  sleep 2
  sudo systemctl status rccar.service --no-pager || true
else
  echo "[*] Starting app via python3 dashboard/app.py..."
  nohup python3 dashboard/app.py > autonomous_car.log 2>&1 &
fi

IP=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "localhost")
echo ""
echo "==========================================================="
echo " Update Complete! Live Dashboard is now active at:"
echo "   http://$IP:5000"
echo "==========================================================="
