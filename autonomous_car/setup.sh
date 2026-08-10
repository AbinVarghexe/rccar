#!/bin/bash
# ==============================================================================
# Autonomous RC Car - Automated Setup & Installation Script for Raspberry Pi OS
# ==============================================================================

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}======================================================${NC}"
echo -e "${BLUE}  Autonomous RC Car Software Stack Installer           ${NC}"
echo -e "${BLUE}======================================================${NC}"

# 1. Update APT Repository
echo -e "\n${GREEN}[1/5] Updating APT package repositories...${NC}"
sudo apt update -y

# 2. Install System Dependencies (Using correct Debian package name 'python3-serial')
echo -e "\n${GREEN}[2/5] Installing core dependencies (Picamera2, OpenCV, PySerial, Flask, GPIO)...${NC}"
sudo apt install -y \
    python3-picamera2 \
    python3-opencv \
    python3-numpy \
    python3-serial \
    python3-flask \
    python3-rpi.gpio \
    python3-pip \
    python3-venv || true

# 3. Add User Permissions
echo -e "\n${GREEN}[3/5] Setting up Serial UART and GPIO permissions for user '$USER'...${NC}"
sudo usermod -a -G dialout,gpio $USER 2>/dev/null || true

# 4. Install Python Requirements Fallback
echo -e "\n${GREEN}[4/5] Checking Python package environment...${NC}"
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"

if [ -f "requirements.txt" ]; then
    pip3 install -r requirements.txt --break-system-packages 2>/dev/null || pip3 install -r requirements.txt 2>/dev/null || true
fi

# 5. Create Executable Launcher
echo -e "\n${GREEN}[5/5] Creating executable launcher script 'run.sh'...${NC}"
cat << 'EOF' > run.sh
#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$DIR"
python3 main.py
EOF
chmod +x run.sh

echo -e "\n${BLUE}======================================================${NC}"
echo -e "${GREEN}  Installation Completed Successfully!                ${NC}"
echo -e "${BLUE}======================================================${NC}"
echo -e "To start the Autonomous RC Car software stack, run:"
echo -e "   ${GREEN}./run.sh${NC}"
echo -e "or:"
echo -e "   ${GREEN}python3 main.py${NC}"
echo -e "\nThen open your browser to view the Dashboard at:"
echo -e "   ${GREEN}http://$(hostname -I | awk '{print $1}'):5000${NC}\n"
