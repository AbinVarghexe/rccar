#!/bin/bash
# Install autostart systemd service for RC Car on Raspberry Pi

SERVICE_FILE="/etc/systemd/system/rccar.service"

echo "[1] Installing systemd service file at $SERVICE_FILE..."
sudo cat << 'EOF' | sudo tee $SERVICE_FILE > /dev/null
[Unit]
Description=RC Car Autonomous Web Dashboard & Motor Control Service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=alfin
WorkingDirectory=/home/alfin/autonomous_car
ExecStart=/home/alfin/autonomous_car/venv/bin/python3 /home/alfin/autonomous_car/dashboard/app.py
Restart=always
RestartSec=5
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

echo "[2] Reloading systemctl daemon..."
sudo systemctl daemon-reload

echo "[3] Enabling rccar.service on boot..."
sudo systemctl enable rccar.service

echo "[4] Starting rccar.service..."
sudo systemctl restart rccar.service

echo "[OK] Autostart Service Installed Successfully! Checking Status:"
sudo systemctl status rccar.service --no-pager
