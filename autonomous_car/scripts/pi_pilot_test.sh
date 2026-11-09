#!/bin/bash
# Temporary wheels-up pilot test. Run ON the Pi with rccar.service already
# stopped (main.py needs the camera and GPIO). Enables drive telemetry and the
# pilot for ~50 s, logs a status line every 2 s to /tmp/pilot_poll.log, then
# always disables the pilot and stops main.py by PID. It does NOT restart the
# service; the caller does that.
cd /home/alfin/autonomous_car || exit 1
API=http://127.0.0.1:5000/api/auto
LOG=/tmp/pilot_poll.log
: > "$LOG"

timeout 100 ./venv/bin/python3 main.py > /tmp/main_run.log 2>&1 &
MAIN_PID=$!

cleanup() {
    curl -s -m 3 -X POST -H 'Content-Type: application/json' \
        -d '{"enabled": false}' "$API/toggle" > /dev/null
    sleep 1
    kill "$MAIN_PID" 2> /dev/null
    wait "$MAIN_PID" 2> /dev/null
    echo "DONE" >> "$LOG"
}
trap cleanup EXIT

post() {
    curl -s -m 5 -X POST -H 'Content-Type: application/json' -d "$2" "$API/$1"
    echo
}

sleep 15
{
    post config '{"telemetry_enabled": true}' | cut -c1-80
    post toggle '{"enabled": true}' | cut -c1-120
} >> "$LOG"

for _ in $(seq 1 25); do
    curl -s -m 4 "$API/status" | ./venv/bin/python3 -c '
import json, sys, time
try:
    d = json.load(sys.stdin)
except Exception as exc:
    print(time.strftime("%H:%M:%S"), "status read failed:", exc)
    sys.exit()
us = d.get("ultrasonic") or {}
st = d.get("stage_latency_ms") or {}
stages = " ".join("%s=%s/%s" % (k, v["avg"], v["p95"]) for k, v in st.items())
print(time.strftime("%H:%M:%S"), "en=%s act=%s blind=%s sonar=%s/%s loop=%sms [%s] why=%s" % (
    d.get("autonomous_enabled"), d.get("action"), d.get("camera_blind"),
    us.get("state"), us.get("distance_cm"), d.get("latency_ms"), stages,
    (d.get("reason") or "")[:55]))
' >> "$LOG"
    sleep 2
done
