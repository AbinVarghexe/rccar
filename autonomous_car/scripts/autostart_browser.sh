#!/usr/bin/env bash
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/1000}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export DISPLAY="${DISPLAY:-:0}"

# Wait for the dashboard server to become healthy on port 5555
for i in $(seq 1 30); do
    if curl -s -o /dev/null -w "%{http_code}" http://localhost:5555/status 2>/dev/null | grep -q "200"; then
        break
    fi
    sleep 1
done

# Launch Chromium in app mode pointing to the local dashboard
exec /usr/bin/chromium \
    --noerrdialogs \
    --disable-infobars \
    --check-for-update-interval=31536000 \
    --app=http://localhost:5555
