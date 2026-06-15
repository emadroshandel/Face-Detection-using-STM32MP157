#!/bin/sh
# face_detect_launch.sh  –  start server + dashboard, clean up on exit
# Launched by the Weston desktop icon (Terminal=false).

unset SDL_DYNAMIC_API        # prevent SDL warning from ST SDK environment

CASCADE=/home/root/haarcascade_frontalface_default.xml
SERVER=/home/root/face_detect_server.py
DASHBOARD=/home/root/face_detect_dashboard

# ── Cleanup function ──────────────────────────────────────────────────────
cleanup() {
    echo "[launch] stopping…"
    kill "$SERVER_PID" 2>/dev/null
    wait "$SERVER_PID" 2>/dev/null
    exit 0
}

# Trap Ctrl+C (SIGINT) and normal termination (SIGTERM)
trap cleanup INT TERM

# Kill any stale instances from a previous run
pkill -f face_detect_server.py 2>/dev/null
pkill -f face_detect_dashboard  2>/dev/null
sleep 1

# Start Python web server in background (log to /tmp)
python3 "$SERVER" --cascade "$CASCADE" > /tmp/face_detect_server.log 2>&1 &
SERVER_PID=$!

# Give the server 2 s to open the camera and start listening
sleep 2

# Launch the SDL2 dashboard (blocks until user taps "X Close" or kills it)
"$DASHBOARD"

# Dashboard exited — stop the server
cleanup
