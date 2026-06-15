#!/bin/sh
# face_detect_launch.sh  –  start server + dashboard, clean up on exit
#
# Deployed to:  /home/weston/face_detect_launch.sh
# Launched by:  /usr/local/demo/application/face_detect/launch.sh
#               (the demo-launcher wrapper — see face_detect_demo_launcher.sh)
#
# IMPORTANT: this script (and everything it references) must be owned by
# the "weston" user, because the demo launcher runs as "weston", not root.
#   chown weston:weston /home/weston/face_detect_*
#   chmod +x /home/weston/face_detect_launch.sh /home/weston/face_detect_dashboard

# Log everything (stdout+stderr) so we can debug via `cat /tmp/face_launch.log`
exec >> /tmp/face_launch.log 2>&1
echo "=== [launch] $(date) user=$(id -un) ==="
echo "[launch] WAYLAND=${WAYLAND_DISPLAY} XDG=${XDG_RUNTIME_DIR}"

unset SDL_DYNAMIC_API        # prevent SDL warning from ST SDK environment

# Fall back to sane defaults if the caller didn't export these
export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u weston)}

CASCADE=/home/weston/haarcascade_frontalface_default.xml
SERVER=/home/weston/face_detect_server.py
DASHBOARD=/home/weston/face_detect_dashboard

# ── Cleanup function ──────────────────────────────────────────────────────
cleanup() {
    echo "[launch] stopping..."
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

echo "[launch] starting dashboard"

# Launch the SDL2 dashboard (blocks until user taps "X Close" or kills it)
"$DASHBOARD"
echo "[launch] dashboard exited $?"

# Dashboard exited — stop the server
cleanup
