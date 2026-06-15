#!/bin/sh
# face_detect_demo_launcher.sh
#
# Deployed to: /usr/local/demo/application/face_detect/launch.sh
# Referenced by: 057-face-detect.yaml  (Script.Start: application/face_detect/launch.sh)
#
# This is a thin wrapper invoked by the OpenSTLinux demo launcher
# (demo_launcher.py), which runs as user "weston". Its only job is to
# figure out where the Wayland compositor socket actually lives at
# runtime (this can vary across boots / images) and export the correct
# XDG_RUNTIME_DIR / WAYLAND_DISPLAY before handing off to the real
# launch script.
#
# Deploy with:
#   scp face_detect_demo_launcher.sh root@<BOARD_IP>:/usr/local/demo/application/face_detect/launch.sh
#   ssh root@<BOARD_IP> "chmod +x /usr/local/demo/application/face_detect/launch.sh"

WESTON_UID=$(id -u weston 2>/dev/null || echo 1000)

for try_dir in "$XDG_RUNTIME_DIR" \
               "/run/user/$WESTON_UID" \
               "/run/user/0" "/run" "/tmp"; do
    [ -z "$try_dir" ] && continue
    if [ -S "$try_dir/wayland-0" ] || [ -S "$try_dir/wayland-1" ]; then
        export XDG_RUNTIME_DIR="$try_dir"
        break
    fi
done

[ -S "$XDG_RUNTIME_DIR/wayland-1" ] \
    && export WAYLAND_DISPLAY=wayland-1 \
    || export WAYLAND_DISPLAY=wayland-0

exec /home/weston/face_detect_launch.sh
