# Face Detection App for STM32MP157

A real-time face detection application for the **STM32MP157** (ARM Cortex‑A7, OpenSTLinux). A USB webcam feed is analyzed with OpenCV, annotated with bounding boxes around detected faces, and shown **simultaneously**:

- on the board's **LCD touchscreen** via a native SDL2 dashboard, and
- in a **web browser** (any device on the same network) via an MJPEG stream.

I documented the **entire process from zero**, including every problem that came up during development and exactly how it was solved — so you can reproduce the build without hitting the same walls.

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Prerequisites](#2-prerequisites)
3. [System Architecture](#3-system-architecture)
4. [Challenges and Solutions](#4-challenges-and-solutions)
5. [Step-by-Step Build From Scratch](#5-step-by-step-build-from-scratch)
6. [Verification](#6-verification)
7. [Troubleshooting Reference](#7-troubleshooting-reference)
8. [Key Lessons Learned](#8-key-lessons-learned)

---

## 1. Project Overview

| Component | Details |
|---|---|
| Target board | STM32MP157 (ARM Cortex-A7, OpenSTLinux) |
| Display | 800 × 480 LCD with touchscreen, Weston/Wayland |
| Camera | USB webcam via V4L2 / GStreamer |
| Backend | Python 3 — HTTP server + OpenCV face detection |
| Frontend | C++ SDL2 dashboard on the LCD, plus a browser view on port 5000 |
| Board IP (example) | `192.168.1.111` — adjust to match your network |

### Application Files

| File | Purpose |
|---|---|
| `face_detect_server.py` | Python HTTP server: camera capture, OpenCV detection, MJPEG stream, BMP frames, JSON status |
| `face_detect_dashboard.cpp` | C++ SDL2 dashboard: live video on LCD, face indicator, stats, event log, close button |
| `face_detect_launch.sh` | Main launcher (deployed to `/home/weston/`): starts the server + dashboard together, handles clean shutdown, writes `/tmp/face_launch.log` |
| `057-face-detect.yaml` | Demo launcher YAML (deployed to `/usr/local/demo/gtk-application/`) — makes the app icon appear in the board's app grid |
| `face_detect_demo_launcher.sh` | Thin wrapper (deployed to `/usr/local/demo/application/face_detect/launch.sh`) — finds the Wayland socket at runtime, then hands off to `face_detect_launch.sh` |

---

## 2. Prerequisites

### 2.1 Hardware

- STM32MP157 Discovery board (or compatible) running OpenSTLinux
- 800 × 480 DSI/MIPI LCD with touchscreen
- USB webcam (UVC-compatible)
- Ethernet connection (example board IP: `192.168.1.111`)
- Ubuntu 20.04 / 22.04 build machine (x86_64)

### 2.2 Software on the Build Machine (Ubuntu)

- ST OpenSTLinux SDK installed (e.g. at `~/st-sdk/`)
- SSH access to the board as `root`
- `scp` for file transfer

### 2.3 Software Already on the Board (OpenSTLinux default image)

- Python 3 with OpenCV (`cv2`) and `numpy`
- SDL2 and SDL2_ttf libraries
- GStreamer with the `v4l2src` plugin
- `v4l2-ctl` utility
- Weston compositor (Wayland)
- LiberationSans TrueType fonts at `/usr/share/fonts/ttf/`

> **Note:** `SDL2_image` is **not** required. Frames are served as BMP and decoded with `SDL_LoadBMP_RW()`, which is part of core SDL2.

---

## 3. System Architecture

### 3.1 Data Flow

```
USB Webcam
    |
    v  GStreamer pipeline (v4l2src, io-mode=4, 640x480, BGR)
face_detect_server.py
    |
    |-- OpenCV Haar cascade face detection
    |-- Annotates frame (green rectangles)
    |
    |-- GET /video     -> MJPEG stream   (browser)
    |-- GET /status    -> JSON stats      (SDL2 dashboard polled every 500ms)
    |-- GET /rawframe  -> BMP frame       (SDL2 dashboard polled every 50ms)
    |-- GET /          -> HTML dashboard  (browser)

face_detect_dashboard (SDL2, C++)
    |-- rawframe_thread  polls /rawframe every 50ms  -> live video texture
    |-- poll_thread      polls /status  every 500ms  -> stats overlay
    |-- main loop        renders portrait layout, handles touch to close
```

### 3.2 Process Hierarchy at Runtime

```
weston (Wayland compositor, runs as user "weston")
  |
  +-- demo_launcher.py  (GTK app grid, runs as user "weston")
        |
        +-- launch.sh  (wrapper: /usr/local/demo/application/face_detect/)
              |
              +-- face_detect_launch.sh  (/home/weston/)
                    |
                    +-- face_detect_server.py  (background, port 5000)
                    +-- face_detect_dashboard  (SDL2 window on LCD)
```

### 3.3 GStreamer Camera Pipeline

This is the pipeline that makes the webcam work reliably on this board:

```
v4l2src device=/dev/video0 io-mode=4 !
video/x-raw,width=640,height=480 !
videoconvert !
video/x-raw,format=BGR !
appsink drop=1 max-buffers=1 sync=false
```

> **Key fact:** `io-mode=4` (DMA buffer mode) is **required** for reliable capture on this board. Without it, the camera either fails to open or produces corrupted frames. This was discovered by examining the board's built-in Camera app pipeline.

---

## 4. Challenges and Solutions

This section documents every significant problem encountered during development and how each was solved. Reading it carefully will save you hours of debugging.

### Challenge 1 — Wrong Camera Pipeline

**Problem:** Using the default OpenCV `VideoCapture` or a basic `v4l2src` pipeline without `io-mode=4` causes the camera to fail or return garbled frames on this board.

**Solution:** Use `io-mode=4` (DMA buffer) in the GStreamer pipeline. Also call `v4l2-ctl --device=/dev/video0 --set-parm=30` before opening the camera to set the framerate. Resolution must be **640×480** — the board's camera hardware requires this specific resolution.

```python
# In face_detect_server.py — before opening camera:
subprocess.run(["v4l2-ctl", f"--device={device}", "--set-parm=30"], ...)

# GStreamer pipeline string:
gst = (f"v4l2src device={device} io-mode=4 ! "
       f"video/x-raw,width=640,height=480 ! "
       f"videoconvert ! video/x-raw,format=BGR ! "
       f"appsink drop=1 max-buffers=1 sync=false")
cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
```

---

### Challenge 2 — SDL2_image Not Available on the Board

**Problem:** The initial design served JPEG frames to the SDL2 dashboard and decoded them with `SDL_image`. But `SDL2_image` development headers were not available in the ST SDK sysroot, making cross-compilation impossible.

**Solution:** Serve frames as **BMP** instead. BMP decoding is built into SDL2 core via `SDL_LoadBMP_RW()` — no extra library needed. The server encodes each annotated frame as BMP with `cv2.imencode(".bmp", frame)` and serves it at the `/rawframe` endpoint.

```python
# In face_detect_server.py — dual encoding per frame:
ok_j, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
ok_b, bmp  = cv2.imencode(".bmp", frame)
if ok_j and ok_b:
    set_frame(jpeg.tobytes(), bmp.tobytes())
```

```cpp
// In face_detect_dashboard.cpp — BMP decode, no extra library:
SDL_RWops   *rw   = SDL_RWFromConstMem(frame_data, frame_size);
SDL_Surface *surf = SDL_LoadBMP_RW(rw, 1);   // 1 = auto-free rw
video_tex = SDL_CreateTextureFromSurface(renderer, surf);
```

---

### Challenge 3 — BMP Frame Buffer Too Small

**Problem:** A 640×480 uncompressed BMP is roughly 900 KB. The initial frame buffer was allocated at 512 KB, causing a buffer overflow as soon as the first real frame arrived — crashing or corrupting the dashboard.

**Solution:** Allocate **2 MB** for the frame buffer — large enough for any standard webcam-resolution BMP.

```cpp
// In face_detect_dashboard.cpp:
#define MAX_BMP_BYTES (2 * 1024 * 1024)   // 2 MB covers 640x480 BMP (~900 KB)

struct FrameStore {
    uint8_t  data[MAX_BMP_BYTES];
    int      size;
    uint32_t version;
};
```

---

### Challenge 4 — Demo Launcher Ignores `.desktop` Files

**Problem:** The face-detect icon does not appear in the board's application grid after creating a standard `.desktop` file. The board's demo launcher (`demo_launcher.py`) does **not** read `.desktop` files from `/usr/share/applications/`. It reads YAML files from `/usr/local/demo/gtk-application/` instead — completely different from a standard Linux desktop.

**Solution:** Create a YAML file at `/usr/local/demo/gtk-application/057-face-detect.yaml` following the format used by other apps. The `.desktop` file is irrelevant to the demo launcher.

```yaml
# /usr/local/demo/gtk-application/057-face-detect.yaml
Application:
    Name: Face Detect
    Description: live webcam
    Icon: pictures/face_detect.png
    Board:
        List: all
    Type: script
    Script:
        Start: application/face_detect/launch.sh
```

> **Key fact:** The `Script.Start` path **must be relative** to `/usr/local/demo/`. An absolute path (starting with `/`) causes Python's `os.path.join` to ignore the demo-path prefix, which silently breaks script execution from the demo launcher.

---

### Challenge 5 — Demo Launcher Runs as User `weston`, Not `root`

**Problem:** The app icon was visible in the demo launcher, but tapping it did nothing — not even a log file was created. All application files (`face_detect_dashboard`, `face_detect_server.py`, `face_detect_launch.sh`, the Haar cascade XML) were placed in `/home/root/`. The demo launcher actually runs as the **`weston` user (UID 1000)**, and `/home/root/` has mode `700` — accessible only to `root`. The demo launcher's attempt to execute `/home/root/face_detect_launch.sh` failed with a silent permission error.

**Solution:** Move **all** application files to `/home/weston/` and set ownership to `weston:weston`. Note that I updated every path reference in every script accordingly.

```sh
# Correct file locations (accessible by the weston user):
/home/weston/face_detect_dashboard          (binary, chmod +x)
/home/weston/face_detect_server.py          (Python server)
/home/weston/face_detect_launch.sh          (launcher script)
/home/weston/haarcascade_frontalface_default.xml

# Set ownership:
chown weston:weston /home/weston/face_detect_*
chown weston:weston /home/weston/haarcascade_frontalface_default.xml
```

---

### Challenge 6 — Wrong Wayland Runtime Directory After Reboot

**Problem:** The app worked when the demo launcher was started manually, but failed after a reboot. The SDL2 dashboard connects to the Wayland compositor via the socket at `$XDG_RUNTIME_DIR/wayland-0`. After a fresh boot, the system-started demo launcher sometimes had `XDG_RUNTIME_DIR` unset or pointing at `/run` instead of `/run/user/1000` (where Weston actually creates its socket). The server would start fine, but the dashboard immediately crashed with a Wayland connection error.

**Solution:** A wrapper script at `/usr/local/demo/application/face_detect/launch.sh` dynamically locates the Wayland socket at runtime by checking common candidate paths, then exports the correct `XDG_RUNTIME_DIR` before calling the main launch script.

```sh
#!/bin/sh
# /usr/local/demo/application/face_detect/launch.sh
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
```

---

### Challenge 7 — Close Button Does Not Respond to Touch

**Problem:** I considered a "X Close" button for the app to close the app when required. Tapping the "X Close" button on the LCD had no effect. The touchscreen on this board reports touch events in a coordinate space (normalized 0.0–1.0) that can be **rotated relative to the rendered display** (800×480 landscape). The button was drawn in only the bottom 8% of the screen (`BTN_Y = 438` on a 480-tall screen), and touch Y coordinates near the bottom did not always land in that range due to the rotation.

**Solution:** Two fixes were applied:

1. Increase the button height from 8% to **18%** of the screen height — much easier to hit.
2. Check **both** the Y coordinate (bottom of screen, normal orientation) **and** the X coordinate (right edge, covering a 90°-rotated touch surface).

Also add `SIGTERM`/`SIGINT` signal handlers so the dashboard can be killed cleanly over SSH.

```cpp
// In face_detect_dashboard.cpp — button area:
int BTN_H = SH * 18 / 100;   // was 8% — now 18% for easy touch
int BTN_Y = SH - P - BTN_H;

// Touch detection covers both orientations:
auto in_btn = [&](float fx, float fy) -> bool {
    return (int)(fy * SH) >= BTN_Y ||    // normal landscape
           (int)(fx * SW) >= (SW * 4/5); // rotated 90 degrees
};

// Signal handler for clean exit via pkill / Ctrl+C:
static volatile sig_atomic_t g_quit_signal = 0;
static void sig_handler(int) { g_quit_signal = 1; }

// In main():
signal(SIGTERM, sig_handler);
signal(SIGINT,  sig_handler);

// In the event loop:
if (g_quit_signal) { running = false; continue; }
```

---

### Challenge 8 — App Does Not Stop Cleanly / Server Left Running

**Problem:** When I closed the SDL2 dashboard, `main()` returned, but the Python server (started in the background by the launch script) kept running. The camera stayed occupied and port 5000 remained bound, preventing a clean restart.

**Solution:** Add a `system()` call at the end of `main()` to kill the server, and add a `trap cleanup` in the launch script. Both `SIGINT` and `SIGTERM` are handled.

```cpp
// At the very end of main() in face_detect_dashboard.cpp:
fprintf(stderr, "[face-dashboard] stopping server...\n");
system("pkill -f face_detect_server.py 2>/dev/null");
```

```sh
# In face_detect_launch.sh:
cleanup() {
    kill "$SERVER_PID" 2>/dev/null
    wait "$SERVER_PID" 2>/dev/null
    exit 0
}
trap cleanup INT TERM
```

---

## 5. Step-by-Step Build From Scratch

Follow these steps in order. Every command is shown exactly as you should type it. Commands run from your **Ubuntu build machine** are marked accordingly; everything else runs **on the board via SSH**. Replace `192.168.1.111` with your board's actual IP address.

### Step 1 — Prepare the Cascade File on the Board

The Haar cascade XML file for face detection must be placed on the board before the server can run.

1. Download the file on your Ubuntu machine:

   ```sh
   wget https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
   ```

2. Copy it to the board (run from Ubuntu):

   ```sh
   scp haarcascade_frontalface_default.xml root@192.168.1.111:/home/weston/
   ssh root@192.168.1.111 "chown weston:weston /home/weston/haarcascade_frontalface_default.xml"
   ```

### Step 2 — Deploy the Python Server

1. Copy `face_detect_server.py` to the board:

   ```sh
   scp face_detect_server.py root@192.168.1.111:/home/weston/
   ssh root@192.168.1.111 "chown weston:weston /home/weston/face_detect_server.py"
   ```

2. Test that the server runs (SSH to the board):

   ```sh
   python3 /home/weston/face_detect_server.py \
       --cascade /home/weston/haarcascade_frontalface_default.xml &
   sleep 3
   curl http://localhost:5000/status
   pkill -f face_detect_server.py
   ```

### Step 3 — Cross-Compile the SDL2 Dashboard

Run all compilation steps on your **Ubuntu build machine**, not on the board.

1. Source the ST SDK environment:

   ```sh
   source ~/st-sdk/environment-setup-cortexa7t2hf-neon-vfpv4-ostl-linux-gnueabi
   ```

2. Compile (notice: no `-lSDL2_image` needed):

   ```sh
   arm-ostl-linux-gnueabi-g++ -mfloat-abi=hard -mfpu=neon-vfpv4 \
     --sysroot=$SDKTARGETSYSROOT \
     -I$SDKTARGETSYSROOT/usr/include/SDL2 \
     face_detect_dashboard.cpp \
     -L$SDKTARGETSYSROOT/usr/lib \
     -lSDL2 -lSDL2_ttf -lpthread -lm \
     -Wl,-rpath-link,$SDKTARGETSYSROOT/usr/lib \
     -o face_detect_dashboard
   ```

3. Deploy the binary to the board:

   ```sh
   scp face_detect_dashboard root@192.168.1.111:/home/weston/
   ssh root@192.168.1.111 "chown weston:weston /home/weston/face_detect_dashboard && chmod +x /home/weston/face_detect_dashboard"
   ```

### Step 4 — Deploy the Launch Script

1. Copy the main launch script:

   ```sh
   scp face_detect_launch.sh root@192.168.1.111:/home/weston/
   ssh root@192.168.1.111 "chown weston:weston /home/weston/face_detect_launch.sh && chmod +x /home/weston/face_detect_launch.sh"
   ```

### Step 5 — Create the Demo Launcher Integration

Step 1 runs on the board via SSH. Steps 2–3 are run from your **Ubuntu build machine** (`scp` + a couple of follow-up SSH commands), using the `face_detect_demo_launcher.sh` and `057-face-detect.yaml` files included in this repo.

1. Copy an icon for the app (on the board via SSH):

   ```sh
   ssh root@192.168.1.111 "cp /usr/share/pixmaps/camera_preview.png /usr/local/demo/pictures/face_detect.png"
   ```

2. Deploy the wrapper script:

   ```sh
   ssh root@192.168.1.111 "mkdir -p /usr/local/demo/application/face_detect/"
   scp face_detect_demo_launcher.sh root@192.168.1.111:/usr/local/demo/application/face_detect/launch.sh
   ssh root@192.168.1.111 "chmod +x /usr/local/demo/application/face_detect/launch.sh"
   ```

3. Deploy the YAML for the demo launcher:

   ```sh
   scp 057-face-detect.yaml root@192.168.1.111:/usr/local/demo/gtk-application/
   ```

4. Reboot the board — the icon appears automatically after boot:

   ```sh
   ssh root@192.168.1.111 reboot
   ```

---

## 6. Verification

### 6.1 Test the Web Dashboard

Open a browser and navigate to:

```
http://192.168.1.111:5000
```

You should see a live, annotated MJPEG stream. Hold your face in front of the webcam — a green rectangle should appear around it.

### 6.2 Test the LCD Dashboard

On the board's touchscreen, tap the **Face Detect** icon in the demo launcher grid. After roughly 3 seconds (server startup time), the SDL2 dashboard appears, showing the live video feed, a face indicator, and statistics.

### 6.3 Test the Close Button

Tap the large red **"X Close"** button at the bottom of the LCD. The dashboard should close and the Python server should also stop (port 5000 becomes unreachable). If the button doesn't respond after 2–3 tries, use SSH:

```sh
pkill -f face_detect_dashboard   # stops dashboard (server auto-stops too)
```

### 6.4 Check Logs

```sh
cat /tmp/face_launch.log         # launch script output
cat /tmp/face_detect_server.log  # Python server output (GStreamer, OpenCV)
```

---

## 7. Troubleshooting Reference

| Symptom | Fix |
|---|---|
| "Camera connecting..." stays > 5 s on LCD | Check `face_detect_server.log` for GStreamer errors. Confirm the webcam is `/dev/video0`. Try unplugging and replugging it. |
| Server log shows "cannot open camera" | Make sure no other app is using the camera. Run `pkill -f face_detect_server.py`, then restart. |
| Icon not in demo launcher after reboot | Verify the YAML file exists: `ls /usr/local/demo/gtk-application/057-face-detect.yaml` |
| Tapping icon does nothing, no log created | Files must be owned by the `weston` user. Run `chown weston:weston /home/weston/face_detect_*` |
| SDL2 dashboard: black screen / "Server offline" | The server is not running or the Wayland env is wrong. Check `/tmp/face_launch.log` for the `XDG_RUNTIME_DIR` value. |
| Cannot close app from LCD | Use SSH: `pkill -f face_detect_dashboard`. Then rebuild with the larger button (`BTN_H = 18%`). |
| Browser stream works, LCD shows "Server offline" | Dashboard connected to Wayland but cannot reach the HTTP server. Both run on `localhost:5000` — check firewall or the python process. |
| Compilation fails: `SDL_image.h` not found | Do **not** link `-lSDL2_image`. Remove all SDL2_image references. Use `SDL_LoadBMP_RW()` for frame decoding. |
| App fails after reboot but works from SSH | Check file ownership (must be `weston:weston`). Also verify the wrapper script dynamically finds the Wayland socket. |

---

## 8. Key Lessons Learned

### 8.1 OpenSTLinux-Specific Facts

- The demo launcher (`demo_launcher.py`) reads YAML files from `/usr/local/demo/gtk-application/` — **not** `.desktop` files from `/usr/share/applications/`.
- The demo launcher and everything it launches runs as user **`weston`** (UID 1000) — not `root`.
- All application files must be accessible to the `weston` user. Place them in `/home/weston/`, not `/home/root/`.
- The Wayland compositor socket is at `/run/user/1000/wayland-0` (1000 = UID of the `weston` user).
- Script paths in the YAML must be **relative** to `/usr/local/demo/` — absolute paths silently break `os.path.join` behavior.

### 8.2 Camera and GStreamer

- Always use `io-mode=4` in the `v4l2src` GStreamer element on this board.
- Always call `v4l2-ctl --set-parm=30` to set the framerate before opening the capture pipeline.
- The supported native resolution for this board's webcam pipeline is **640×480**.
- The board's own Camera app source (`launch_camera_control_mp1.sh`) is the most reliable reference for the correct pipeline.

### 8.3 SDL2 on Wayland

- Set `SDL_VIDEODRIVER=wayland` and `XDG_RUNTIME_DIR` before `SDL_Init()`.
- Use `SDL_WINDOW_FULLSCREEN_DESKTOP` for the window flags.
- Touch event coordinates (`SDL_FINGERDOWN`/`UP`) are normalized `[0.0, 1.0]`. Multiply by `SW`/`SH` to get pixel coordinates.
- On some boards, the touch surface is rotated relative to the display. Check **both** X and Y coordinates when detecting button regions.
- Always install `SIGTERM` and `SIGINT` handlers so the dashboard can be killed cleanly from scripts or SSH.

### 8.4 Debugging Tips

- Add `exec >> /tmp/face_launch.log 2>&1` as the first line of any shell script you want to diagnose — all output goes to the log file regardless of how the script was launched.
- Use `ps aux | grep weston` to check which user the demo launcher and Weston run as.
- Use `find /run -name 'wayland-*'` to locate the actual Wayland socket at runtime.
- Never `scp` from the board to the board (self-copy). Always `scp` from the Ubuntu machine to the board.

---

## Author
 
Emad Roshandel
