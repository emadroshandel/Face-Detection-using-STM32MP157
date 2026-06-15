#!/usr/bin/env python3
"""
face_detect_server.py — Web Dashboard for Face Detection on STM32MP157
=======================================================================
Uses ONLY Python standard library (http.server, socketserver, threading, json).
No pip / Flask / external packages required.

Run:
    python3 face_detect_server.py --cascade /home/root/haarcascade_frontalface_default.xml

Then open in any browser on your network:
    http://192.168.1.111:5000

Options:
    --device N         Camera device index     (default: 0)
    --width  W         Capture width           (default: 320)
    --height H         Capture height          (default: 240)
    --port   P         HTTP server port        (default: 5000)
    --cascade PATH     Path to Haar cascade XML (required on OpenSTLinux)
    --skip-frames N    Analyse every Nth frame (default: 2)
    --scale-factor F   Haar scaleFactor        (default: 1.1)
    --min-neighbors N  Haar minNeighbors       (default: 5)
    --min-face-size PX Minimum face size px    (default: 60)
    --cooldown SEC     Min seconds between log events (default: 2.0)
    --lcd              Also show on board LCD via Wayland/GStreamer waylandsink
    --lcd-width  W     LCD resolution width    (default: 1024)
    --lcd-height H     LCD resolution height   (default: 600)
    --wayland-display  Wayland socket name     (default: wayland-0)

LCD usage (run from board console or with WAYLAND_DISPLAY set):
    WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run python3 face_detect_server.py \\
        --cascade /home/root/haarcascade_frontalface_default.xml --lcd

-- Author: Emad Roshandel
"""

import argparse
import json
import logging
import os
import sys
import threading
import time
import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2

# ── Logging ───────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)


# ── Arguments ─────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Face detection web dashboard — stdlib only")
    p.add_argument("--device",        type=int,   default=0)
    p.add_argument("--width",         type=int,   default=640)   # Camera app uses 640x480
    p.add_argument("--height",        type=int,   default=480)
    p.add_argument("--port",          type=int,   default=5000)
    p.add_argument("--cascade",       type=str,   default=None)
    p.add_argument("--skip-frames",   type=int,   default=2)
    p.add_argument("--scale-factor",  type=float, default=1.1)
    p.add_argument("--min-neighbors", type=int,   default=5)
    p.add_argument("--min-face-size", type=int,   default=60)
    p.add_argument("--cooldown",         type=float, default=2.0)
    p.add_argument("--lcd",              action="store_true",
                   help="Show on board LCD via Wayland waylandsink")
    p.add_argument("--lcd-width",        type=int,   default=1024)
    p.add_argument("--lcd-height",       type=int,   default=600)
    p.add_argument("--wayland-display",  type=str,   default="wayland-0")
    return p.parse_args()


# ── Cascade path resolution (safe on Yocto — no cv2.data crash) ───────────────
HAAR_SEARCH_PATHS = [
    "/home/root/haarcascade_frontalface_default.xml",
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
    "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
]

def find_cascade(override=None):
    if override:
        if not os.path.isfile(override):
            log.error(f"Cascade not found: {override}")
            sys.exit(1)
        return override
    if hasattr(cv2, "data") and hasattr(cv2.data, "haarcascades"):
        HAAR_SEARCH_PATHS.append(
            os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")
        )
    for path in HAAR_SEARCH_PATHS:
        if os.path.isfile(path):
            log.info(f"Cascade: {path}")
            return path
    log.error("Cannot find cascade XML. Pass --cascade /path/to/haarcascade_frontalface_default.xml")
    sys.exit(1)


# ── Shared detection state ────────────────────────────────────────────────────
class DetectionState:
    def __init__(self):
        self._lock      = threading.Lock()
        self.face_count = 0
        self.total      = 0
        self.fps        = 0.0
        self.last_seen  = "Never"
        self.events     = []        # [{"time": str, "count": int}, ...]

    def update(self, face_count, fps):
        now_str = datetime.datetime.now().strftime("%H:%M:%S")
        with self._lock:
            self.face_count = face_count
            self.fps        = round(fps, 1)
            if face_count > 0:
                self.total     += face_count
                self.last_seen  = now_str
                self.events.insert(0, {"time": now_str, "count": face_count})
                self.events = self.events[:20]

    def snapshot(self):
        with self._lock:
            return {
                "face_count": self.face_count,
                "total":      self.total,
                "fps":        self.fps,
                "last_seen":  self.last_seen,
                "events":     list(self.events),
            }


state = DetectionState()

# ── Shared frame buffers ──────────────────────────────────────────────────────
_frame_lock  = threading.Lock()
_latest_jpeg = None          # raw JPEG bytes  (for /video MJPEG stream)
_latest_bmp  = None          # raw BMP  bytes  (for /rawframe — SDL2 dashboard)

def set_frame(jpeg_bytes, bmp_bytes):
    global _latest_jpeg, _latest_bmp
    with _frame_lock:
        _latest_jpeg = jpeg_bytes
        _latest_bmp  = bmp_bytes

def get_frame():
    with _frame_lock:
        return _latest_jpeg

def get_bmp_frame():
    with _frame_lock:
        return _latest_bmp


# ── LCD display (optional) ───────────────────────────────────────────────────
def create_lcd_writer(args):
    """Open a GStreamer VideoWriter that renders to the Wayland display."""
    import os
    os.environ.setdefault("WAYLAND_DISPLAY", args.wayland_display)
    os.environ.setdefault("XDG_RUNTIME_DIR", "/run")

    w, h = args.lcd_width, args.lcd_height
    gst = (
        f"appsrc ! "
        f"video/x-raw,format=BGR,width={w},height={h},framerate=15/1 ! "
        f"videoconvert ! "
        f"waylandsink sync=false fullscreen=true"
    )
    writer = cv2.VideoWriter(gst, cv2.CAP_GSTREAMER, 0, 15, (w, h), True)
    if not writer.isOpened():
        log.warning("LCD: could not open waylandsink — LCD display disabled.")
        log.warning("Tip: run with WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run")
        return None
    log.info(f"LCD display active: {w}x{h} via {args.wayland_display}")
    return writer


def render_lcd_frame(frame, face_count, fps, lcd_w, lcd_h):
    """
    Scale camera frame to LCD size and draw a status bar at the bottom.
    Uses only OpenCV drawing — no numpy, no external libs.
    """
    BAR_H = 80   # height of the status bar in pixels
    cam_h = lcd_h - BAR_H

    # Scale camera feed to fill upper portion of screen
    display = cv2.resize(frame, (lcd_w, cam_h))

    # ── Status bar background ─────────────────────────────────────────────────
    if face_count > 0:
        bar_color  = (0, 90, 0)      # dark green
        text_color = (60, 255, 120)  # bright green
        status_txt = f"FACE DETECTED   [{face_count}]"
    else:
        bar_color  = (40, 0, 60)     # dark purple
        text_color = (120, 80, 140)  # muted purple
        status_txt = "NO FACE"

    # Draw bar by drawing a filled rectangle at the bottom of the frame
    # We extend the display image by BAR_H rows by drawing on a taller canvas
    # (pure OpenCV — no numpy vstack needed)
    full = cv2.copyMakeBorder(display, 0, BAR_H, 0, 0,
                              cv2.BORDER_CONSTANT, value=bar_color)

    # Status text (large, left-aligned)
    cv2.putText(full, status_txt,
                (24, lcd_h - 22),
                cv2.FONT_HERSHEY_SIMPLEX, 1.6, text_color, 3, cv2.LINE_AA)

    # FPS counter (small, right-aligned)
    fps_txt = f"FPS {fps:.1f}"
    (tw, _), _ = cv2.getTextSize(fps_txt, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
    cv2.putText(full, fps_txt,
                (lcd_w - tw - 20, lcd_h - 24),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (160, 160, 160), 2, cv2.LINE_AA)

    # Detection indicator circle (left of status bar)
    cx, cy = 28, lcd_h - BAR_H // 2
    ind_color = (60, 255, 120) if face_count > 0 else (60, 40, 80)
    cv2.circle(full, (cx, cy), 18, ind_color, -1, cv2.LINE_AA)
    # Checkmark or cross drawn with lines
    if face_count > 0:
        cv2.line(full, (cx - 9, cy),     (cx - 2, cy + 8),  (0, 0, 0), 3, cv2.LINE_AA)
        cv2.line(full, (cx - 2, cy + 8), (cx + 10, cy - 8), (0, 0, 0), 3, cv2.LINE_AA)
    else:
        cv2.line(full, (cx - 8, cy - 8), (cx + 8, cy + 8), (0, 0, 0), 3, cv2.LINE_AA)
        cv2.line(full, (cx + 8, cy - 8), (cx - 8, cy + 8), (0, 0, 0), 3, cv2.LINE_AA)

    return full


# ── Detection thread ──────────────────────────────────────────────────────────
def detection_loop(args, cascade_path):
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        log.error("Failed to load Haar cascade.")
        sys.exit(1)

    device_path = f"/dev/video{args.device}"
    log.info(f"Opening {device_path} via GStreamer at {args.width}x{args.height}")

    # Set framerate to 30 fps — same as the board's Camera app (v4l2-ctl --set-parm=30)
    import subprocess
    try:
        subprocess.run(
            ["v4l2-ctl", f"--device={device_path}", "--set-parm=30"],
            check=False, timeout=3,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        log.info(f"v4l2-ctl: framerate set to 30 fps on {device_path}")
    except Exception as e:
        log.warning(f"v4l2-ctl not available or failed: {e}")

    # OpenSTLinux OpenCV uses GStreamer.
    # io-mode=4 (dmabuf) is required for reliable capture on STM32MP157 —
    # copied directly from the board's own Camera app pipeline.
    gst = (
        f"v4l2src device={device_path} io-mode=4 ! "
        f"video/x-raw,width={args.width},height={args.height} ! "
        f"videoconvert ! "
        f"video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=1 sync=false"
    )
    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        # Fallback: try without io-mode (plain v4l2)
        log.warning("io-mode=4 failed, retrying without io-mode...")
        gst = (
            f"v4l2src device={device_path} ! "
            f"video/x-raw,width={args.width},height={args.height} ! "
            f"videoconvert ! "
            f"video/x-raw,format=BGR ! "
            f"appsink drop=1 max-buffers=1 sync=false"
        )
        cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        log.error(f"Cannot open {device_path}. Is webcam connected?")
        sys.exit(1)

    # LCD display writer (None if --lcd not set or waylandsink unavailable)
    lcd_writer = create_lcd_writer(args) if args.lcd else None

    frame_count  = 0
    fps_timer    = time.time()
    fps_frames   = 0
    current_fps  = 0.0
    last_event_t = 0.0

    log.info("Detection thread running.")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        frame_count += 1
        fps_frames  += 1

        elapsed = time.time() - fps_timer
        if elapsed >= 2.0:
            current_fps = fps_frames / elapsed
            fps_frames  = 0
            fps_timer   = time.time()

        # Detect on every Nth frame
        face_count = 0
        if frame_count % args.skip_frames == 0:
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray  = cv2.equalizeHist(gray)
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor  = args.scale_factor,
                minNeighbors = args.min_neighbors,
                minSize      = (args.min_face_size, args.min_face_size),
                flags        = cv2.CASCADE_SCALE_IMAGE,
            )
            face_count = len(faces)

            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 220, 80), 2)
                cv2.putText(frame, "Face", (x, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 80), 2)

            now = time.time()
            if face_count > 0 and (now - last_event_t) >= args.cooldown:
                last_event_t = now
                state.update(face_count, current_fps)
            elif face_count == 0:
                state.update(0, current_fps)

        # Overlay FPS / face count
        cv2.putText(frame, f"FPS:{current_fps:.1f}  Faces:{face_count}",
                    (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

        # ── LCD display ───────────────────────────────────────────────────────
        if lcd_writer is not None:
            lcd_frame = render_lcd_frame(
                frame, face_count, current_fps,
                args.lcd_width, args.lcd_height
            )
            lcd_writer.write(lcd_frame)

        # ── Encode and publish ────────────────────────────────────────────────
        ok_j, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        ok_b, bmp  = cv2.imencode(".bmp", frame)   # BMP for SDL2 dashboard
        if ok_j and ok_b:
            set_frame(jpeg.tobytes(), bmp.tobytes())
        elif ok_j:
            set_frame(jpeg.tobytes(), None)

    cap.release()
    if lcd_writer:
        lcd_writer.release()


# ── HTML dashboard ────────────────────────────────────────────────────────────
DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Face Detection \xe2\x80\x94 STM32MP157</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0f1117; color: #e0e0e0;
    font-family: 'Segoe UI', system-ui, sans-serif;
    min-height: 100vh; display: flex; flex-direction: column;
  }
  header {
    background: #1a1d27; border-bottom: 2px solid #2a2d3a;
    padding: 14px 24px; display: flex; align-items: center; gap: 12px;
  }
  header h1 { font-size: 1.1rem; font-weight: 600; color: #fff; }
  .chip {
    background: #2a2d3a; color: #7c8aff;
    font-size: 0.72rem; padding: 3px 10px;
    border-radius: 20px; font-weight: 600; letter-spacing: 0.04em;
  }
  .main { display: flex; flex: 1; }

  .video-panel {
    flex: 1; display: flex; align-items: center; justify-content: center;
    background: #080a0f; padding: 20px; min-width: 0;
  }
  .video-panel img {
    max-width: 100%; max-height: calc(100vh - 120px);
    border-radius: 8px; border: 2px solid #2a2d3a; display: block;
  }

  .status-panel {
    width: 280px; min-width: 280px; background: #13151f;
    border-left: 2px solid #2a2d3a;
    display: flex; flex-direction: column; padding: 20px 16px; gap: 16px;
  }
  .indicator {
    background: #1a1d27; border-radius: 12px; padding: 22px 16px 18px;
    text-align: center; border: 2px solid #2a2d3a;
    transition: border-color 0.3s, box-shadow 0.3s;
  }
  .indicator.detected { border-color: #22c55e; box-shadow: 0 0 20px rgba(34,197,94,0.25); }
  .indicator.clear    { border-color: #ef4444; box-shadow: 0 0 12px rgba(239,68,68,0.1); }
  .tick { font-size: 3.2rem; line-height: 1; margin-bottom: 8px; transition: transform 0.2s; }
  .indicator.detected .tick { transform: scale(1.15); }
  .status-label { font-size: 0.85rem; font-weight: 700; letter-spacing: 0.08em; text-transform: uppercase; }
  .indicator.detected .status-label { color: #22c55e; }
  .indicator.clear    .status-label { color: #ef4444; }
  .face-count-big { font-size: 1.6rem; font-weight: 800; color: #fff; margin-top: 6px; }

  .stats {
    background: #1a1d27; border-radius: 10px; padding: 14px;
    border: 1px solid #2a2d3a; display: flex; flex-direction: column; gap: 10px;
  }
  .stat-row { display: flex; justify-content: space-between; align-items: center; }
  .stat-label { font-size: 0.75rem; color: #6b7280; text-transform: uppercase; letter-spacing: 0.06em; }
  .stat-value { font-size: 0.92rem; font-weight: 700; color: #e0e0e0; }
  .stat-divider { height: 1px; background: #2a2d3a; }

  .log-section { flex: 1; display: flex; flex-direction: column; min-height: 0; }
  .log-title { font-size: 0.72rem; text-transform: uppercase; letter-spacing: 0.08em; color: #6b7280; margin-bottom: 8px; font-weight: 600; }
  .log-list { flex: 1; overflow-y: auto; display: flex; flex-direction: column; gap: 5px; }
  .log-item {
    background: #1a1d27; border: 1px solid #2a2d3a; border-radius: 7px;
    padding: 7px 10px; display: flex; justify-content: space-between;
    align-items: center; font-size: 0.8rem; animation: fadeIn 0.3s ease;
  }
  @keyframes fadeIn { from { opacity:0; transform:translateY(-4px); } to { opacity:1; } }
  .log-time { color: #6b7280; }
  .log-faces { color: #22c55e; font-weight: 700; }
  .log-empty { color: #444; font-size: 0.8rem; text-align: center; padding-top: 20px; }

  .conn-badge { display: flex; align-items: center; gap: 6px; font-size: 0.72rem; color: #6b7280; margin-top: auto; padding-top: 12px; }
  .conn-dot { width: 8px; height: 8px; border-radius: 50%; background: #22c55e; animation: pulse 2s infinite; }
  .conn-dot.offline { background: #ef4444; animation: none; }
  @keyframes pulse { 0%,100% { opacity:1; } 50% { opacity:0.4; } }
</style>
</head>
<body>
<header>
  <h1>Face Detection Dashboard</h1>
  <span class="chip">STM32MP157</span>
  <span class="chip">OpenCV 4.9</span>
</header>
<div class="main">
  <div class="video-panel">
    <img src="/video" alt="Camera stream">
  </div>
  <div class="status-panel">
    <div class="indicator clear" id="indicator">
      <div class="tick" id="tick">\xe2\x9c\x97</div>
      <div class="status-label" id="statusLabel">No Face</div>
      <div class="face-count-big" id="faceCountBig">\xe2\x80\x94</div>
    </div>
    <div class="stats">
      <div class="stat-row">
        <span class="stat-label">Total Detected</span>
        <span class="stat-value" id="statTotal">0</span>
      </div>
      <div class="stat-divider"></div>
      <div class="stat-row">
        <span class="stat-label">FPS</span>
        <span class="stat-value" id="statFps">\xe2\x80\x94</span>
      </div>
      <div class="stat-divider"></div>
      <div class="stat-row">
        <span class="stat-label">Last Seen</span>
        <span class="stat-value" id="statLast">Never</span>
      </div>
    </div>
    <div class="log-section">
      <div class="log-title">Detection Events</div>
      <div class="log-list" id="logList">
        <div class="log-empty" id="logEmpty">No events yet\xe2\x80\xa6</div>
      </div>
    </div>
    <div class="conn-badge">
      <div class="conn-dot" id="connDot"></div>
      <span id="connLabel">Connected</span>
    </div>
  </div>
</div>
<script>
async function pollStatus() {
  try {
    const res = await fetch('/status');
    if (!res.ok) throw new Error();
    const d = await res.json();
    const detected = d.face_count > 0;
    document.getElementById('indicator').className = 'indicator ' + (detected ? 'detected' : 'clear');
    document.getElementById('tick').textContent = detected ? '✓' : '✗';
    document.getElementById('statusLabel').textContent = detected ? 'Face Detected' : 'No Face';
    const n = d.face_count;
    document.getElementById('faceCountBig').textContent = n === 0 ? '—' : n + (n === 1 ? ' face' : ' faces');
    document.getElementById('statTotal').textContent = d.total;
    document.getElementById('statFps').textContent = d.fps + ' fps';
    document.getElementById('statLast').textContent = d.last_seen;
    document.getElementById('connDot').classList.remove('offline');
    document.getElementById('connLabel').textContent = 'Connected';
    const logList = document.getElementById('logList');
    const logEmpty = document.getElementById('logEmpty');
    if (d.events && d.events.length > 0) {
      logEmpty.style.display = 'none';
      const firstItem = logList.querySelector('.log-item');
      const firstTime = firstItem ? firstItem.dataset.time : null;
      if (firstTime !== d.events[0].time) {
        logList.innerHTML = '';
        d.events.forEach(ev => {
          const item = document.createElement('div');
          item.className = 'log-item';
          item.dataset.time = ev.time;
          item.innerHTML = '<span class="log-time">' + ev.time + '</span><span class="log-faces">👤 ' + ev.count + '</span>';
          logList.appendChild(item);
        });
      }
    } else {
      logEmpty.style.display = 'block';
    }
  } catch(e) {
    document.getElementById('connDot').classList.add('offline');
    document.getElementById('connLabel').textContent = 'Disconnected';
  }
}
pollStatus();
setInterval(pollStatus, 1000);
</script>
</body>
</html>
"""


# ── HTTP request handler ──────────────────────────────────────────────────────
class Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self._serve_dashboard()
        elif self.path == "/status":
            self._serve_status()
        elif self.path == "/video":
            self._serve_mjpeg()
        elif self.path == "/snapshot":
            self._serve_snapshot()
        elif self.path == "/rawframe":
            self._serve_rawframe()
        else:
            self.send_error(404)

    def _serve_dashboard(self):
        data = DASHBOARD_HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type",   "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_status(self):
        data = json.dumps(state.snapshot()).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type",   "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control",  "no-cache")
        self.end_headers()
        self.wfile.write(data)

    def _serve_snapshot(self):
        """Return the latest annotated JPEG frame as a single image (for browser)."""
        jpeg = get_frame()
        if jpeg is None:
            self.send_error(503, "No frame yet")
            return
        self.send_response(200)
        self.send_header("Content-Type",   "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.send_header("Cache-Control",  "no-cache")
        self.end_headers()
        self.wfile.write(jpeg)

    def _serve_rawframe(self):
        """Return the latest annotated frame as BMP (for SDL2 dashboard — no extra library)."""
        bmp = get_bmp_frame()
        if bmp is None:
            self.send_error(503, "No frame yet")
            return
        self.send_response(200)
        self.send_header("Content-Type",   "image/bmp")
        self.send_header("Content-Length", str(len(bmp)))
        self.send_header("Cache-Control",  "no-cache")
        self.end_headers()
        self.wfile.write(bmp)

    def _serve_mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type",  "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        try:
            while True:
                jpeg = get_frame()
                if jpeg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + jpeg +
                    b"\r\n"
                )
                self.wfile.flush()
                time.sleep(0.04)   # ~25 fps max sent to browser
        except (BrokenPipeError, ConnectionResetError):
            pass   # browser tab closed — normal

    # Silence the default per-request log lines
    def log_message(self, fmt, *args):
        pass


# ── Threaded HTTP server ──────────────────────────────────────────────────────
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads      = True   # threads die with the main process
    allow_reuse_address = True   # avoid "Address already in use" after restart


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    args         = parse_args()
    cascade_path = find_cascade(args.cascade)

    # Start detection in a background daemon thread
    t = threading.Thread(
        target=detection_loop,
        args=(args, cascade_path),
        daemon=True,
        name="DetectionThread",
    )
    t.start()

    # Give the camera a moment to produce the first frame
    log.info("Waiting for first camera frame…")
    for _ in range(30):
        if get_frame() is not None:
            break
        time.sleep(0.1)

    # Start HTTP server
    server = ThreadedHTTPServer(("0.0.0.0", args.port), Handler)
    log.info(f"Dashboard → http://192.168.1.232:{args.port}")
    log.info("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Stopped.")
        server.shutdown()

