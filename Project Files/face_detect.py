#!/usr/bin/env python3
"""
face_detect.py — Webcam Face Detection for STM32MP157
======================================================
Uses OpenCV (Haar Cascade) to detect faces from a V4L2 USB webcam.

Features:
  - Headless mode (no display required — works over SSH)
  - Saves a snapshot JPEG every time a new face is detected
  - Optional GPIO output pulse when a face appears (requires libgpiod)
  - Configurable via command-line arguments

Hardware:
  - STM32MP157 running OpenSTLinux or Buildroot Linux
  - USB webcam (V4L2 compatible, e.g. /dev/video0)

Dependencies:
  - python3-opencv  (apt install python3-opencv  OR  pip3 install opencv-python-headless)
  - python3-gpiod   (optional, for GPIO trigger: apt install python3-gpiod)

Usage examples:
  # Headless face detection, save snapshots to ./captures/
  python3 face_detect.py

  # Show live preview on HDMI display
  python3 face_detect.py --display

  # Use a different camera device
  python3 face_detect.py --device 1

  # Pulse GPIO chip0 line 5 on face detection
  python3 face_detect.py --gpio-chip 0 --gpio-line 5

  # Process only every 3rd frame (reduce CPU load)
  python3 face_detect.py --skip-frames 3
"""

import argparse
import os
import sys
import time
import datetime
import logging

import cv2

# ── Logging setup ────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)


# ── Argument parsing ──────────────────────────────────────────────────────────
def parse_args():
    parser = argparse.ArgumentParser(
        description="Face detection via webcam on STM32MP157"
    )
    parser.add_argument(
        "--device", type=int, default=0,
        help="V4L2 camera device index (default: 0 → /dev/video0)"
    )
    parser.add_argument(
        "--width", type=int, default=320,
        help="Capture width in pixels (default: 320). Lower = faster."
    )
    parser.add_argument(
        "--height", type=int, default=240,
        help="Capture height in pixels (default: 240)."
    )
    parser.add_argument(
        "--display", action="store_true",
        help="Show live preview window (requires HDMI/display connected)"
    )
    parser.add_argument(
        "--no-save", action="store_true",
        help="Do not save snapshot images on detection"
    )
    parser.add_argument(
        "--save-dir", type=str, default="captures",
        help="Directory to save snapshot images (default: ./captures)"
    )
    parser.add_argument(
        "--scale-factor", type=float, default=1.1,
        help="Haar cascade scaleFactor (default: 1.1). Lower = more sensitive but slower."
    )
    parser.add_argument(
        "--min-neighbors", type=int, default=5,
        help="Haar cascade minNeighbors (default: 5). Higher = fewer false positives."
    )
    parser.add_argument(
        "--min-face-size", type=int, default=60,
        help="Minimum face width/height in pixels to detect (default: 60)."
    )
    parser.add_argument(
        "--skip-frames", type=int, default=2,
        help="Process every Nth frame to reduce CPU load (default: 2)."
    )
    parser.add_argument(
        "--cooldown", type=float, default=3.0,
        help="Seconds to wait before saving another snapshot (default: 3.0)."
    )
    parser.add_argument(
        "--gpio-chip", type=int, default=None,
        help="GPIO chip number for face-detected output pulse (e.g. 0 for gpiochip0)"
    )
    parser.add_argument(
        "--gpio-line", type=int, default=None,
        help="GPIO line number within the chip (e.g. 5)"
    )
    parser.add_argument(
        "--gpio-pulse-ms", type=int, default=200,
        help="GPIO pulse duration in milliseconds (default: 200)"
    )
    parser.add_argument(
        "--cascade", type=str, default=None,
        help="Path to Haar cascade XML. Auto-detected if not specified."
    )
    return parser.parse_args()


# ── Haar cascade path auto-detection ─────────────────────────────────────────
HAAR_SEARCH_PATHS = [
    # OpenSTLinux / Yocto
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
    # Debian / Ubuntu
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
    "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
    # Python package data (not available in Yocto builds — wrapped safely)
    *(
        [os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")]
        if hasattr(cv2, "data") and hasattr(cv2.data, "haarcascades")
        else []
    ),
]


def find_cascade(override=None):
    if override:
        if not os.path.isfile(override):
            log.error(f"Cascade file not found: {override}")
            sys.exit(1)
        return override
    for path in HAAR_SEARCH_PATHS:
        if os.path.isfile(path):
            log.info(f"Using Haar cascade: {path}")
            return path
    log.error(
        "Could not find haarcascade_frontalface_default.xml. "
        "Install opencv data or pass --cascade <path>."
    )
    sys.exit(1)


# ── Optional GPIO support ─────────────────────────────────────────────────────
def setup_gpio(chip_num, line_num):
    """Return a gpiod Line object or None if gpiod is unavailable."""
    if chip_num is None or line_num is None:
        return None
    try:
        import gpiod  # type: ignore
        chip = gpiod.Chip(f"gpiochip{chip_num}")
        line = chip.get_line(line_num)
        line.request(consumer="face_detect", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        log.info(f"GPIO ready: gpiochip{chip_num} line {line_num}")
        return line
    except ImportError:
        log.warning("python3-gpiod not installed — GPIO trigger disabled.")
        return None
    except Exception as e:
        log.warning(f"GPIO setup failed: {e} — GPIO trigger disabled.")
        return None


def gpio_pulse(line, duration_ms):
    """Briefly pulse the GPIO line HIGH then LOW."""
    if line is None:
        return
    try:
        line.set_value(1)
        time.sleep(duration_ms / 1000.0)
        line.set_value(0)
    except Exception as e:
        log.warning(f"GPIO pulse error: {e}")


# ── Snapshot saving ───────────────────────────────────────────────────────────
def save_snapshot(frame, save_dir, face_count):
    """Save a JPEG snapshot with timestamp filename."""
    os.makedirs(save_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(save_dir, f"face_{ts}_{face_count}faces.jpg")
    cv2.imwrite(filename, frame)
    log.info(f"Snapshot saved → {filename}")
    return filename


# ── Main detection loop ───────────────────────────────────────────────────────
def run(args):
    # Load Haar cascade
    cascade_path = find_cascade(args.cascade)
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        log.error("Failed to load Haar cascade classifier.")
        sys.exit(1)

    # Open webcam
    device_path = f"/dev/video{args.device}"
    log.info(f"Opening {device_path} at {args.width}x{args.height}")
    cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # reduce latency on embedded hardware

    if not cap.isOpened():
        log.error(f"Cannot open /dev/video{args.device}. Is webcam connected?")
        sys.exit(1)

    # Setup GPIO (optional)
    gpio_line = setup_gpio(args.gpio_chip, args.gpio_line)

    log.info("Face detection started. Press Ctrl+C to stop.")
    if args.display:
        log.info("Display mode ON — live preview window active.")

    frame_count = 0
    last_snapshot_time = 0.0
    total_detections = 0
    fps_timer = time.time()
    fps_frames = 0
    current_fps = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                log.warning("Failed to grab frame — retrying...")
                time.sleep(0.1)
                continue

            frame_count += 1
            fps_frames += 1

            # FPS calculation (every 2 seconds)
            elapsed = time.time() - fps_timer
            if elapsed >= 2.0:
                current_fps = fps_frames / elapsed
                fps_frames = 0
                fps_timer = time.time()

            # ── Skip frames to reduce CPU load ───────────────────────────────
            if frame_count % args.skip_frames != 0:
                # Still show the frame in display mode (with previous results)
                if args.display:
                    cv2.imshow("Face Detection - STM32MP157", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                continue

            # ── Convert to grayscale for detection ───────────────────────────
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)   # improve contrast in varied lighting

            # ── Run face detection ────────────────────────────────────────────
            faces = face_cascade.detectMultiScale(
                gray,
                scaleFactor=args.scale_factor,
                minNeighbors=args.min_neighbors,
                minSize=(args.min_face_size, args.min_face_size),
                flags=cv2.CASCADE_SCALE_IMAGE,
            )

            face_count = len(faces)
            now = time.time()

            if face_count > 0:
                total_detections += face_count
                log.info(f"Face(s) detected: {face_count}  |  FPS: {current_fps:.1f}")

                # Draw rectangles in display mode
                if args.display:
                    for (x, y, w, h) in faces:
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(
                            frame, "Face", (x, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                        )

                # Cooldown: don't spam saves/GPIO
                if now - last_snapshot_time >= args.cooldown:
                    last_snapshot_time = now

                    # Save snapshot
                    if not args.no_save:
                        save_snapshot(frame, args.save_dir, face_count)

                    # Pulse GPIO
                    gpio_pulse(gpio_line, args.gpio_pulse_ms)

            # ── Display overlay ───────────────────────────────────────────────
            if args.display:
                overlay = f"FPS: {current_fps:.1f}  Faces: {face_count}  Total: {total_detections}"
                cv2.putText(
                    frame, overlay, (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1
                )
                cv2.imshow("Face Detection - STM32MP157", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    log.info("Quit key pressed.")
                    break

    except KeyboardInterrupt:
        log.info("Interrupted by user.")
    finally:
        cap.release()
        if args.display:
            cv2.destroyAllWindows()
        if gpio_line:
            gpio_line.release()
        log.info(
            f"Stopped. Frames processed: {frame_count}  |  Total face detections: {total_detections}"
        )


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    args = parse_args()
    run(args)
