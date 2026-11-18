#!/usr/bin/env python3
"""Desktop-side receiver for the overhead C920e feed streamed from the laptop.

The laptop runs ffmpeg (see ../laptop_sender/send_windows.bat) which H.264-encodes
the webcam and pushes MPEG-TS over UDP to this desktop's Tailscale IP. Here we read
that stream with OpenCV, always displaying the *newest* frame (a background thread
drains the socket so we never build up latency), with a live HUD plus snapshot /
record controls.

Keys (in the video window):
    q / Esc  quit
    s        save a snapshot PNG
    r        toggle recording to an .mp4

Examples:
    # default: listen on UDP 5000 (matches the sender's target port)
    python3 receive.py

    # show the colored sticker triangle, center and heading
    python3 receive.py --track

    # explicit URL / different port
    python3 receive.py --url 'udp://@:5000'

    # headless (no window) - just report stats and allow --record
    python3 receive.py --no-show --record out.mp4
"""
import argparse
import os
import sys
import threading
import time
from datetime import datetime

import cv2

# UDP tuning passed to ffmpeg: tolerate reordering/overrun and give it a big
# kernel-side fifo so bursts over Tailscale don't drop whole frames.
DEFAULT_URL = "udp://@:5000?overrun_nonfatal=1&fifo_size=50000000"


class LatestFrame:
    """Continuously read a capture in a thread, keep only the most recent frame.

    Reading UDP/H.264 through OpenCV otherwise queues frames internally and you
    drift seconds behind real time. Draining in a thread keeps us live.
    """

    def __init__(self, url):
        self.url = url
        self.cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        self._lock = threading.Lock()
        self._frame = None
        self._stamp = 0.0
        self._n = 0
        self._running = self.cap.isOpened()
        if self._running:
            self._t = threading.Thread(target=self._loop, daemon=True)
            self._t.start()

    def opened(self):
        return self.cap.isOpened()

    def _loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                # transient loss on UDP; brief pause and retry rather than die
                time.sleep(0.005)
                continue
            with self._lock:
                self._frame = frame
                self._stamp = time.time()
                self._n += 1

    def read(self):
        with self._lock:
            if self._frame is None:
                return None, 0.0, 0
            return self._frame, self._stamp, self._n

    def release(self):
        self._running = False                     # join before freeing (see stream.py)
        t = getattr(self, "_t", None)
        if t is not None:
            t.join(timeout=3.0)
            if t.is_alive():
                return
        self.cap.release()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default=DEFAULT_URL,
                    help="OpenCV/ffmpeg capture URL (default: %(default)s)")
    ap.add_argument("--no-show", action="store_true",
                    help="don't open a window (headless stats only)")
    ap.add_argument("--record", metavar="OUT.mp4", default=None,
                    help="start recording to this file immediately")
    ap.add_argument("--snapshot-dir", default=None,
                    help="where 's' saves PNGs (default: ./snapshots)")
    ap.add_argument("--track", action="store_true", help="overlay colored sticker tracking")
    ap.add_argument("--stickers", default=None, help="sticker tracker configuration JSON")
    ap.add_argument("--calib", default=None, help="grid calibration JSON")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 until quit)")
    args = ap.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    tracker = None
    if args.track:
        sys.path.insert(0, os.path.join(here, "..", "viz"))
        from metric_overlay import load_calib, CALIB_DEFAULT
        from sticker_tracker import StickerTracker, DEFAULT_CONFIG, draw_tracking
        try:
            tracker = StickerTracker(args.stickers or DEFAULT_CONFIG)
            H_img2m, _, _ = load_calib(args.calib or CALIB_DEFAULT)
        except (ValueError, OSError) as exc:
            print(f"[receive] ERROR: {exc}", file=sys.stderr)
            return 2
    snap_dir = args.snapshot_dir or os.path.join(here, "snapshots")
    os.makedirs(snap_dir, exist_ok=True)

    show = not args.no_show and bool(os.environ.get("DISPLAY"))
    if not args.no_show and not show:
        print("[receive] no DISPLAY found -> running headless", file=sys.stderr)

    print(f"[receive] opening {args.url}")
    print("[receive] waiting for first frame (start the laptop sender if idle)...")
    src = LatestFrame(args.url)
    if not src.opened():
        print("[receive] ERROR: could not open stream. Is the sender running "
              "and is UDP 5000 reachable over Tailscale?", file=sys.stderr)
        return 2

    writer = None
    rec_path = None

    def open_writer(frame, path):
        h, w = frame.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        return cv2.VideoWriter(path, fourcc, 30.0, (w, h))

    # FPS measurement over a sliding count of processed frames
    last_n = 0
    last_t = time.time()
    fps = 0.0
    processed_n = -1
    tracked_n = lost_n = 0
    pose = None
    win = "C920e feed (q quit, s snap, r rec)"

    try:
        # block until first frame or timeout
        t0 = time.time()
        while src.read()[0] is None:
            if time.time() - t0 > 20:
                print("[receive] ERROR: no frames after 20s. Check sender + "
                      "firewall/Tailscale.", file=sys.stderr)
                return 3
            time.sleep(0.05)

        if args.record:
            f0 = src.read()[0]
            rec_path = args.record
            writer = open_writer(f0, rec_path)
            print(f"[receive] recording -> {rec_path}")

        started = time.monotonic()
        while not args.duration or time.monotonic() - started < args.duration:
            frame, stamp, n = src.read()
            if frame is None:
                time.sleep(0.01)
                continue

            now = time.time()
            if now - last_t >= 0.5:
                fps = (n - last_n) / (now - last_t)
                last_n, last_t = n, now

            age_ms = (now - stamp) * 1000.0

            if n != processed_n:
                processed_n = n
                if tracker is not None:
                    pose = tracker.detect(frame, H_img2m)
                    if pose is None:
                        lost_n += 1
                    else:
                        tracked_n += 1
                if writer is not None:
                    writer.write(frame)

            if show:
                h, w = frame.shape[:2]
                hud = f"{w}x{h}  {fps:4.1f} fps  age {age_ms:4.0f}ms  frames {n}"
                if writer is not None:
                    hud += "  [REC]"
                disp = frame.copy()
                if tracker is not None:
                    if age_ms > 500:
                        tracker.status = "camera feed stale"
                    draw_tracking(disp, pose if age_ms <= 500 else None, tracker)
                cv2.putText(disp, hud, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 0, 0), 3, cv2.LINE_AA)
                cv2.putText(disp, hud, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0), 1, cv2.LINE_AA)
                cv2.imshow(win, disp)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("s"):
                    fn = os.path.join(
                        snap_dir,
                        datetime.now().strftime("snap_%Y%m%d_%H%M%S_%f.png"))
                    cv2.imwrite(fn, frame)
                    print(f"[receive] snapshot -> {fn}")
                if key == ord("r"):
                    if writer is None:
                        rec_path = os.path.join(
                            here,
                            datetime.now().strftime("rec_%Y%m%d_%H%M%S.mp4"))
                        writer = open_writer(frame, rec_path)
                        print(f"[receive] recording -> {rec_path}")
                    else:
                        writer.release()
                        writer = None
                        print(f"[receive] stopped recording -> {rec_path}")
            else:
                # headless: periodic status line
                if now - last_t < 0.02:  # just refreshed fps this tick
                    print(f"[receive] {fps:4.1f} fps  age {age_ms:4.0f}ms  "
                          f"frames {n}", end="\r", flush=True)
                time.sleep(.005)
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
            print(f"\n[receive] saved {rec_path}")
        src.release()
        if show:
            cv2.destroyAllWindows()
        if tracker is not None:
            print(f"\n[receive] sticker poses: {tracked_n}; frames without triangle: {lost_n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
