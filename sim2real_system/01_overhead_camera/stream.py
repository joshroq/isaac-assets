"""Shared access to the overhead-camera UDP stream.

One process at a time can bind UDP 5000, so only run ONE consumer of the stream
(receive.py, or a calibration/tracking tool) at once. Stop the plain receiver
before running a calibration step.
"""
import threading
import time

import cv2

# UDP tuning: tolerate reordering/overrun and give ffmpeg a big fifo so bursts
# over Tailscale don't drop whole frames.
STREAM_URL = "udp://@:5000?overrun_nonfatal=1&fifo_size=50000000"


class LatestFrame:
    """Read a capture in a background thread, keep only the newest frame.

    Reading UDP/H.264 through OpenCV otherwise queues frames internally and you
    drift seconds behind real time. Draining in a thread keeps us live.
    """

    def __init__(self, url=STREAM_URL):
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

    def wait_first(self, timeout=20.0):
        """Block until the first frame arrives; return it or None on timeout."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            f, _, _ = self.read()
            if f is not None:
                return f
            time.sleep(0.05)
        return None

    def release(self):
        """Stop the reader thread BEFORE freeing the capture: releasing while the thread is
        inside cap.read() made it touch freed FFmpeg state (segfault on exit, which also
        cost a session's unfinalised dashboard video)."""
        self._running = False
        t = getattr(self, "_t", None)
        if t is not None:
            t.join(timeout=3.0)
            if t.is_alive():            # stuck in a blocking read: leave it to process exit
                return
        self.cap.release()
