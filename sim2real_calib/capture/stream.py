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


# Background reader for the Pi camera's UDP/H.264 stream.
# Attributes: cap = OpenCV capture, _frame/_stamp/_n = newest frame, its arrival
# time (time.time()) and a running frame counter (lets callers wait for NEW frames).
class LatestFrame:
    """Read a capture in a background thread, keep only the newest frame.

    Reading UDP/H.264 through OpenCV otherwise queues frames internally and you
    drift seconds behind real time. Draining in a thread keeps us live.
    """

    # Open the stream and start the reader thread (only if the capture opened).
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

    # True if OpenCV managed to open the UDP stream.
    def opened(self):
        return self.cap.isOpened()

    # Reader thread: decode frames as fast as they arrive and keep only the newest,
    # so consumers never fall behind real time.
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

    # Newest frame as (frame, arrival_time, frame_counter); (None, 0.0, 0) before the first.
    def read(self):
        with self._lock:
            if self._frame is None:
                return None, 0.0, 0
            return self._frame, self._stamp, self._n

    # Poll every 50 ms until a frame exists or `timeout` seconds pass.
    def wait_first(self, timeout=20.0):
        """Block until the first frame arrives; return it or None on timeout."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            frame, _, _ = self.read()
            if frame is not None:
                return frame
            time.sleep(0.05)
        return None

    # Stop the reader thread, then free the capture (order matters, see docstring).
    def release(self):
        """Stop the reader thread BEFORE freeing the capture: releasing while the thread is
        inside cap.read() made it touch freed FFmpeg state (segfault on exit, which also
        cost a session's unfinalised dashboard video)."""
        self._running = False
        reader_thread = getattr(self, "_t", None)
        if reader_thread is not None:
            reader_thread.join(timeout=3.0)
            if reader_thread.is_alive():            # stuck in a blocking read: leave it to process exit
                return
        self.cap.release()
