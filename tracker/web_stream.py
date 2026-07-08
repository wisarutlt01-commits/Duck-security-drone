"""
tracker/web_stream.py
======================
Headless MJPEG viewer — serves the latest annotated frame over HTTP so it
can be watched from a browser (no X11/Qt needed on the Pi).

Usage: browse to http://<pi-ip>:<port>/ while the tracker is running.
"""

import logging
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2
import numpy as np

_log = logging.getLogger("web_stream")

_BOUNDARY = "frame"

_INDEX_HTML = b"""<!doctype html>
<html><head><title>Drone Tracker</title></head>
<body style="margin:0;background:#111">
<img src="/stream" style="width:100%;height:100vh;object-fit:contain" />
</body></html>"""


class FrameBroadcaster:
    """Thread-safe holder for the latest annotated frame, shared between
    the vision loop (writer) and any number of HTTP client threads (readers)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None

    def update(self, frame: np.ndarray, quality: int = 80) -> None:
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return
        with self._lock:
            self._jpeg = buf.tobytes()

    def get(self) -> Optional[bytes]:
        with self._lock:
            return self._jpeg


def _make_handler(broadcaster: FrameBroadcaster):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # silence default per-request stderr logging

        def do_GET(self):
            if self.path == "/stream":
                self._serve_stream()
            elif self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(_INDEX_HTML)))
                self.end_headers()
                self.wfile.write(_INDEX_HTML)
            else:
                self.send_response(404)
                self.end_headers()

        def _serve_stream(self):
            self.send_response(200)
            self.send_header(
                "Content-Type", f"multipart/x-mixed-replace; boundary={_BOUNDARY}"
            )
            self.end_headers()
            try:
                while True:
                    jpeg = broadcaster.get()
                    if jpeg is None:
                        continue
                    self.wfile.write(f"--{_BOUNDARY}\r\n".encode())
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def start_server(broadcaster: FrameBroadcaster, host: str, port: int) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), _make_handler(broadcaster))
    thread = threading.Thread(target=server.serve_forever, name="MJPEGServer", daemon=True)
    thread.start()
    _log.info(f"MJPEG viewer: http://{host}:{port}/")
    return server
