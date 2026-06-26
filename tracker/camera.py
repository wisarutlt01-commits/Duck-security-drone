"""
tracker/camera.py
=================
FrameSource abstraction + two concrete implementations:

  V4L2Camera       — Pi Camera / USB webcam via V4L2/OpenCV (hardware mode)
  GazeboUDPStream  — H.264/RTP UDP stream from ardupilot_gazebo's
                     GstCameraPlugin (gazebo mode, networked Pi↔desktop)

Backend selection for GazeboUDPStream:
  1. _GstUDPSubscriber  — OpenCV + GStreamer pipeline (preferred)
  2. _FFmpegUDPSubscriber — system ffmpeg subprocess (fallback)

Both backends expose the same interface: read() → Optional[np.ndarray].
If neither GStreamer nor ffmpeg is available, GazeboUDPStream raises a
clear RuntimeError with install instructions.

The Gazebo-native transport backends (same-host only) have been
intentionally removed: they rely on local discovery and cannot bridge
two machines (Gazebo on a desktop PC, tracker on a Pi 5).
UDP-only is the correct topology for the networked setup.
"""

import os
import time
import shutil
import logging
import tempfile
import subprocess
import threading
from typing import Optional, Protocol, runtime_checkable

import cv2
import numpy as np

_log = logging.getLogger("Camera")

# ── Backend capability flags ──────────────────────────────────────────────────

_CV2_GSTREAMER: bool = "GStreamer:                   YES" in cv2.getBuildInformation()
_FFMPEG_AVAILABLE: bool = shutil.which("ffmpeg") is not None


# ── FrameSource protocol ──────────────────────────────────────────────────────

@runtime_checkable
class FrameSource(Protocol):
    """
    Minimal interface for a camera frame source.
    Implementations must be thread-safe: read() may be called from any thread.
    """

    def read(self) -> Optional[np.ndarray]:
        """Return the latest BGR frame, or None if not yet available."""
        ...

    def release(self) -> None:
        """Release hardware resources. Called once on shutdown."""
        ...


# ── Hardware camera (V4L2) ────────────────────────────────────────────────────

class V4L2Camera:
    """
    Pi Camera AI / USB webcam via V4L2 backend.

    Opens /dev/video<camera_index> with V4L2 (falls back to CAP_ANY),
    sets MJPEG, minimal buffer size, and requested resolution/fps.
    """

    def __init__(
        self,
        camera_index: int = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 60,
    ):
        self._cap = self._open(camera_index, width, height, fps)

    @staticmethod
    def _open(
        camera_index: int, width: int, height: int, fps: int
    ) -> cv2.VideoCapture:
        for backend in (cv2.CAP_V4L2, cv2.CAP_ANY):
            cap = cv2.VideoCapture(camera_index, backend)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                cap.set(cv2.CAP_PROP_FPS,          fps)
                # Reduce internal buffer to 1 frame → always get the latest frame
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                # Use MJPEG for faster USB/CSI throughput
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                _log.info(
                    f"Camera opened: {actual_w:.0f}x{actual_h:.0f}  "
                    f"backend={backend}"
                )
                return cap
        raise RuntimeError(
            "Cannot open camera — check /dev/video0 or libcamera config"
        )

    def read(self) -> Optional[np.ndarray]:
        ret, frame = self._cap.read()
        return frame if ret else None

    def release(self) -> None:
        self._cap.release()


# ── GStreamer UDP backend ─────────────────────────────────────────────────────

class _GstUDPSubscriber:
    """
    Receives H.264 RTP stream from ardupilot_gazebo's GstCameraPlugin.

    The plugin streams to udp://<udpHost>:<udpPort> (default port 5600).
    Requires OpenCV built with GStreamer support:
        python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i gst
    """

    _H264_PIPELINE = (
        "udpsrc address={host} port={port} "
        "caps=\"application/x-rtp,media=video,encoding-name=H264\" "
        "! rtph264depay ! h264parse ! avdec_h264 "
        "! videoconvert ! appsink drop=1 sync=false"
    )

    def __init__(self, host: str, port: int):
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._running = True

        pipeline = self._H264_PIPELINE.format(host=host, port=port)
        self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise RuntimeError(
                f"GStreamer UDP capture failed (udp://{host}:{port}).\n"
                "  Check: OpenCV built with GStreamer? GstCameraPlugin running?\n"
                "  Verify: python3 -c \"import cv2; "
                "print(cv2.getBuildInformation())\" | grep -i gst"
            )

        self._thread = threading.Thread(
            target=self._read_loop, name="GstUDP", daemon=True
        )
        self._thread.start()
        _log.info(f"GStreamer UDP subscriber ready: udp://{host}:{port}")

    def _read_loop(self) -> None:
        while self._running:
            ret, frame = self._cap.read()
            if ret and frame is not None:
                with self._lock:
                    self._frame = frame
            else:
                time.sleep(0.005)

    def get_latest_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def destroy(self) -> None:
        self._running = False
        self._cap.release()


# ── ffmpeg subprocess backend ─────────────────────────────────────────────────

class _FFmpegUDPSubscriber:
    """
    Receives H.264 RTP from GstCameraPlugin using a system ffmpeg subprocess.
    Used automatically when OpenCV was built without GStreamer support.

    Writes a minimal SDP to a temp file so ffmpeg can decode the RTP stream,
    then pipes raw bgr24 frames into this process via stdout.
    """

    _SDP_TEMPLATE = (
        "v=0\n"
        "o=- 0 0 IN IP4 {host}\n"
        "s=drone_cam\n"
        "c=IN IP4 {host}\n"
        "t=0 0\n"
        "m=video {port} RTP/AVP 96\n"
        "a=rtpmap:96 H264/90000\n"
    )

    def __init__(self, host: str, port: int, width: int, height: int):
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._running = True
        self._width = width
        self._height = height

        sdp_content = self._SDP_TEMPLATE.format(host=host, port=port)
        self._sdp_fd, self._sdp_path = tempfile.mkstemp(suffix=".sdp")
        with os.fdopen(self._sdp_fd, "w") as f:
            f.write(sdp_content)

        self._proc = self._start_proc()
        self._thread = threading.Thread(
            target=self._read_loop, name="FfmpegUDP", daemon=True
        )
        self._thread.start()
        _log.info(
            f"ffmpeg UDP subscriber ready: rtp://{host}:{port}  "
            f"({width}x{height})"
        )

    def _start_proc(self) -> subprocess.Popen:
        cmd = [
            "ffmpeg", "-hide_banner", "-loglevel", "error",
            "-protocol_whitelist", "file,udp,rtp",
            "-i", self._sdp_path,
            "-vf", f"scale={self._width}:{self._height}",
            "-f", "rawvideo", "-pix_fmt", "bgr24", "-",
        ]
        return subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=self._width * self._height * 3 * 4,
        )

    def _read_loop(self) -> None:
        frame_bytes = self._width * self._height * 3
        while self._running:
            if self._proc.poll() is not None:
                if not self._running:
                    break
                _log.warning("ffmpeg process exited — restarting in 2 s")
                time.sleep(2.0)
                self._proc = self._start_proc()
                continue
            raw = self._proc.stdout.read(frame_bytes)
            if len(raw) == frame_bytes:
                frame = np.frombuffer(raw, dtype=np.uint8).reshape(
                    (self._height, self._width, 3)
                )
                with self._lock:
                    self._frame = frame.copy()
            elif len(raw) == 0:
                time.sleep(0.005)

    def get_latest_frame(self) -> Optional[np.ndarray]:
        with self._lock:
            return self._frame.copy() if self._frame is not None else None

    def destroy(self) -> None:
        self._running = False
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        try:
            os.unlink(self._sdp_path)
        except OSError:
            pass


# ── Gazebo UDP stream (wraps one of the two backends above) ───────────────────

class GazeboUDPStream:
    """
    Frame source for Gazebo mode: H.264/RTP UDP stream from the desktop PC's
    ardupilot_gazebo GstCameraPlugin.

    Tries GStreamer UDP first (lower latency, zero subprocess overhead), then
    falls back to ffmpeg UDP.  Raises RuntimeError if neither is available.
    """

    def __init__(self, host: str, port: int, width: int, height: int):
        self._sub: "_GstUDPSubscriber | _FFmpegUDPSubscriber"
        if _CV2_GSTREAMER:
            _log.info(f"GazeboUDPStream: backend=GStreamer  udp://{host}:{port}")
            self._sub = _GstUDPSubscriber(host, port)
        elif _FFMPEG_AVAILABLE:
            _log.info(f"GazeboUDPStream: backend=ffmpeg  rtp://{host}:{port}")
            self._sub = _FFmpegUDPSubscriber(host, port, width, height)
        else:
            raise RuntimeError(
                "No UDP camera backend available for Gazebo mode.\n"
                "  Option 1 (preferred): install OpenCV with GStreamer support\n"
                "    apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev\n"
                "    pip install opencv-python  # or build from source with GStreamer\n"
                "  Option 2 (fallback):  apt install ffmpeg"
            )

    def read(self) -> Optional[np.ndarray]:
        return self._sub.get_latest_frame()

    def release(self) -> None:
        self._sub.destroy()
