"""
tracker/vision.py
=================
VisionSystem — Thread 1
-----------------------
Responsibilities:
  • Read frames from a FrameSource (V4L2Camera or GazeboUDPStream)
  • Run YOLO inference on every frame
  • Filter results to configured target classes only
  • Publish latest detection into a thread-safe shared state
  • Maintain a Kalman filter for forward-prediction and jitter smoothing

The FrameSource is injected by main.py based on --mode, keeping this
class mode-agnostic.

Performance notes for Pi 5:
  • Use imgsz=320 for fastest CPU inference (2–3× faster than .pt)
  • device defaults to "cpu"; set cfg.device to your AI HAT accelerator
    string when available — do not hardcode "cpu"
  • half=False on cpu (Pi 5 has no hardware FP16); set True on GPU
"""

import time
import logging
import threading
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
from ultralytics import YOLO

from tracker.config import SystemConfig
from tracker.camera import FrameSource
from tracker.kalman import BBoxKalmanFilter


@dataclass
class Detection:
    """Immutable detection result shared between threads."""
    cx: int            # Bounding box center X (pixels)
    cy: int            # Bounding box center Y (pixels)
    confidence: float  # YOLO confidence score
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    timestamp: float   # time.monotonic() at detection


@dataclass
class TrackState:
    """
    Kalman-predicted target state at read time.  Controller consumes this
    instead of the raw Detection so we compensate for inference latency
    and bridge dropped frames.
    """
    cx: int                  # predicted center X (px)
    cy: int                  # predicted center Y (px)
    w:  int                  # predicted bbox width (px)
    vx_px: float             # pixel velocity X (px/s)
    vy_px: float             # pixel velocity Y (px/s)
    vw_px: float             # bbox width growth rate (px/s) — TTI proxy
    confidence: float        # last detection confidence
    last_detection_ts: float # monotonic time of last raw detection
    age: float               # seconds since last detection
    fresh: bool              # last detection within freshness window


class VisionSystem:
    """
    Camera capture + YOLO inference running in a dedicated thread.

    Accepts any FrameSource (V4L2Camera for hardware, GazeboUDPStream for
    Gazebo mode).  External code reads `latest_detection` thread-safely.
    """

    def __init__(
        self,
        cfg: SystemConfig,
        source: FrameSource,
        shutdown: threading.Event,
    ):
        self.cfg      = cfg
        self.source   = source
        self.shutdown = shutdown
        self.log      = logging.getLogger("Vision")
        self.device   = cfg.device   # configurable; default "cpu"

        # ── Shared detection state ────────────────────────────────────────────
        self._lock: threading.Lock           = threading.Lock()
        self._detection: Optional[Detection] = None

        # ── Kalman tracker ────────────────────────────────────────────────────
        self._kf = BBoxKalmanFilter(
            process_noise=getattr(cfg, "kf_process_noise", 200.0),
            measurement_noise=getattr(cfg, "kf_measurement_noise", 9.0),
        )
        self._freshness_window: float = getattr(cfg, "track_freshness_sec", 0.25)

        # ── Performance counters ──────────────────────────────────────────────
        self._fps_counter = 0
        self._fps_last_ts = time.monotonic()
        self.actual_fps   = 0.0

        # ── Load YOLO model ───────────────────────────────────────────────────
        self.log.info(
            f"Loading YOLO model: {cfg.model_path}  (device={self.device})"
        )
        self.model = YOLO(cfg.model_path)
        # Warm-up pass so first inference isn't slow
        dummy = np.zeros((cfg.infer_size, cfg.infer_size, 3), dtype=np.uint8)
        self.model.predict(
            dummy, verbose=False, imgsz=cfg.infer_size,
            device=self.device,
        )
        self.log.info("Model loaded and warmed up")

        # ── Optional headless MJPEG viewer ────────────────────────────────────
        self._broadcaster = None
        web_port = getattr(cfg, "web_port", None)
        if web_port:
            from tracker.web_stream import FrameBroadcaster, start_server
            self._broadcaster = FrameBroadcaster()
            start_server(self._broadcaster, "0.0.0.0", web_port)

    # ── Public API (thread-safe) ───────────────────────────────────────────────

    @property
    def latest_detection(self) -> Optional[Detection]:
        """Return the most recent raw Detection or None."""
        with self._lock:
            return self._detection

    def get_track_state(self, now: float) -> Optional[TrackState]:
        """
        Return the Kalman-predicted target state at time `now`, or None if
        there has never been a detection or the track has gone stale.

        Call once per control tick with time.monotonic() — the filter
        compensates for latency elapsed since the last YOLO inference.
        """
        with self._lock:
            det = self._detection
        if det is None:
            return None

        age = now - det.timestamp
        if age > self.cfg.no_detect_timeout:
            return None

        pred = self._kf.predict_at(now)
        if pred is None:
            return None

        return TrackState(
            cx=int(pred["cx"]),
            cy=int(pred["cy"]),
            w=max(1, int(pred["w"])),
            vx_px=pred["vx"],
            vy_px=pred["vy"],
            vw_px=pred["vw"],
            confidence=det.confidence,
            last_detection_ts=det.timestamp,
            age=age,
            fresh=(age < self._freshness_window),
        )

    def reset_track(self) -> None:
        """Drop Kalman state — call after long target loss."""
        self._kf.reset()

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _update_detection(self, det: Optional[Detection]) -> None:
        with self._lock:
            self._detection = det
        if det is not None:
            w = det.bbox[2] - det.bbox[0]
            self._kf.update(det.cx, det.cy, w, det.timestamp)

    def _process_results(self, results) -> Optional[Detection]:
        """Extract the highest-confidence target detection from YOLO results."""
        best: Optional[Detection] = None
        best_conf = 0.0

        for result in results:
            boxes = result.boxes
            if boxes is None or len(boxes) == 0:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                conf   = float(box.conf[0])
                if cls_id not in self.cfg.target_classes:
                    continue
                if conf < self.cfg.conf_thresh:
                    continue
                if conf > best_conf:
                    best_conf = conf
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    best = Detection(
                        cx=cx, cy=cy,
                        confidence=conf,
                        bbox=(x1, y1, x2, y2),
                        timestamp=time.monotonic(),
                    )
        return best

    def _draw_overlay(
        self, frame: np.ndarray, det: Optional[Detection]
    ) -> np.ndarray:
        """Draw bounding box and HUD info onto frame for debug display."""
        h, w = frame.shape[:2]
        cx_frame, cy_frame = w // 2, h // 2

        # Crosshair at frame center
        cv2.line(frame, (cx_frame - 15, cy_frame), (cx_frame + 15, cy_frame), (0, 255, 0), 1)
        cv2.line(frame, (cx_frame, cy_frame - 15), (cx_frame, cy_frame + 15), (0, 255, 0), 1)

        mode_label = "[GAZEBO]" if self.cfg.is_gazebo else "[HW]"
        cv2.putText(frame, mode_label, (w - 100, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 200, 100), 1)

        if det:
            x1, y1, x2, y2 = det.bbox
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 120, 255), 2)
            cv2.circle(frame, (det.cx, det.cy), 4, (0, 120, 255), -1)
            cv2.line(frame, (cx_frame, cy_frame), (det.cx, det.cy), (255, 255, 0), 1)
            label = f"DRONE {det.confidence:.2f}"
            cv2.putText(frame, label, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 120, 255), 2)
            dx = det.cx - cx_frame
            dy = det.cy - cy_frame
            cv2.putText(frame, f"dx={dx:+d} dy={dy:+d}", (10, h - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1)
        else:
            cv2.putText(frame, "NO TARGET", (10, h - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)

        cv2.putText(frame, f"FPS: {self.actual_fps:.1f}", (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)
        return frame

    def _update_fps(self) -> None:
        self._fps_counter += 1
        now = time.monotonic()
        elapsed = now - self._fps_last_ts
        if elapsed >= 2.0:
            self.actual_fps   = self._fps_counter / elapsed
            self._fps_counter = 0
            self._fps_last_ts = now
            self.log.debug(f"Vision FPS: {self.actual_fps:.1f}")

    # ── Main thread entry point ────────────────────────────────────────────────

    def run(self) -> None:
        """VisionThread main loop.  Runs until shutdown_event is set."""
        self.log.info("VisionThread starting")

        try:
            while not self.shutdown.is_set():
                frame = self.source.read()
                if frame is None:
                    time.sleep(0.01)
                    continue

                results = self.model.predict(
                    frame,
                    imgsz=self.cfg.infer_size,
                    conf=self.cfg.conf_thresh,
                    iou=self.cfg.iou_thresh,
                    classes=list(self.cfg.target_classes),
                    verbose=False,
                    device=self.device,
                    half=(self.device != "cpu"),  # FP16 on GPU ~2× faster; off on CPU
                )

                det = self._process_results(results)
                self._update_detection(det)

                if self.cfg.show_display or self._broadcaster is not None:
                    annotated = self._draw_overlay(frame.copy(), det)

                    if self.cfg.show_display:
                        cv2.imshow("Drone Tracker", annotated)
                        if cv2.waitKey(1) & 0xFF == ord("q"):
                            self.log.info("Display quit key — triggering shutdown")
                            self.shutdown.set()

                    if self._broadcaster is not None:
                        self._broadcaster.update(annotated)

                self._update_fps()

        except Exception as e:
            self.log.exception(f"VisionThread crashed: {e}")
            self.shutdown.set()
        finally:
            self.source.release()
            if self.cfg.show_display:
                cv2.destroyAllWindows()
            self.log.info("VisionThread stopped")
