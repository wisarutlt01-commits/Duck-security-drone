"""
tracker/kalman.py
=================
Constant-velocity Kalman filter for bounding-box tracking.

State  (6D): [cx, cy, w, vx, vy, vw]     positions (px) + velocities (px/s)
Measurement (3D): [cx, cy, w]            YOLO bbox center + width

Why this matters for high-speed intercept:
  • YOLO inference + control loop add ~100–150 ms latency. At 100 km/h
    (~27.8 m/s) the world moves ~3–4 m per cycle. Predicting the target
    forward by `dt = now - last_detection_ts` compensates for that lag.
  • Bridges missed detections — the filter keeps producing predictions
    even when YOLO drops a frame, so the controller never goes blind
    for a single dropout.
  • vw (bbox width growth rate) is a free time-to-impact proxy.
"""

import threading
import numpy as np


class BBoxKalmanFilter:
    """
    Thread-safe constant-velocity Kalman filter for a single tracked bbox.

    Public API:
        update(cx, cy, w, ts)       — fold in a new YOLO measurement
        predict_at(ts)              — read predicted state at time ts (no update)
        reset()                     — drop state on track loss

    Tuning:
        process_noise     — higher → trusts measurements more (snappier,
                            noisier). Lower → smoother but laggier.
        measurement_noise — YOLO bbox jitter variance (px²).
    """

    def __init__(
        self,
        process_noise: float = 200.0,
        measurement_noise: float = 9.0,
    ):
        self._lock = threading.Lock()
        self._initialized = False

        self.x = np.zeros(6, dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * 100.0

        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

        self.q = float(process_noise)
        self.R = np.eye(3, dtype=np.float64) * float(measurement_noise)

        self.last_update_ts: float = 0.0

    @staticmethod
    def _F(dt: float) -> np.ndarray:
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        return F

    def _Q(self, dt: float) -> np.ndarray:
        q   = self.q
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        Q   = np.zeros((6, 6), dtype=np.float64)
        for pos, vel in ((0, 3), (1, 4), (2, 5)):
            Q[pos, pos] = dt4 / 4.0 * q
            Q[pos, vel] = dt3 / 2.0 * q
            Q[vel, pos] = dt3 / 2.0 * q
            Q[vel, vel] = dt2 * q
        return Q

    def update(self, cx: float, cy: float, w: float, ts: float) -> None:
        with self._lock:
            if not self._initialized:
                self.x[:] = 0.0
                self.x[0], self.x[1], self.x[2] = cx, cy, w
                self.P = np.eye(6, dtype=np.float64) * 10.0
                self.last_update_ts = ts
                self._initialized = True
                return

            dt = ts - self.last_update_ts
            if dt <= 0.0:
                dt = 1e-3
            dt = min(dt, 0.5)

            F = self._F(dt)
            self.x = F @ self.x
            self.P = F @ self.P @ F.T + self._Q(dt)

            z = np.array([cx, cy, w], dtype=np.float64)
            y = z - self.H @ self.x
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)
            self.x = self.x + K @ y
            self.P = (np.eye(6, dtype=np.float64) - K @ self.H) @ self.P

            self.last_update_ts = ts

    def predict_at(self, ts: float):
        """
        Predict state at time `ts`.  Does NOT mutate the filter.
        Returns dict or None if filter has no measurements yet.
        """
        with self._lock:
            if not self._initialized:
                return None
            dt = ts - self.last_update_ts
            if dt < 0.0:
                dt = 0.0
            dt = min(dt, 1.0)
            x_pred = self._F(dt) @ self.x
            return {
                "cx":  float(x_pred[0]),
                "cy":  float(x_pred[1]),
                "w":   float(x_pred[2]),
                "vx":  float(x_pred[3]),
                "vy":  float(x_pred[4]),
                "vw":  float(x_pred[5]),
                "age": float(dt),
            }

    def reset(self) -> None:
        with self._lock:
            self._initialized = False
            self.x[:] = 0.0
            self.P = np.eye(6, dtype=np.float64) * 100.0
            self.last_update_ts = 0.0

    @property
    def initialized(self) -> bool:
        with self._lock:
            return self._initialized
