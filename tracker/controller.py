"""
tracker/controller.py
=====================
FlightController — Thread 2
----------------------------
Responsibilities:
  • Read latest detection from VisionSystem at fixed control rate
  • Compute pixel error (target center - frame center)
  • Apply dead-zone: ignore tiny errors (avoids constant jitter)
  • Run PID on yaw (horizontal error) and pitch/forward (vertical error)
  • Convert PID output → body-frame velocity + yaw rate commands
  • Send commands via MAVLinkInterface
  • Detect tracking loss → switch to hover after timeout
  • State machine: SEARCHING | TRACKING | HOVERING

Coordinate mapping (ArduPilot body frame NED):
  • dx > 0 (target right of center)  → yaw right (positive yaw rate)
  • dx < 0 (target left of center)   → yaw left  (negative yaw rate)
  • dy > 0 (target below center)     → descend   (positive vz in NED = down)
  • dy < 0 (target above center)     → climb     (negative vz)
"""

import time
import logging
import threading
from enum import Enum, auto

from tracker.config import SystemConfig
from tracker.vision import VisionSystem, TrackState
from tracker.mavlink import MAVLinkInterface
from tracker.pid import PIDController


class TrackerState(Enum):
    SEARCHING = auto()   # No recent detection
    TRACKING  = auto()   # Actively following target
    HOVERING  = auto()   # Lost target for > timeout → hold position


class FlightController:
    """
    PID-based tracking controller running at fixed control_rate_hz.
    Reads VisionSystem detections, outputs MAVLink velocity commands.
    """

    def __init__(
        self,
        cfg:      SystemConfig,
        mavlink:  MAVLinkInterface,
        vision:   VisionSystem,
        shutdown: threading.Event,
    ):
        self.cfg      = cfg
        self.mavlink  = mavlink
        self.vision   = vision
        self.shutdown = shutdown
        self.log      = logging.getLogger("Controller")

        # ── State machine ─────────────────────────────────────────────────────
        self.state           = TrackerState.SEARCHING
        self._last_detect_ts = 0.0
        self._track_count    = 0

        # ── PID controllers ───────────────────────────────────────────────────
        self.pid_yaw = PIDController(
            kp=cfg.pid_yaw.kp,
            ki=cfg.pid_yaw.ki,
            kd=cfg.pid_yaw.kd,
            output_limit=cfg.max_yaw_rate,
        )
        self.pid_pitch = PIDController(
            kp=cfg.pid_pitch.kp,
            ki=cfg.pid_pitch.ki,
            kd=cfg.pid_pitch.kd,
            output_limit=cfg.max_vz,
        )
        self.pid_approach = PIDController(
            kp=cfg.pid_approach.kp,
            ki=cfg.pid_approach.ki,
            kd=cfg.pid_approach.kd,
            output_limit=cfg.max_pitch_vel,
        )

        # ── Timing ────────────────────────────────────────────────────────────
        self._last_cmd_ts  = time.monotonic()
        self._loop_count   = 0
        self._log_interval = max(1, int(cfg.control_rate_hz * 2))

    # ── Main control loop ──────────────────────────────────────────────────────

    def run(self) -> None:
        """ControlThread main loop.  Runs at cfg.control_rate_hz."""
        self.log.info(f"ControlThread starting at {self.cfg.control_rate_hz} Hz")

        period   = self.cfg.control_period
        next_run = time.monotonic()

        try:
            while not self.shutdown.is_set():
                now = time.monotonic()

                if now < next_run:
                    time.sleep(next_run - now)
                    now = time.monotonic()
                next_run = now + period

                dt = now - self._last_cmd_ts
                self._last_cmd_ts = now
                dt = max(0.001, min(dt, 0.5))

                self._step(now, dt)
                self._loop_count += 1

        except Exception as e:
            self.log.exception(f"ControlThread crashed: {e}")
            self.shutdown.set()
        finally:
            self.mavlink.send_hover()
            self.log.info("ControlThread stopped — hover command sent")

    def _step(self, now: float, dt: float) -> None:
        """Single control step."""
        track = self.vision.get_track_state(now)

        if track is not None:
            self._last_detect_ts = track.last_detection_ts
            new_state = TrackerState.TRACKING
        else:
            elapsed_no_detect = now - self._last_detect_ts
            if elapsed_no_detect > self.cfg.no_detect_timeout:
                new_state = TrackerState.HOVERING
            else:
                new_state = TrackerState.SEARCHING

        if new_state != self.state:
            self.log.info(f"State: {self.state.name} → {new_state.name}")
            if new_state in (TrackerState.SEARCHING, TrackerState.HOVERING):
                self.pid_yaw.reset()
                self.pid_pitch.reset()
                self.pid_approach.reset()
            if new_state == TrackerState.HOVERING:
                self.vision.reset_track()
            self.state = new_state

        if self.state == TrackerState.TRACKING:
            self._do_tracking(track, dt)
        elif self.state == TrackerState.HOVERING:
            self.mavlink.send_hover()
        elif self.state == TrackerState.SEARCHING:
            self.mavlink.send_hover()

        if self._loop_count % self._log_interval == 0:
            self.log.debug(
                f"State={self.state.name} | "
                f"Armed={self.mavlink.is_armed} | "
                f"Mode={self.mavlink.flight_mode} | "
                f"Vision FPS={self.vision.actual_fps:.1f}"
            )

    def _do_tracking(self, track: TrackState, dt: float) -> None:
        """
        Compute PID commands from Kalman-predicted target state and send to
        MAVLink.

        Coordinate system:
          dx = track.cx - frame_cx  → positive = target is RIGHT of center
          dy = track.cy - frame_cy  → positive = target is BELOW center (Y flipped)

        Inputs come from the Kalman filter (tracker/kalman.py), already projected
        forward to `now` — so (dx, dy, bbox_w) represent where we believe the
        target IS, not where it was 100 ms ago when YOLO ran.
        """
        dx = track.cx - self.cfg.frame_cx
        dy = track.cy - self.cfg.frame_cy

        if abs(dx) < self.cfg.dead_zone_px:
            dx = 0.0
        if abs(dy) < self.cfg.dead_zone_px:
            dy = 0.0

        yaw_rate = self.pid_yaw.update(dx, dt)
        vz       = self.pid_pitch.update(dy, dt)

        yaw_rate = max(-self.cfg.max_yaw_rate, min(self.cfg.max_yaw_rate, yaw_rate))
        vz       = max(-self.cfg.max_vz,       min(self.cfg.max_vz, vz))

        bbox_w   = track.w
        size_err = self.cfg.approach_target_size_px - bbox_w
        if abs(dx) < self.cfg.approach_align_px:
            vx = self.pid_approach.update(size_err, dt)
            vx = max(-self.cfg.max_pitch_vel, min(self.cfg.max_pitch_vel, vx))
        else:
            vx = 0.0
            self.pid_approach.reset()

        if track.vw_px > 1.0 and bbox_w > 0:
            tti = max(0.0, (self.cfg.approach_target_size_px - bbox_w) / track.vw_px)
        else:
            tti = float("inf")

        self.log.debug(
            f"[TRACK] dx={dx:+.0f}px dy={dy:+.0f}px w={bbox_w}px "
            f"vw={track.vw_px:+.0f}px/s tti={tti:.2f}s age={track.age*1000:.0f}ms | "
            f"yaw={yaw_rate:+.2f}°/s vz={vz:+.3f}m/s vx={vx:+.3f}m/s "
            f"conf={track.confidence:.2f}"
        )

        self.mavlink.send_velocity(vx=vx, vy=0.0, vz=vz, yaw_rate=yaw_rate)
