"""
tracker/config.py
=================
Central configuration for all subsystems.
Tune PID gains, velocity limits, and thresholds here.

Two run modes:
  "hardware" — Pi Camera via V4L2 + serial MAVLink to real flight controller
  "gazebo"   — UDP H.264 stream from Gazebo PC + MAVLink UDP to SITL
"""

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class PIDConfig:
    """PID gains for one axis."""
    kp: float = 0.15   # Proportional — immediate response to error
    ki: float = 0.01   # Integral     — corrects steady-state drift
    kd: float = 0.05   # Derivative   — damps oscillation


@dataclass
class SystemConfig:
    # ── Mode ─────────────────────────────────────────────────────────────────
    # "hardware" → Pi Camera + serial MAVLink
    # "gazebo"   → UDP stream from Gazebo PC + MAVLink UDP to SITL
    mode: str = "hardware"

    # ── Model ────────────────────────────────────────────────────────────────
    # ONNX export benchmarked on Pi 5 CPU: still the main FPS bottleneck
    # (inference-only isolation test showed ~7-8 FPS vs. ~400+ FPS with
    # inference skipped). Dropping imgsz further trades accuracy for FPS —
    # re-export whenever this changes: model.export(format="onnx", imgsz=256, simplify=True)
    model_path:     str   = "models/drone_yolo.onnx"
    conf_thresh:    float = 0.20      # YOLO confidence minimum
    iou_thresh:     float = 0.45      # NMS IoU threshold
    # Class indices to track. {0:'drone', 1:'interveptor-drone', 2:'fixedwing'}
    target_classes: tuple = (0, 1)
    # Must match the imgsz the ONNX model was exported with (fixed, dynamic=False).
    # 896 was the trained resolution; 256 trades detection range/accuracy for FPS.
    infer_size:     int   = 256
    # YOLO inference device.
    # Valid values: "cpu" | future AI HAT accelerator string (e.g. "hailo")
    # Default is "cpu" — Pi 5 has no GPU. Change to your accelerator when the
    # AI HAT is installed; do NOT hardcode "cpu" in the inference call.
    device:         str   = "cpu"

    # ── Camera ───────────────────────────────────────────────────────────────
    frame_width:    int   = 640
    frame_height:   int   = 480
    target_fps:     int   = 60        # request high FPS — actual capped by sensor
    camera_index:   int   = 0         # /dev/video0 by default

    # ── MAVLink ──────────────────────────────────────────────────────────────
    serial_device:  str   = "/dev/ttyAMA0"
    baud_rate:      int   = 921600    # high baud for low-latency at 50 Hz control
    heartbeat_timeout: float = 5.0
    # Attempts to wait for the first heartbeat at connect() time, heartbeat_timeout
    # seconds apart. The FC often isn't booted yet when this script starts (e.g.
    # started the moment the battery goes in), so retry instead of aborting on
    # the first miss.
    heartbeat_connect_retries: int = 6
    mavlink_system_id: int = 255
    mavlink_comp_id:   int = 0

    # ── Velocity limits (m/s) ────────────────────────────────────────────────
    max_yaw_rate:   float = 40.0      # deg/s
    max_pitch_vel:  float = 30.0      # m/s forward
    max_roll_vel:   float = 15.0      # m/s left/right
    max_vz:         float = 8.0       # m/s vertical correction

    # ── PID controllers ──────────────────────────────────────────────────────
    pid_yaw:      PIDConfig = field(default_factory=lambda: PIDConfig(kp=0.15, ki=0.0,   kd=0.02))
    pid_pitch:    PIDConfig = field(default_factory=lambda: PIDConfig(kp=0.12, ki=0.0,   kd=0.06))
    pid_approach: PIDConfig = field(default_factory=lambda: PIDConfig(kp=0.08, ki=0.005, kd=0.01))

    # ── Tracking logic ───────────────────────────────────────────────────────
    dead_zone_px:      int   = 8
    no_detect_timeout: float = 0.5
    control_rate_hz:   float = 50.0

    # ── Kalman tracker tuning ────────────────────────────────────────────────
    kf_process_noise:     float = 200.0
    kf_measurement_noise: float = 9.0
    track_freshness_sec:  float = 0.25

    # ── Approach (close-in) control ──────────────────────────────────────────
    approach_target_size_px: int = 600
    approach_align_px:       int = 200

    # ── Display ──────────────────────────────────────────────────────────────
    show_display: bool = False
    web_port: Optional[int] = None   # if set, serve annotated frames as MJPEG on this port

    # ── Gazebo / UDP stream settings ─────────────────────────────────────────
    # Set gst_udp_host to the Gazebo PC's LAN IP when running networked
    # (tracker on Pi 5, Gazebo on desktop).  Default 127.0.0.1 is for
    # same-machine testing only.
    gst_udp_host: str = "127.0.0.1"
    gst_udp_port: int = 5600
    # MAVLink UDP connection string for ArduPilot SITL.
    # Set to udp:<DESKTOP_IP>:14550 for networked Pi↔desktop setup.
    sitl_connection: str = "udp:127.0.0.1:14550"

    # ── Derived helpers (not constructor args) ────────────────────────────────

    @property
    def is_gazebo(self) -> bool:
        """True when running in Gazebo/SITL mode."""
        return self.mode == "gazebo"

    @property
    def frame_cx(self) -> int:
        return self.frame_width // 2

    @property
    def frame_cy(self) -> int:
        return self.frame_height // 2

    @property
    def control_period(self) -> float:
        return 1.0 / self.control_rate_hz
