# Drone Tracker

Intercept an aerial target drone using a Raspberry Pi 5, a YOLO detection model,
and ArduPilot MAVLink velocity commands.

Two run modes:

| Mode | Camera | MAVLink | Typical hardware |
|------|--------|---------|-----------------|
| `hardware` | Pi Camera via V4L2 | Serial `/dev/ttyAMA0` | Pi 5 on a real drone |
| `gazebo` | H.264/RTP UDP from desktop PC | UDP to ArduPilot SITL | Pi 5 + desktop PC on same LAN |

---

## Install

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

---

## Mode 1: Hardware (real flight)

Connect the Pi Camera and a serial MAVLink link to your flight controller, then:

```bash
python main.py --mode hardware \
    --model models/drone_yolo.pt \
    --device /dev/ttyAMA0 --baud 57600 \
    --conf 0.3
```

The tracker will:
1. Open the Pi Camera via V4L2.
2. Run YOLO inference on CPU (default `device=cpu`; change in `tracker/config.py`
   when an AI HAT is fitted).
3. Connect to the flight controller over serial and wait for a heartbeat.
4. Run safety checks (GUIDED mode, heartbeat recent).
5. Start the PID tracking loop.

Useful flags:
```
--conf FLOAT        YOLO confidence threshold (default: 0.1)
--width / --height  Camera resolution (default: 640x480)
--fps INT           Target camera FPS (default: 30)
--display           Show OpenCV preview window
--infer-device STR  Override inference device (e.g. "hailo" for AI HAT)
```

---

## Mode 2: Gazebo (networked Pi 5 ↔ desktop PC)

**On the desktop PC:** start Gazebo and ArduPilot SITL (see `gazebo/README_GAZEBO.md`).
The desktop sends the camera stream to the Pi's IP on UDP port 5600 and exposes
SITL's MAVLink on UDP 14550.

**On the Pi 5:**

```bash
python main.py --mode gazebo \
    --model models/drone_yolo.pt \
    --sitl-conn udp:<DESKTOP_IP>:14550 \
    --gst-host 0.0.0.0 --gst-port 5600
```

The tracker will:
1. Listen for the H.264/RTP UDP stream from Gazebo (GStreamer preferred, ffmpeg fallback).
2. Connect to the SITL MAVLink UDP endpoint on the desktop.
3. Set GUIDED mode and start the PID tracking loop.

See `gazebo/README_GAZEBO.md` for full setup, networking details, and the
protobuf fix for Gazebo Harmonic on Ubuntu 22.04.

---

## Quick camera test (no MAVLink needed)

```bash
python tools/test_camera.py --model models/drone_yolo.pt --display
```

---

## Project layout

```
drone_tracker/
├── main.py                # entry point: --mode hardware|gazebo
├── requirements.txt
├── tracker/               # Python package
│   ├── config.py          # SystemConfig dataclass (all tunable parameters)
│   ├── vision.py          # VisionSystem: YOLO + Kalman tracker (thread 1)
│   ├── camera.py          # FrameSource: V4L2Camera | GazeboUDPStream
│   ├── controller.py      # FlightController: PID loop (thread 2)
│   ├── mavlink.py         # MAVLinkInterface: serial/UDP MAVLink
│   ├── pid.py             # PIDController
│   ├── kalman.py          # BBoxKalmanFilter
│   └── logger.py          # setup_logger
├── tools/
│   └── test_camera.py     # standalone camera + YOLO smoke test
├── models/
│   └── drone_yolo.pt      # trained YOLO weights (gitignored)
└── gazebo/                # desktop-side Gazebo assets
    ├── README_GAZEBO.md   # setup + networking guide
    ├── worlds/
    ├── models/
    ├── launch/            # start_sim.sh, fix_and_start.sh
    └── scripts/
        └── target_mover.py
```

---

## Configuration

All tunable parameters live in `tracker/config.py` (`SystemConfig`).  Key fields:

| Field | Default | Description |
|-------|---------|-------------|
| `device` | `"cpu"` | YOLO inference device. Change to your AI HAT string when available. |
| `conf_thresh` | `0.20` | YOLO confidence threshold |
| `infer_size` | `896` | YOLO input resolution. Use 640 on CPU for speed. |
| `pid_yaw.kp` | `0.15` | Yaw PID proportional gain |
| `gst_udp_host` | `"127.0.0.1"` | Set to Gazebo PC's LAN IP for networked mode |
| `sitl_connection` | `"udp:127.0.0.1:14550"` | Set to `udp:<DESKTOP_IP>:14550` for networked mode |

---

## Adding an AI HAT later

When your AI HAT is installed, change one line in `tracker/config.py`:

```python
device: str = "hailo"   # or whatever the HAT's device string is
```

Or pass it at runtime:
```bash
python main.py --mode hardware --infer-device hailo ...
```
