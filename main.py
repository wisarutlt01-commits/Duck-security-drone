#!/usr/bin/env python3
"""
Drone Tracking System — Main Entry Point
=========================================
Two run modes:

  HARDWARE (default):
    python main.py --mode hardware \\
        --model models/drone_yolo.pt \\
        --device /dev/ttyAMA0 --baud 57600

  GAZEBO (tracker on Pi 5, Gazebo+SITL on desktop PC):
    # On the DESKTOP: launch Gazebo + SITL, stream camera to <PI_IP>:5600,
    #                 expose SITL MAVLink to <PI_IP>:14550
    #                 (see gazebo/README_GAZEBO.md)
    # On the PI:
    python main.py --mode gazebo \\
        --model models/drone_yolo.pt \\
        --sitl-conn udp:<DESKTOP_IP>:14550 \\
        --gst-host 0.0.0.0 --gst-port 5600
"""

import os

# Must be set before onnxruntime/torch/numpy load their threading runtime —
# Pi 5 is quad-core; reserve 1 core for the GStreamer decode + MAVLink/control
# threads instead of letting inference claim all 4.
os.environ.setdefault("OMP_NUM_THREADS", "3")
os.environ.setdefault("OPENBLAS_NUM_THREADS", "3")

import time
import signal
import logging
import argparse
import threading

# Lightweight imports only at module level — argparse help works without deps
from tracker.config import SystemConfig
from tracker.logger import setup_logger

shutdown_event = threading.Event()


def signal_handler(sig, frame):
    logging.getLogger("main").info("Shutdown signal received")
    shutdown_event.set()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Drone Tracker — intercept an aerial target with a drone.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--mode",
        choices=["hardware", "gazebo"],
        default="hardware",
        help="Run mode: 'hardware' (Pi Camera + serial MAVLink) or "
             "'gazebo' (UDP stream + MAVLink UDP to SITL). Default: hardware",
    )
    parser.add_argument("--model",      default="models/drone_yolo.pt",
                        help="Path to YOLO .pt model weights")
    parser.add_argument("--conf",       type=float, default=0.1,
                        help="YOLO confidence threshold")
    parser.add_argument("--infer-device", dest="infer_device", default=None,
                        metavar="DEVICE",
                        help="YOLO inference device (default: cfg.device = 'cpu'). "
                             "Override here for testing, e.g. 'cpu' or 'hailo'.")
    parser.add_argument("--device",     default="/dev/ttyAMA0",
                        help="Serial device for MAVLink (hardware mode)")
    parser.add_argument("--baud",       type=int, default=57600,
                        help="Serial baud rate (hardware mode)")
    parser.add_argument("--sitl-conn",  default="udp:127.0.0.1:14550",
                        dest="sitl_conn",
                        help="MAVLink UDP connection string for SITL (gazebo mode)")
    parser.add_argument("--gst-port",   type=int, default=5600,
                        metavar="PORT",
                        help="UDP port for H.264/RTP camera stream (gazebo mode)")
    parser.add_argument("--gst-host",   default="0.0.0.0",
                        metavar="HOST",
                        help="UDP listen address for camera stream (gazebo mode)")
    parser.add_argument("--width",      type=int, default=640)
    parser.add_argument("--height",     type=int, default=480)
    parser.add_argument("--fps",        type=int, default=30)
    parser.add_argument("--display",    action="store_true",
                        help="Show OpenCV preview window (requires Qt/X11)")
    parser.add_argument("--web-port",   type=int, default=None,
                        metavar="PORT",
                        help="Serve annotated frames as MJPEG over HTTP on this "
                             "port (e.g. 8080) — view at http://<host>:PORT/. "
                             "No X11/Qt needed; works headless over SSH/Tailscale.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logger = setup_logger("main", log_file="logs/tracker.log")

    cfg = SystemConfig(
        mode              = args.mode,
        model_path        = args.model,
        conf_thresh       = args.conf,
        serial_device     = args.device,
        baud_rate         = args.baud,
        frame_width       = args.width,
        frame_height      = args.height,
        target_fps        = args.fps,
        show_display      = args.display,
        web_port          = args.web_port,
        sitl_connection   = args.sitl_conn,
        gst_udp_host      = args.gst_host,
        gst_udp_port      = args.gst_port,
    )

    # Optional CLI override for inference device
    if args.infer_device is not None:
        cfg.device = args.infer_device

    mode_str = "GAZEBO/SITL" if cfg.is_gazebo else "HARDWARE"
    logger.info("=" * 60)
    logger.info(f"Drone Tracker — {mode_str} mode")
    logger.info(f"  Model  : {cfg.model_path}")
    logger.info(f"  Device : {cfg.device}")
    if cfg.is_gazebo:
        logger.info(f"  SITL   : {cfg.sitl_connection}")
        logger.info(
            f"  Camera : UDP  udp://{cfg.gst_udp_host}:{cfg.gst_udp_port}"
        )
    else:
        logger.info(f"  Serial : {cfg.serial_device} @ {cfg.baud_rate}")
    logger.info("=" * 60)

    signal.signal(signal.SIGINT,  signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # ── Build the FrameSource for the chosen mode ─────────────────────────────
    if cfg.is_gazebo:
        from tracker.camera import GazeboUDPStream
        source = GazeboUDPStream(
            host=cfg.gst_udp_host,
            port=cfg.gst_udp_port,
            width=cfg.frame_width,
            height=cfg.frame_height,
        )
        logger.info("FrameSource: GazeboUDPStream")
    else:
        from tracker.camera import V4L2Camera
        source = V4L2Camera(
            camera_index=cfg.camera_index,
            width=cfg.frame_width,
            height=cfg.frame_height,
            fps=cfg.target_fps,
        )
        logger.info("FrameSource: V4L2Camera")

    # ── Instantiate subsystems ────────────────────────────────────────────────
    from tracker.vision import VisionSystem
    from tracker.mavlink import MAVLinkInterface
    from tracker.controller import FlightController
    vision_sys    = VisionSystem(cfg, source, shutdown_event)
    mavlink_iface = MAVLinkInterface(cfg, shutdown_event)
    controller    = FlightController(cfg, mavlink_iface, vision_sys, shutdown_event)

    if not mavlink_iface.connect():
        logger.error("MAVLink connection failed — aborting.")
        return 1

    # Safety checks for hardware mode only (SITL arms itself)
    if not cfg.is_gazebo and not mavlink_iface.safety_checks():
        logger.error("Safety checks FAILED — aborting.")
        mavlink_iface.disconnect()
        return 1

    if cfg.is_gazebo:
        logger.info("SITL: requesting GUIDED mode")
        mavlink_iface.set_mode("GUIDED")
        time.sleep(1.0)

    vision_thread  = threading.Thread(
        target=vision_sys.run,  name="VisionThread",  daemon=True
    )
    control_thread = threading.Thread(
        target=controller.run,  name="ControlThread", daemon=True
    )

    vision_thread.start()
    logger.info("VisionThread started")
    time.sleep(1.0)
    control_thread.start()
    logger.info("ControlThread started")

    try:
        while not shutdown_event.is_set():
            if not vision_thread.is_alive():
                logger.error("VisionThread died")
                shutdown_event.set()
            if not control_thread.is_alive():
                logger.error("ControlThread died")
                shutdown_event.set()
            time.sleep(0.5)
    finally:
        vision_thread.join(timeout=3)
        control_thread.join(timeout=3)
        mavlink_iface.disconnect()
        logger.info("Shutdown complete.")

    return 0


if __name__ == "__main__":
    exit(main())
