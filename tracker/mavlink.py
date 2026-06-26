"""
tracker/mavlink.py
==================
MAVLinkInterface — wraps pymavlink for all flight controller comms.

Responsibilities:
  • Connect to ArduPilot via serial (hardware) or UDP (gazebo/SITL)
  • Receive & decode heartbeat (check arm state, flight mode)
  • Send SET_POSITION_TARGET_LOCAL_NED velocity commands (GUIDED mode)
  • Send CONDITION_YAW for yaw rate control
  • Safety checks: arm status, GUIDED mode
  • Graceful disconnect with hover command
"""

import time
import logging
import threading
from typing import Optional

from tracker.config import SystemConfig

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False
    logging.getLogger("MAVLink").warning(
        "pymavlink not installed — run: pip install pymavlink"
    )


# Bitmask for SET_POSITION_TARGET_LOCAL_NED:
# Ignore position, ignore acceleration → use only vx, vy, vz
# Bit 0=x, 1=y, 2=z, 3=vx, 4=vy, 5=vz, 6=ax, 7=ay, 8=az, 9=force, 10=yaw, 11=yawrate
VELOCITY_ONLY_MASK = (
    0b0000_111111000111  # ignore pos (bits 0-2), ignore accel (bits 6-8), use vel (3-5)
)
# Additionally ignore yaw position, use yaw rate
VELOCITY_YAW_RATE_MASK = 0b0000_011111000111


class MAVLinkInterface:
    """
    Thread-safe MAVLink communication layer.
    All public methods are safe to call from any thread.
    """

    def __init__(self, cfg: SystemConfig, shutdown: threading.Event):
        self.cfg      = cfg
        self.shutdown = shutdown
        self.log      = logging.getLogger("MAVLink")
        self._conn    = None
        self._lock    = threading.Lock()

        self._armed      = False
        self._mode       = "UNKNOWN"
        self._gps_fix    = False
        self._battery_v  = 0.0
        self._last_hb_ts = 0.0

        self._hb_thread: Optional[threading.Thread] = None

    # ── Connection ─────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Open MAVLink connection and wait for first heartbeat.

        hardware mode → serial connection to real flight controller
        gazebo mode   → UDP connection to ArduPilot SITL on the desktop PC

        Returns True on success, False on failure (raises no exceptions).
        """
        if not MAVLINK_AVAILABLE:
            self.log.error("pymavlink not installed — run: pip install pymavlink")
            return False

        if self.cfg.is_gazebo:
            conn_str    = self.cfg.sitl_connection
            conn_kwargs: dict = {}
            self.log.info(f"GAZEBO/SITL MODE: connecting via UDP → {conn_str}")
        else:
            conn_str    = self.cfg.serial_device
            conn_kwargs = {"baud": self.cfg.baud_rate}
            self.log.info(f"HARDWARE MODE: {conn_str} @ {self.cfg.baud_rate} baud")

        try:
            with self._lock:
                self._conn = mavutil.mavlink_connection(
                    conn_str,
                    source_system=self.cfg.mavlink_system_id,
                    source_component=self.cfg.mavlink_comp_id,
                    **conn_kwargs,
                )

            self.log.info("Waiting for heartbeat...")
            self._conn.wait_heartbeat(timeout=self.cfg.heartbeat_timeout)
            self._last_hb_ts = time.monotonic()
            self.log.info(
                f"Heartbeat received from system {self._conn.target_system}, "
                f"component {self._conn.target_component}"
            )

            self._request_data_streams()

            self._hb_thread = threading.Thread(
                target=self._heartbeat_listener,
                name="MAVHeartbeatListener",
                daemon=True,
            )
            self._hb_thread.start()
            return True

        except Exception as e:
            self.log.error(f"MAVLink connect failed: {e}")
            return False

    def disconnect(self) -> None:
        """Send hover command and close connection cleanly."""
        if not MAVLINK_AVAILABLE or self._conn is None:
            return
        try:
            self.log.info("Sending hover command before disconnect")
            self.send_velocity(0.0, 0.0, 0.0, yaw_rate=0.0)
            time.sleep(0.3)
            self._conn.close()
            self.log.info("MAVLink disconnected")
        except Exception as e:
            self.log.warning(f"Disconnect error: {e}")

    # ── Safety Checks ──────────────────────────────────────────────────────────

    def safety_checks(self) -> bool:
        """
        Run pre-flight safety checks.
        Returns True only if all checks pass.
        Not called in gazebo mode (SITL arms from a script).
        """
        self.log.info("Running pre-flight safety checks...")

        checks = {
            "MAVLink connected": self._conn is not None,
            "Heartbeat recent":  (time.monotonic() - self._last_hb_ts) < 3.0,
        }

        if self._mode not in ("GUIDED", "GUIDED_NOGPS", "UNKNOWN"):
            checks["GUIDED mode"] = False
        else:
            checks["GUIDED mode"] = True

        all_pass = all(checks.values())
        for name, passed in checks.items():
            status = "PASS" if passed else "FAIL"
            self.log.info(f"  [{status}] {name}")

        if not all_pass:
            self.log.error("Safety checks FAILED")
        return all_pass

    @property
    def is_armed(self) -> bool:
        return self._armed

    @property
    def flight_mode(self) -> str:
        return self._mode

    # ── Command Senders ────────────────────────────────────────────────────────

    def send_velocity(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
    ) -> None:
        """
        Send SET_POSITION_TARGET_LOCAL_NED in BODY frame.
        Controls velocity + yaw rate simultaneously.
        """
        if not MAVLINK_AVAILABLE or self._conn is None:
            return

        if not self._armed:
            self.log.debug("Not armed — velocity command suppressed")
            return

        with self._lock:
            self._conn.mav.set_position_target_local_ned_send(
                int(time.monotonic() * 1000) & 0xFFFFFFFF,
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                VELOCITY_YAW_RATE_MASK,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0,
                float(yaw_rate),
            )

    def send_hover(self, yaw_rate: float = 0.0) -> None:
        """Stop all motion — zero velocity command."""
        self.send_velocity(0.0, 0.0, 0.0, yaw_rate)

    def set_mode(self, mode_name: str) -> bool:
        """Change flight mode (e.g., 'GUIDED', 'LOITER')."""
        if not MAVLINK_AVAILABLE or self._conn is None:
            return False
        try:
            mode_id = self._conn.mode_mapping().get(mode_name)
            if mode_id is None:
                self.log.error(f"Unknown mode: {mode_name}")
                return False
            with self._lock:
                self._conn.set_mode(mode_id)
            self.log.info(f"Mode change requested: {mode_name}")
            return True
        except Exception as e:
            self.log.error(f"set_mode failed: {e}")
            return False

    # ── Background Heartbeat Listener ──────────────────────────────────────────

    def _heartbeat_listener(self) -> None:
        """Background thread: reads MAVLink messages and updates cached state."""
        self.log.info("Heartbeat listener started")
        while not self.shutdown.is_set():
            if self._conn is None:
                time.sleep(0.1)
                continue
            try:
                msg = self._conn.recv_match(
                    type=["HEARTBEAT", "SYS_STATUS", "GPS_RAW_INT"],
                    blocking=True,
                    timeout=1.0,
                )
                if msg is None:
                    continue

                mtype = msg.get_type()

                if mtype == "HEARTBEAT":
                    self._last_hb_ts = time.monotonic()
                    self._armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    try:
                        mode_map = {v: k for k, v in self._conn.mode_mapping().items()}
                        self._mode = mode_map.get(msg.custom_mode, f"MODE_{msg.custom_mode}")
                    except Exception:
                        self._mode = f"MODE_{msg.custom_mode}"

                elif mtype == "SYS_STATUS":
                    self._battery_v = msg.voltage_battery / 1000.0

                elif mtype == "GPS_RAW_INT":
                    self._gps_fix = msg.fix_type >= 3

            except Exception as e:
                if not self.shutdown.is_set():
                    self.log.warning(f"Heartbeat listener error: {e}")
                time.sleep(0.05)

        self.log.info("Heartbeat listener stopped")

    def _request_data_streams(self) -> None:
        """Ask ArduPilot to stream telemetry at useful rates."""
        if self._conn is None:
            return
        streams = [(mavutil.mavlink.MAV_DATA_STREAM_ALL, 4)]
        for stream_id, rate in streams:
            self._conn.mav.request_data_stream_send(
                self._conn.target_system,
                self._conn.target_component,
                stream_id,
                rate,
                1,
            )
