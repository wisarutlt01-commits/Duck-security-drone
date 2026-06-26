#!/usr/bin/env python3
"""
sim/scripts/target_mover.py
============================
Controls the TARGET drone (SITL instance 1) to fly interesting patterns
so the tracker has something to chase.

Connects to ArduPilot SITL instance 1 via UDP 14560.
Flies a sequence of waypoint patterns:
  - Circle
  - Figure-8
  - Random wandering

Run this in a separate terminal AFTER starting the full sim:
    python sim/scripts/target_mover.py

Options:
    --pattern  circle | figure8 | random | zigzag
    --speed    m/s  (default 2.0)
    --altitude m    (default 5.0)
    --conn     MAVLink connection string (default udp:127.0.0.1:14560)
"""

import time
import math
import random
import argparse
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("TargetMover")

try:
    from pymavlink import mavutil
except ImportError:
    raise SystemExit("pymavlink not installed. Run: pip install pymavlink")


# ── MAVLink velocity bitmask: use vx, vy, vz only ─────────────────────────────
VEL_MASK = 0b0000_011111000111


class TargetDroneMover:
    """
    Controls target drone via MAVLink velocity commands.
    Flies preset patterns to test tracker drone's tracking ability.
    """

    def __init__(self, conn_str: str, altitude: float = 5.0, speed: float = 2.0):
        self.altitude = altitude
        self.speed    = speed

        log.info(f"Connecting to target SITL: {conn_str}")
        self.mav = mavutil.mavlink_connection(conn_str)
        log.info("Waiting for heartbeat...")
        self.mav.wait_heartbeat(timeout=10)
        log.info(f"Connected: sys={self.mav.target_system} comp={self.mav.target_component}")

        self._arm_and_takeoff()

    def _arm_and_takeoff(self):
        """Arm target drone and takeoff to target altitude."""
        log.info("Setting GUIDED mode...")
        mode_id = self.mav.mode_mapping().get("GUIDED")
        self.mav.set_mode(mode_id)
        time.sleep(1)

        log.info("Arming (force)...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 21196, 0, 0, 0, 0, 0
        )
        time.sleep(2)

        log.info(f"Taking off to {self.altitude}m...")
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, self.altitude
        )
        time.sleep(5)
        log.info("Takeoff complete — starting pattern")

    def _send_velocity(self, vx: float, vy: float, vz: float = 0.0):
        """Send NED velocity command in body frame."""
        self.mav.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            VEL_MASK,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0,
        )

    def hover(self, duration: float = 1.0):
        """Hover in place."""
        t0 = time.monotonic()
        while time.monotonic() - t0 < duration:
            self._send_velocity(0, 0, 0)
            time.sleep(0.05)

    # ── Flight Patterns ────────────────────────────────────────────────────────

    def fly_circle(self, radius: float = 8.0, duration: float = 40.0):
        """
        Fly a horizontal circle.
        Angular velocity ω = speed / radius.
        vx = speed * cos(angle), vy = speed * sin(angle) in NED frame.
        """
        log.info(f"Pattern: CIRCLE r={radius}m speed={self.speed}m/s")
        omega = self.speed / radius   # rad/s
        t0    = time.monotonic()
        angle = 0.0
        dt    = 0.05

        while time.monotonic() - t0 < duration:
            vx =  self.speed * math.cos(angle)
            vy =  self.speed * math.sin(angle)
            self._send_velocity(vx, vy)
            angle += omega * dt
            time.sleep(dt)

    def fly_figure8(self, size: float = 6.0, duration: float = 60.0):
        """
        Fly a figure-8 (lemniscate of Bernoulli).
        Parametric: x = A*sin(t), y = A*sin(t)*cos(t)
        Velocity = derivative of position.
        """
        log.info(f"Pattern: FIGURE-8 size={size}m")
        omega = self.speed / size
        t0    = time.monotonic()
        t     = 0.0
        dt    = 0.05

        while time.monotonic() - t0 < duration:
            # Lemniscate parametric velocities
            vx = size * omega * math.cos(omega * t)
            vy = size * omega * (math.cos(omega * t)**2 - math.sin(omega * t)**2)
            # Normalize to speed
            mag = math.sqrt(vx**2 + vy**2)
            if mag > 0.1:
                vx = (vx / mag) * self.speed
                vy = (vy / mag) * self.speed
            self._send_velocity(vx, vy)
            t  += dt
            time.sleep(dt)

    def fly_zigzag(self, leg: float = 10.0, legs: int = 8):
        """Fly a zigzag pattern back and forth."""
        log.info(f"Pattern: ZIGZAG leg={leg}m")
        direction = 1
        for _ in range(legs):
            t0 = time.monotonic()
            duration = leg / self.speed
            vx = self.speed * direction
            vy = self.speed * 0.3 * direction
            while time.monotonic() - t0 < duration:
                self._send_velocity(vx, vy)
                time.sleep(0.05)
            direction *= -1
            self.hover(1.0)

    def fly_random(self, duration: float = 60.0):
        """
        Random wandering: pick a random direction every 3-6 seconds.
        Simulates an uncooperative target.
        """
        log.info("Pattern: RANDOM WANDER")
        t0    = time.monotonic()
        t_dir = time.monotonic()
        vx, vy = self.speed, 0.0

        while time.monotonic() - t0 < duration:
            now = time.monotonic()
            if now - t_dir > random.uniform(2.0, 5.0):
                angle = random.uniform(0, 2 * math.pi)
                vx    = self.speed * math.cos(angle)
                vy    = self.speed * math.sin(angle)
                t_dir = now
                log.debug(f"New heading: vx={vx:.1f} vy={vy:.1f}")
            self._send_velocity(vx, vy)
            time.sleep(0.05)

    def run_pattern(self, pattern: str):
        """Run the selected pattern in a loop."""
        log.info(f"Starting pattern: {pattern.upper()}")
        try:
            while True:
                if pattern == "circle":
                    self.fly_circle(radius=8, duration=60)
                elif pattern == "figure8":
                    self.fly_figure8(size=6, duration=80)
                elif pattern == "zigzag":
                    self.fly_zigzag(leg=12, legs=10)
                elif pattern == "random":
                    self.fly_random(duration=60)
                else:
                    log.error(f"Unknown pattern: {pattern}")
                    break

                # Brief hover between pattern repetitions
                self.hover(2.0)

        except KeyboardInterrupt:
            log.info("Interrupted — hovering")
            self.hover(2.0)


def main():
    parser = argparse.ArgumentParser(description="Target drone autonomous mover")
    parser.add_argument("--conn",     default="udp:127.0.0.1:14560")
    parser.add_argument("--pattern",  default="circle",
                        choices=["circle", "figure8", "zigzag", "random"])
    parser.add_argument("--speed",    type=float, default=2.0)
    parser.add_argument("--altitude", type=float, default=5.0)
    args = parser.parse_args()

    mover = TargetDroneMover(
        conn_str  = args.conn,
        altitude  = args.altitude,
        speed     = args.speed,
    )
    mover.run_pattern(args.pattern)


if __name__ == "__main__":
    main()
