# Gazebo Testing Guide

Setup and usage for the **networked** Gazebo test topology:

```
[ Desktop PC ]                         [ Raspberry Pi 5 ]
  Gazebo Harmonic / Classic 11           drone_tracker (main.py)
  ArduPilot SITL (tracker drone)     ←──UDP 5600  ── H.264/RTP camera stream
  ArduPilot SITL (target drone)      ←──UDP 14550 ── MAVLink GCS
  target_mover.py                    ──►UDP 14560 ── MAVLink target SITL
```

The camera stream travels **desktop → Pi** over the LAN.
MAVLink travels bidirectionally over UDP.
No ROS2 required.

---

## Quick-start (same machine, for local testing)

```bash
cd /path/to/drone_tracker
./gazebo/launch/start_sim.sh --pattern circle        # Gazebo Classic 11
# or:
./gazebo/launch/fix_and_start.sh --pattern circle    # Gazebo Harmonic (Ubuntu 24.04 protobuf fix)
```

Each script opens a `tmux` session named `drone_sim`.  Attach with:
```bash
tmux attach -t drone_sim
```
Navigate panes: `Ctrl+B` then window number.

---

## Networked setup (Pi 5 ↔ Desktop)

### Step 1 — Find your IPs

```bash
# On the desktop:
hostname -I | awk '{print $1}'   # DESKTOP_IP, e.g. 192.168.1.10

# On the Pi:
hostname -I | awk '{print $1}'   # PI_IP, e.g. 192.168.1.20
```

### Step 2 — Edit the Gazebo world to point the camera at the Pi

Open `gazebo/worlds/drone_tracking_harmonic.world` and find the
`GstCameraPlugin` section in the tracker drone's `model.sdf`:

```xml
<udpHost>192.168.1.20</udpHost>   <!-- PI_IP -->
<udpPort>5600</udpPort>
```

Replace `192.168.1.20` with your Pi's actual LAN IP.

### Step 3 — Start Gazebo + SITL on the desktop

```bash
# Terminal 1 — Gazebo (Classic 11)
export GAZEBO_MODEL_PATH=$(pwd)/gazebo/models:$GAZEBO_MODEL_PATH
gazebo --verbose gazebo/worlds/drone_tracking.world

# Or for Harmonic with the protobuf fix:
LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH \
  gz sim gazebo/worlds/drone_tracking_harmonic.world -v 3

# Terminal 2 — Tracker SITL (exposes MAVLink to Pi on UDP 14550)
mkdir -p /tmp/sitl0 && cd /tmp/sitl0
~/ardupilot/build/sitl/bin/arducopter \
  --model gazebo-iris \
  --home -35.363261,149.165230,584,353 \
  --instance 0 --speedup 1 --sysid 1 \
  --out=udp:192.168.1.20:14550         # PI_IP

# Terminal 3 — Target SITL
mkdir -p /tmp/sitl1 && cd /tmp/sitl1
~/ardupilot/build/sitl/bin/arducopter \
  --model gazebo-iris \
  --home -35.363261,149.165230,584,353 \
  --instance 1 --speedup 1 --sysid 2

# Terminal 4 — target_mover (flies the target in a pattern)
python gazebo/scripts/target_mover.py \
  --conn udp:127.0.0.1:14560 --pattern circle --speed 2.0
```

### Step 4 — Start the tracker on the Pi

```bash
python main.py --mode gazebo \
    --model models/drone_yolo.pt \
    --sitl-conn udp:192.168.1.10:14550 \   # DESKTOP_IP
    --gst-host  0.0.0.0 \
    --gst-port  5600
```

---

## Networking sanity checks

```bash
# Verify camera stream reaches the Pi (run on Pi):
nc -u -l 5600 | head -c 100 | xxd    # should see RTP bytes

# Verify MAVLink reaches the Pi:
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udp:0.0.0.0:14550')
hb = m.wait_heartbeat(timeout=10)
print('Heartbeat:', hb)
"

# Verify firewall allows UDP traffic:
# On desktop: sudo ufw allow 14550/udp; sudo ufw allow 5600/udp
# On Pi:      sudo ufw allow 5600/udp
```

---

## Prerequisites

### Desktop (Gazebo machine)

```bash
# Gazebo Classic 11
sudo apt install gazebo libgazebo11-dev

# ArduPilot SITL
git clone https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y && . ~/.profile
./waf configure --board sitl && ./waf copter

# ardupilot_gazebo plugin
git clone https://github.com/khancyr/ardupilot_gazebo.git ~/ardupilot_gazebo
cd ~/ardupilot_gazebo && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && sudo make install
```

### Pi 5 (tracker machine)

```bash
pip install -r requirements.txt

# OpenCV with GStreamer support (preferred camera backend):
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
# Then build or install a GStreamer-enabled OpenCV wheel

# ffmpeg fallback (if OpenCV lacks GStreamer):
sudo apt install ffmpeg
```

---

## SITL UDP Port Reference

| Instance  | Gazebo FDM  | MAVLink GCS | MAVLink API |
|-----------|-------------|-------------|-------------|
| 0 tracker | 9002/9003   | **14550**   | 14551       |
| 1 target  | 9012/9013   | **14560**   | 14561       |

---

## Protobuf fix for Gazebo Harmonic on Ubuntu 22.04

ROS2 Humble ships `libprotobuf.so.23` while Harmonic needs `.so.32`.
Running `gz sim` after `source /opt/ros/humble/setup.bash` causes a crash.

**Quick fix:**
```bash
LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH \
  gz sim gazebo/worlds/drone_tracking_harmonic.world -v 3
```

Add an alias to `~/.bashrc`:
```bash
alias gz_fixed="LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:\${LD_LIBRARY_PATH} gz"
```

**Version matrix:**

| Ubuntu | ROS2   | Gazebo      | Status                  |
|--------|--------|-------------|-------------------------|
| 22.04  | Humble | Classic 11  | No conflict             |
| 22.04  | Humble | Harmonic    | Needs LD_LIBRARY_PATH fix |
| 24.04  | Jazzy  | Harmonic    | No conflict (both use .so.32) |

`gazebo/launch/fix_and_start.sh` applies the fix automatically for Ubuntu 22.04.

---

## Target drone flight patterns

```bash
python gazebo/scripts/target_mover.py --pattern circle   --speed 1.5
python gazebo/scripts/target_mover.py --pattern figure8  --speed 2.0
python gazebo/scripts/target_mover.py --pattern zigzag   --speed 3.0
python gazebo/scripts/target_mover.py --pattern random   --speed 2.5
```

---

## YOLO model for simulation

The real `.pt` model may not detect the Gazebo drone well due to domain gap.

**Option 1 — Try your existing model** with `--display` and see if it detects
at all.  Lowering `--conf 0.1` helps.

**Option 2 — Generate sim training data**:
```bash
# Record frames from the UDP stream while Gazebo runs
ffmpeg -protocol_whitelist file,udp,rtp \
  -i <(cat gazebo/scripts/stream.sdp) \
  -vframes 1000 frames/frame%04d.jpg
# Label with LabelImg / Roboflow, then retrain:
yolo train data=sim_dataset.yaml model=yolov8s.pt epochs=50 imgsz=640
```
