#!/usr/bin/env bash
# =============================================================================
# fix_and_start.sh — Ubuntu 24.04 + Gazebo Harmonic (no ROS2 required)
# =============================================================================
# Uses LD_LIBRARY_PATH fix: puts /lib/x86_64-linux-gnu first so gz sim
# finds libprotobuf.so.23 (which gz-msgs was compiled against) instead of
# the Ubuntu 24 system libprotobuf.so.32.
#
# Camera frames are received via gz-transport Python bindings (gz-python):
#   sudo apt install python3-gz-transport13 python3-gz-msgs11
# No ros_gz_bridge or ROS2 installation needed.
# =============================================================================

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
MODELS_DIR="$REPO_ROOT/gazebo/models"
WORLDS_DIR="$REPO_ROOT/gazebo/worlds"
ARDUPILOT_SITL="${ARDUPILOT_SITL:-$HOME/ardupilot/build/sitl/bin/arducopter}"

PATTERN="circle"
while [[ $# -gt 0 ]]; do
  case $1 in --pattern) PATTERN="$2"; shift 2;; *) shift;; esac
done

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}[start]${NC} $1"; }
warn() { echo -e "${YELLOW}[warn]${NC} $1"; }

# The critical fix: .so.23 lives in /lib/x86_64-linux-gnu on Ubuntu 24
# Prepend it so gz sim and gz-msgs both see the same protobuf version
export GZ_PROTO_PATH="/lib/x86_64-linux-gnu"
export GZ_LD_PREFIX="LD_LIBRARY_PATH=${GZ_PROTO_PATH}:\${LD_LIBRARY_PATH}"

# Verify fix works before launching
log "Testing gz_fixed sim with protobuf fix..."
if ! eval "$GZ_LD_PREFIX gz_fixed sim --version" >/dev/null 2>&1; then
    warn "gz_fixed sim test failed — trying fallback path..."
    GZ_PROTO_PATH="/usr/lib/x86_64-linux-gnu"
    export GZ_LD_PREFIX="LD_LIBRARY_PATH=${GZ_PROTO_PATH}:\${LD_LIBRARY_PATH}"
    if ! eval "$GZ_LD_PREFIX gz_fixed sim --version" >/dev/null 2>&1; then
        echo ""
        echo "gz_fixed sim still failing. Run the diagnostic first:"
        echo "  bash sim/launch/fix_ubuntu24_gz.sh"
        exit 1
    fi
fi
log "gz_fixed sim works with fix applied"

export GAZEBO_MODEL_PATH="$MODELS_DIR:${GAZEBO_MODEL_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$WORLDS_DIR:$MODELS_DIR:${GZ_SIM_RESOURCE_PATH:-}"

command -v tmux >/dev/null 2>&1 || { echo "Install tmux: sudo apt install tmux"; exit 1; }
[ -f "$ARDUPILOT_SITL" ] || { echo "SITL not found at $ARDUPILOT_SITL"; echo "Set: export ARDUPILOT_SITL=/path/to/arducopter"; exit 1; }

SESSION="drone_sim"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -x 220 -y 50

VENV=""
[ -f "$REPO_ROOT/venv/bin/activate" ] && VENV="source $REPO_ROOT/venv/bin/activate && "

# ── 1: gz sim with protobuf fix ──────────────────────────────────────────────
log "[1/5] gz sim..."
tmux rename-window -t "$SESSION:0" "gzsim"
tmux send-keys -t "$SESSION:0" \
  "export GAZEBO_MODEL_PATH=$MODELS_DIR:\$GAZEBO_MODEL_PATH && \
   export GZ_SIM_RESOURCE_PATH=$WORLDS_DIR:$MODELS_DIR && \
   LD_LIBRARY_PATH=${GZ_PROTO_PATH}:\$LD_LIBRARY_PATH \
   gz_fixed sim $WORLDS_DIR/drone_tracking_harmonic.world -v 3" Enter
sleep 8

# ── 2: SITL tracker (instance 0 → UDP 14550) ─────────────────────────────────
log "[2/5] SITL tracker (inst 0)..."
tmux new-window -t "$SESSION" -n "sitl-tracker"
tmux send-keys -t "$SESSION:sitl-tracker" \
  "mkdir -p /tmp/sitl0 && cd /tmp/sitl0 && \
   $ARDUPILOT_SITL --model gazebo-iris \
   --home -35.363261,149.165230,584,353 \
   --instance 0 --speedup 1 --sysid 1 -v ArduCopter" Enter
sleep 5

# ── 3: SITL target (instance 1 → UDP 14560) ──────────────────────────────────
log "[3/5] SITL target (inst 1)..."
tmux new-window -t "$SESSION" -n "sitl-target"
tmux send-keys -t "$SESSION:sitl-target" \
  "mkdir -p /tmp/sitl1 && cd /tmp/sitl1 && \
   $ARDUPILOT_SITL --model gazebo-iris \
   --home -35.363261,149.165230,584,353 \
   --instance 1 --speedup 1 --sysid 2 -v ArduCopter" Enter
sleep 5

# ── 4: drone_tracker main.py (Gazebo Harmonic — gz-transport backend) ─────────
log "[4/5] drone_tracker (Gazebo mode)..."
tmux new-window -t "$SESSION" -n "tracker"
tmux send-keys -t "$SESSION:tracker" \
  "cd $REPO_ROOT && \
   ${VENV}python main.py \
   --mode gazebo \
   --model models/drone_yolo.pt \
   --sitl-conn udp:127.0.0.1:14550 \
   --gst-port 0 --camera-topic /tracker_drone/camera/image_raw \
   --display" Enter

# ── 5: target mover ───────────────────────────────────────────────────────────
log "[5/5] target_mover (pattern: $PATTERN)..."
tmux new-window -t "$SESSION" -n "target"
tmux send-keys -t "$SESSION:target" \
  "cd $REPO_ROOT && ${VENV}python gazebo/scripts/target_mover.py \
   --conn udp:127.0.0.1:14560 --pattern $PATTERN --speed 2.0" Enter

echo ""
log "══════════════════════════════════════════════"
log "  tmux attach -t $SESSION"
log "  Windows: gzsim | sitl-tracker | sitl-target | tracker | target"
log "  Stop: tmux kill-session -t $SESSION"
log "══════════════════════════════════════════════"
