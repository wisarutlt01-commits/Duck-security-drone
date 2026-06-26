#!/usr/bin/env bash
# =============================================================================
# gazebo/launch/start_sim.sh
# =======================
# One-shot launcher for the complete Gazebo Classic 11 simulation stack.
#
# Starts (in order, each in its own tmux window):
#   1. Gazebo Classic 11 with drone_tracking.world
#   2. ArduPilot SITL — Tracker drone (instance 0, UDP 14550)
#   3. ArduPilot SITL — Target drone  (instance 1, UDP 14560)
#   4. drone_tracker/main.py  (Gazebo mode — pygazebo backend)
#   5. target_mover.py        (flies target drone in a pattern)
#
# Camera frames are read from Gazebo Classic's internal transport via
# the pygazebo Python library — no ROS2 or ros_gz_bridge required.
#
# Usage:
#   chmod +x gazebo/launch/start_sim.sh
#   ./gazebo/launch/start_sim.sh [--pattern circle|figure8|zigzag|random]
#
# Requirements:
#   - Gazebo Classic 11 installed (apt install gazebo11)
#   - ArduPilot SITL compiled (ardupilot/build/sitl/bin/arducopter)
#   - ardupilot_gazebo plugin compiled and installed
#   - pip install pygazebo  (Gazebo Classic Python transport bindings)
#   - tmux installed (apt install tmux)
#
# For Gazebo Harmonic (recommended), use fix_and_start.sh instead.
# =============================================================================

set -e

# ── Config ────────────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SIM_DIR="$REPO_ROOT/gazebo"
WORLDS_DIR="$SIM_DIR/worlds"
MODELS_DIR="$SIM_DIR/models"

ARDUPILOT_SITL="${ARDUPILOT_SITL:-$HOME/ardupilot/build/sitl/bin/arducopter}"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"

PATTERN="${1:-circle}"
while [[ $# -gt 0 ]]; do
  case $1 in
    --pattern) PATTERN="$2"; shift 2;;
    *) shift;;
  esac
done

MODEL="models/drone_yolo.pt"
# Gazebo Classic internal camera topic (pygazebo backend)
CLASSIC_CAM_TOPIC="/gazebo/default/tracker_drone/base_link/front_camera/image"

# ── Colors ────────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

log()  { echo -e "${GREEN}[start_sim]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err()  { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

# ── Pre-flight checks ─────────────────────────────────────────────────────────
log "Checking dependencies..."

command -v tmux    >/dev/null 2>&1 || err "tmux not found: sudo apt install tmux"
command -v gazebo  >/dev/null 2>&1 || err "Gazebo not found — install gazebo11"
[ -f "$ARDUPILOT_SITL" ]           || err "ArduPilot SITL not found at: $ARDUPILOT_SITL"

# Check pygazebo is available (needed for Classic 11 camera transport)
python3 -c "import pygazebo" 2>/dev/null || \
  warn "pygazebo not installed — camera will not work. Run: pip install pygazebo"

# ── Add Gazebo model path ─────────────────────────────────────────────────────
export GAZEBO_MODEL_PATH="$MODELS_DIR:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="$WORLDS_DIR:${GAZEBO_RESOURCE_PATH:-}"
log "GAZEBO_MODEL_PATH set to include: $MODELS_DIR"

# ── Create tmux session ───────────────────────────────────────────────────────
SESSION="drone_sim"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -x 250 -y 50

log "Starting simulation in tmux session: $SESSION"
log "Attach with: tmux attach -t $SESSION"
echo ""

ACTIVATE=""
[ -f "$REPO_ROOT/venv/bin/activate" ] && ACTIVATE="source $REPO_ROOT/venv/bin/activate &&"

# ── 1. Launch Gazebo Classic 11 ───────────────────────────────────────────────
log "[1/5] Launching Gazebo Classic 11..."
tmux rename-window -t "$SESSION:0" "gazebo"
tmux send-keys -t "$SESSION:0" \
  "export GAZEBO_MODEL_PATH=$MODELS_DIR:\$GAZEBO_MODEL_PATH && \
   gazebo --verbose $WORLDS_DIR/drone_tracking.world" Enter
sleep 6

# ── 2. ArduPilot SITL — Tracker drone (instance 0) ───────────────────────────
log "[2/5] Starting SITL — Tracker drone (instance 0)..."
tmux new-window -t "$SESSION" -n "sitl-tracker"
tmux send-keys -t "$SESSION:sitl-tracker" \
  "cd /tmp && $ARDUPILOT_SITL \
   --model gazebo-iris \
   --home -35.363261,149.165230,584,353 \
   --instance 0 \
   --speedup 1 \
   --sysid 1" Enter
sleep 4

# ── 3. ArduPilot SITL — Target drone (instance 1) ────────────────────────────
log "[3/5] Starting SITL — Target drone (instance 1)..."
tmux new-window -t "$SESSION" -n "sitl-target"
tmux send-keys -t "$SESSION:sitl-target" \
  "cd /tmp && $ARDUPILOT_SITL \
   --model gazebo-iris \
   --home -35.363261,149.165230,584,353 \
   --instance 1 \
   --speedup 1 \
   --sysid 2" Enter
sleep 4

# ── 4. drone_tracker main.py (pygazebo backend for Gazebo Classic) ────────────
log "[4/5] Starting drone_tracker (Gazebo Classic mode)..."
tmux new-window -t "$SESSION" -n "tracker"
tmux send-keys -t "$SESSION:tracker" \
  "cd $REPO_ROOT && \
   $ACTIVATE python main.py \
   --mode gazebo \
   --model $MODEL \
   --sitl-conn udp:127.0.0.1:14550 \
   --gst-port 0 --classic-camera-topic $CLASSIC_CAM_TOPIC \
   --display" Enter
sleep 3

# ── 5. target_mover.py ───────────────────────────────────────────────────────
log "[5/5] Starting target mover (pattern: $PATTERN)..."
tmux new-window -t "$SESSION" -n "target-mover"
tmux send-keys -t "$SESSION:target-mover" \
  "cd $REPO_ROOT && \
   $ACTIVATE python gazebo/scripts/target_mover.py \
   --conn udp:127.0.0.1:14560 \
   --pattern $PATTERN \
   --speed 2.0 \
   --altitude 5.0" Enter

# ── Done ──────────────────────────────────────────────────────────────────────
echo ""
log "═══════════════════════════════════════════════════════"
log "  Simulation running in tmux session: $SESSION"
log "  Attach:  tmux attach -t $SESSION"
log "  Windows: gazebo | sitl-tracker | sitl-target | tracker | target-mover"
log ""
log "  MAVProxy debug (tracker):  mavproxy.py --master udp:127.0.0.1:14550"
log "  MAVProxy debug (target):   mavproxy.py --master udp:127.0.0.1:14560"
log ""
log "  Stop all:  tmux kill-session -t $SESSION"
log "═══════════════════════════════════════════════════════"
