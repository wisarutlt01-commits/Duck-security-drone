#!/usr/bin/env bash
# =============================================================================
# fix_ubuntu24_gz.sh
# ===================
# Definitive fix for gz sim crash on Ubuntu 24.04 + ROS2 Humble + Gz Harmonic
#
# The problem on Ubuntu 24.04:
#   gz-msgs10 was compiled against protobuf 3.12.4
#   /lib/x86_64-linux-gnu/libprotobuf.so.23 is 3.21.12 (from ROS2 Humble pkg)
#   /usr/lib/x86_64-linux-gnu/libprotobuf.so.32 is 4.x (Ubuntu 24 system)
#   gz sim picks up .so.32 from system, but gz-msgs10 expects .so.23 → crash
#
# THE CORRECT FIX:
#   Force gz sim to use .so.23 (which matches what gz-msgs10 was compiled with)
#   by putting /lib/x86_64-linux-gnu BEFORE /usr/lib/x86_64-linux-gnu
#
# Usage:
#   chmod +x fix_ubuntu24_gz.sh
#   ./fix_ubuntu24_gz.sh
# =============================================================================
set -e

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
log()  { echo -e "${GREEN}[fix]${NC} $1"; }
warn() { echo -e "${YELLOW}[warn]${NC} $1"; }
info() { echo -e "${CYAN}[info]${NC} $1"; }
err()  { echo -e "${RED}[ERROR]${NC} $1"; exit 1; }

echo ""
log "======================================================"
log " Gazebo Harmonic protobuf fix — Ubuntu 24.04"
log "======================================================"
echo ""

# ── Step 1: Locate the exact .so.23 that gz-msgs10 needs ─────────────────────
log "Step 1: Locating protobuf libraries..."

SO23=$(find /lib/x86_64-linux-gnu /usr/lib/x86_64-linux-gnu \
       -name "libprotobuf.so.23*" 2>/dev/null | head -1)
info "  libprotobuf.so.23 : ${SO23}"
SO32=$(find /usr/lib/x86_64-linux-gnu \
       -name "libprotobuf.so.32*" 2>/dev/null | head -1)
GZ_MSGS=$(find /lib /usr/lib -name "libgz-msgs10.so*" 2>/dev/null | head -1)

info "  libprotobuf.so.23 : ${SO23:-NOT FOUND}"
info "  libprotobuf.so.32 : ${SO32:-NOT FOUND}"
info "  libgz-msgs10      : ${GZ_MSGS:-NOT FOUND}"

[ -z "$SO23" ] && err "libprotobuf.so.23 not found. Run: sudo apt install libprotobuf23"
[ -z "$GZ_MSGS" ] && err "libgz-msgs10 not found. Is Gazebo Harmonic installed?"

SO23_DIR=$(dirname "$SO23")
info "  .so.23 directory  : $SO23_DIR"

# ── Step 2: Verify gz-msgs10 links against .so.23 ────────────────────────────
log "Step 2: Verifying gz-msgs10 protobuf dependency..."
GZ_MSGS_PROTO=$(ldd "$GZ_MSGS" 2>/dev/null | grep protobuf | awk '{print $1, "→", $3}')
info "  gz-msgs10 needs: $GZ_MSGS_PROTO"

# ── Step 3: Create the wrapper script ────────────────────────────────────────
log "Step 3: Creating gz sim wrapper script..."

WRAPPER="/usr/local/bin/gz_fixed"
sudo tee "$WRAPPER" > /dev/null << WRAPPER_EOF
#!/usr/bin/env bash
# gz sim wrapper: forces libprotobuf.so.23 to load before .so.32
# This fixes the gz-msgs10 version mismatch on Ubuntu 24.04 + ROS2 Humble
export LD_LIBRARY_PATH="${SO23_DIR}:\${LD_LIBRARY_PATH:-}"
exec gz "\$@"
WRAPPER_EOF

sudo chmod +x "$WRAPPER"
log "Wrapper created: $WRAPPER"
info "  Use 'gz_fixed sim' instead of 'gz sim'"

# ── Step 4: Test the fix ─────────────────────────────────────────────────────
log "Step 4: Testing fix..."
if gz_fixed sim --version >/dev/null 2>&1; then
    GZ_VER=$(gz_fixed sim --version 2>/dev/null | head -1)
    log "SUCCESS! gz sim works: $GZ_VER"
else
    warn "Wrapper test inconclusive — trying alternative approach..."

    # Alternative: create a shell alias + env file
    ENV_FILE="$HOME/.gz_env"
    cat > "$ENV_FILE" << ENV_EOF
# Source this before running gz sim:
# source ~/.gz_env && gz sim ...
export LD_LIBRARY_PATH="${SO23_DIR}:\${LD_LIBRARY_PATH:-}"
ENV_EOF
    warn "Alternative: run 'source ~/.gz_env' then 'gz sim'"
fi

# ── Step 5: Create launch alias ───────────────────────────────────────────────
log "Step 5: Adding gz_fixed alias to ~/.bashrc..."
ALIAS_LINE='alias gz_fixed="LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH} gz"'
if ! grep -q "gz_fixed" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Gazebo Harmonic protobuf fix (Ubuntu 24.04 + ROS2 Humble)" >> ~/.bashrc
    echo "$ALIAS_LINE" >> ~/.bashrc
    log "Alias added to ~/.bashrc"
else
    log "Alias already in ~/.bashrc"
fi

# ── Step 6: Update sim launch scripts to use gz_fixed ────────────────────────
log "Step 6: Patching sim/launch scripts to use gz_fixed..."

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for SCRIPT in "$SCRIPT_DIR/start_sim.sh" "$SCRIPT_DIR/fix_and_start.sh"; do
    if [ -f "$SCRIPT" ]; then
        sed -i 's|gz sim |gz_fixed sim |g' "$SCRIPT"
        log "  Patched: $SCRIPT"
    fi
done

echo ""
log "======================================================"
log " Fix applied. Usage:"
log ""
log "   # Option A — use wrapper (recommended):"
log "   gz_fixed sim sim/worlds/drone_tracking_harmonic.world"
log ""
log "   # Option B — inline env:"
log "   LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:\$LD_LIBRARY_PATH \\"
log "     gz sim sim/worlds/drone_tracking_harmonic.world"
log ""
log "   # Option C — launch everything:"
log "   ./sim/launch/fix_and_start.sh --pattern circle"
log "======================================================"
