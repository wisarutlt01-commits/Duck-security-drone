#!/usr/bin/env bash
# Installs the systemd unit(s) for drone-tracker, pointing at an existing
# venv in this project directory. Does NOT create a venv or install deps —
# do that yourself first (python3 -m venv venv && venv/bin/pip install -r
# requirements.txt) if you haven't already.
# Run this ON THE PI, from inside the drone_tracker/ project directory.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_USER="${SUDO_USER:-pi}"
VENV_DIR="${VENV_DIR:-${PROJECT_DIR}/venv}"

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo ./systemd/install.sh" >&2
    exit 1
fi

if [[ ! -x "${VENV_DIR}/bin/python3" ]]; then
    echo "No venv found at ${VENV_DIR}." >&2
    echo "Set VENV_DIR=/path/to/venv sudo -E ./systemd/install.sh if it's elsewhere," >&2
    echo "or create one first: python3 -m venv venv && venv/bin/pip install -r requirements.txt" >&2
    exit 1
fi

echo "==> Adding ${SERVICE_USER} to dialout/video groups (for serial + camera access)"
usermod -aG dialout,video "${SERVICE_USER}"

mkdir -p "${PROJECT_DIR}/logs" "${PROJECT_DIR}/models"
chown -R "${SERVICE_USER}:${SERVICE_USER}" "${PROJECT_DIR}/logs" "${PROJECT_DIR}/models"

echo "==> Installing systemd unit(s), pointed at ${PROJECT_DIR} (venv: ${VENV_DIR})"
sed \
    -e "s/^User=pi/User=${SERVICE_USER}/" \
    -e "s/^Group=pi/Group=${SERVICE_USER}/" \
    -e "s|WorkingDirectory=.*|WorkingDirectory=${PROJECT_DIR}|" \
    -e "s|/opt/drone-tracker/venv|${VENV_DIR}|" \
    "${PROJECT_DIR}/systemd/drone-tracker.service" > /etc/systemd/system/drone-tracker.service
sed \
    -e "s/^User=pi/User=${SERVICE_USER}/" \
    -e "s/^Group=pi/Group=${SERVICE_USER}/" \
    -e "s|WorkingDirectory=.*|WorkingDirectory=${PROJECT_DIR}|" \
    -e "s|/opt/drone-tracker/venv|${VENV_DIR}|" \
    "${PROJECT_DIR}/systemd/drone-tracker-gazebo.service" > /etc/systemd/system/drone-tracker-gazebo.service

mkdir -p /etc/drone-tracker
if [[ ! -f /etc/drone-tracker/gazebo.env ]]; then
    cp "${PROJECT_DIR}/systemd/gazebo.env.example" /etc/drone-tracker/gazebo.env
    echo "    Edit /etc/drone-tracker/gazebo.env to set DESKTOP_IP before using gazebo mode."
fi

systemctl daemon-reload

echo "==> Done."
echo ""
echo "  sudo systemctl enable --now drone-tracker         # hardware mode, autostart on boot"
echo "  sudo systemctl status drone-tracker"
echo "  journalctl -u drone-tracker -f"
echo ""
echo "For gazebo mode instead:"
echo "  sudo systemctl enable --now drone-tracker-gazebo"
