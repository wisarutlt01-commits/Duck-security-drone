#!/usr/bin/env bash
# Installs drone-tracker as a systemd service on the Raspberry Pi 5.
# Run this ON THE PI, from inside the drone_tracker/ project directory.
set -euo pipefail

INSTALL_DIR="/opt/drone-tracker"
SERVICE_USER="${SUDO_USER:-pi}"

if [[ $EUID -ne 0 ]]; then
    echo "Run with sudo: sudo ./systemd/install.sh" >&2
    exit 1
fi

echo "==> Copying project to ${INSTALL_DIR}"
mkdir -p "${INSTALL_DIR}"
rsync -a --exclude venv --exclude .git --exclude logs ./ "${INSTALL_DIR}/"
mkdir -p "${INSTALL_DIR}/logs" "${INSTALL_DIR}/models"
chown -R "${SERVICE_USER}:${SERVICE_USER}" "${INSTALL_DIR}"

echo "==> Adding ${SERVICE_USER} to dialout/video groups (for serial + camera access)"
usermod -aG dialout,video "${SERVICE_USER}"

echo "==> Creating venv and installing dependencies"
sudo -u "${SERVICE_USER}" python3 -m venv "${INSTALL_DIR}/venv"
sudo -u "${SERVICE_USER}" "${INSTALL_DIR}/venv/bin/pip" install --upgrade pip
sudo -u "${SERVICE_USER}" "${INSTALL_DIR}/venv/bin/pip" install \
    -r "${INSTALL_DIR}/requirements.txt" \
    --extra-index-url https://www.piwheels.org/simple

echo "==> Installing systemd unit(s)"
sed "s/^User=pi/User=${SERVICE_USER}/; s/^Group=pi/Group=${SERVICE_USER}/" \
    systemd/drone-tracker.service > /etc/systemd/system/drone-tracker.service
sed "s/^User=pi/User=${SERVICE_USER}/; s/^Group=pi/Group=${SERVICE_USER}/" \
    systemd/drone-tracker-gazebo.service > /etc/systemd/system/drone-tracker-gazebo.service

mkdir -p /etc/drone-tracker
if [[ ! -f /etc/drone-tracker/gazebo.env ]]; then
    cp systemd/gazebo.env.example /etc/drone-tracker/gazebo.env
    echo "    Edit /etc/drone-tracker/gazebo.env to set DESKTOP_IP before using gazebo mode."
fi

systemctl daemon-reload

echo "==> Done."
echo ""
echo "Place your model at ${INSTALL_DIR}/models/drone_yolo.pt, then:"
echo "  sudo systemctl enable --now drone-tracker         # hardware mode, autostart on boot"
echo "  sudo systemctl status drone-tracker"
echo "  journalctl -u drone-tracker -f"
echo ""
echo "For gazebo mode instead:"
echo "  sudo systemctl enable --now drone-tracker-gazebo"
