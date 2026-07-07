#!/usr/bin/env bash
# Removes the drone-tracker docker deployment (container, image, compose
# resources) after switching to the systemd service. Run ON THE PI, from
# inside the drone_tracker/ project directory. Does NOT uninstall Docker
# itself, since other containers may still use it.
set -euo pipefail

echo "==> Stopping and removing compose-managed containers/networks"
docker compose down --remove-orphans || true

echo "==> Removing drone-tracker image(s)"
docker rmi drone-tracker:latest 2>/dev/null || true
docker rmi drone-tracker:test 2>/dev/null || true

echo "==> Pruning dangling build cache"
docker builder prune -f

echo "==> Remaining drone-tracker docker resources (should be empty):"
docker ps -a --filter "name=drone-tracker"
docker images | grep drone-tracker || echo "(none)"
