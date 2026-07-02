# Drone Tracker — Raspberry Pi 5 (arm64) image
#
# Build (on the Pi 5, or with buildx for cross-compiling from another arch):
#   docker build -t drone-tracker:latest .
#
# CPU-only inference (no AI HAT support baked in yet — tracker/config.py's
# `device` field stays configurable for when one is added).

FROM python:3.11-slim

# OpenCV runtime libs + ffmpeg (fallback UDP backend for gazebo mode, since
# the pip opencv-python wheel ships without GStreamer support).
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender1 \
        ffmpeg \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY main.py .
COPY tracker/ tracker/
COPY tools/ tools/

RUN mkdir -p logs models

ENTRYPOINT ["python3", "main.py"]
CMD ["--mode", "hardware"]
