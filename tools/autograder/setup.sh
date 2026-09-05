#!/usr/bin/env bash

# Exit immediately if a command exits with a non-zero status
set -e

# Prevent interactive prompts (e.g. tzdata timezone configuration)
export DEBIAN_FRONTEND=noninteractive
export TZ=Etc/UTC

echo "=== Gradescope Autograder Setup Script ==="

# Update package lists
apt-get update

# Install system dependencies
apt-get install -y --no-install-recommends \
    tzdata \
    python3 \
    python3-pip \
    python3-dev \
    gcc \
    clang \
    make \
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libsdl2-ttf-dev \
    ffmpeg

# Install Python packages (fallback if break-system-packages is needed)
pip3 install --upgrade pip || echo "pip upgrade failed, continuing"
pip3 install --break-system-packages numpy pygame opencv-python-headless || \
pip3 install numpy pygame opencv-python-headless

echo "=== Setup Completed Successfully ==="
