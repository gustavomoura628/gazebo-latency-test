#!/bin/bash
# Gazebo Latency Test - Docker Launcher
# One command to run everything

set -e

echo "=== Gazebo Latency Test (Docker) ==="
echo ""

# Cleanup any existing instances
echo "Cleaning up previous instances..."
docker compose down 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
fuser -k 8888/tcp 2>/dev/null || true
fuser -k 11345/tcp 2>/dev/null || true
sleep 1

# Allow X11 connections from Docker
echo "Setting up X11 permissions..."
xhost +local:docker 2>/dev/null || true

# Check if already built
if ! docker images | grep -q gazebo-latency-test; then
    echo "Building Docker image (first run, may take a few minutes)..."
    docker compose build
fi

echo ""
echo "Starting containers..."
echo "Video will be at: http://$(hostname -I | awk '{print $1}'):8888/"
echo "Controls: WASD in pygame window"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run with docker compose
docker compose up --build

# Cleanup X11 permissions on exit
xhost -local:docker 2>/dev/null || true
