#!/bin/bash
# Gazebo Latency Test - Run Script
# Tested on Ubuntu 22.04 + ROS 2 Humble

set -e

# ROS environment - CRITICAL: these fix discovery issues
source /opt/ros/humble/setup.bash
unset ROS_DISCOVERY_SERVER
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle_pi
export LIBGL_ALWAYS_SOFTWARE=1  # Software rendering (slower but works)

echo "=== Gazebo Latency Test ==="
echo ""
echo "Starting Gazebo..."
tmux new-session -d -s gazebo "source /opt/ros/humble/setup.bash && unset ROS_DISCOVERY_SERVER && export ROS_LOCALHOST_ONLY=1 && export ROS_DOMAIN_ID=0 && export TURTLEBOT3_MODEL=waffle_pi && export LIBGL_ALWAYS_SOFTWARE=1 && ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"

echo "Waiting for Gazebo to start..."
until ros2 topic list 2>/dev/null | grep -q cmd_vel; do sleep 2; done
echo "Gazebo ready!"

echo ""
echo "Starting video server on port 8888..."
python3 "$(dirname "$0")/video_server.py" --port 8888 &
sleep 2

echo ""
echo "Starting teleop (pygame window)..."
python3 "$(dirname "$0")/game_teleop.py" &

echo ""
echo "=== READY ==="
echo "Video: http://$(hostname -I | awk '{print $1}'):8888/"
echo "Controls: WASD in pygame window"
echo ""
echo "Press Ctrl+C to stop all"

trap "tmux kill-session -t gazebo 2>/dev/null; pkill -f video_server; pkill -f game_teleop" EXIT

wait
