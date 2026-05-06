#!/bin/bash
# Docker entrypoint - loads robot from world file (avoids spawn service issues)

set -e

echo "=== Gazebo Latency Test (Docker) ==="
echo ""

# Setup ROS environment
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
export TURTLEBOT3_MODEL=waffle_pi
export LIBGL_ALWAYS_SOFTWARE=1
# Our models + turtlebot3 models
export GAZEBO_MODEL_PATH=/app/models:/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH

# Check display
if [ -z "$DISPLAY" ]; then
    echo "No DISPLAY set - starting Xvfb..."
    Xvfb :99 -screen 0 1024x768x24 &
    export DISPLAY=:99
    sleep 2
fi
echo "Using DISPLAY=$DISPLAY"

# Start gzserver with robot included in world file
echo ""
echo "Starting Gazebo server..."
gzserver /app/worlds/turtlebot3_world.world \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so \
    -s libgazebo_ros_force_system.so &

# Wait for gzserver to start
echo "Waiting for Gazebo server..."
TIMEOUT=120
ELAPSED=0
until ros2 topic list 2>/dev/null | grep -q clock; do
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "ERROR: Gazebo server failed to start"
        exit 1
    fi
    echo "  Waiting... (${ELAPSED}s)"
done
echo "Gazebo server started!"

# Start gzclient (GUI)
echo "Starting Gazebo GUI..."
gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so &

# Start robot state publisher
echo "Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args -p robot_description:="$(cat /opt/ros/humble/share/turtlebot3_gazebo/urdf/turtlebot3_waffle_pi.urdf)" &

# Wait for robot topics (diff_drive controller creates cmd_vel)
echo "Waiting for robot to be ready..."
TIMEOUT=60
ELAPSED=0
until ros2 topic list 2>/dev/null | grep -q cmd_vel; do
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "Note: /cmd_vel not found, robot plugins may not have loaded"
        echo "Available topics:"
        ros2 topic list 2>/dev/null
        break
    fi
    echo "  Waiting for /cmd_vel... (${ELAPSED}s)"
done

# Start video server
echo ""
echo "Starting video server on port 8888..."
python3 /app/video_server.py --port 8888 &
sleep 2

# Start teleop
echo "Starting teleop (pygame window)..."
python3 /app/game_teleop.py &

echo ""
echo "=== READY ==="
echo "Video: http://$(hostname -I | awk '{print $1}'):8888/"
echo "Controls: WASD in pygame window"
echo ""
echo "Press Ctrl+C to stop all"

# Wait for processes
wait
