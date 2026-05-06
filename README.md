# Gazebo Latency Test

Teleoperate a simulated robot with adjustable artificial latency to demonstrate the effects of network delay on robot control.

## What it does

- TurtleBot3 simulation in Gazebo
- WASD keyboard control (pygame)
- Video stream viewable in any browser (including VR headsets)
- Adjustable artificial latency: 0ms to 10s

## Quick Start (Docker)

```bash
docker-compose up
```

Then:
1. Open http://localhost:8888 in browser (or VR headset browser)
2. Click on the pygame window to control robot with WASD
3. Adjust latency with slider or buttons

## Manual Setup (Ubuntu 22.04 + ROS 2 Humble)

### Install dependencies

```bash
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-teleop
pip install pygame opencv-python numpy
```

### Run

Terminal 1 - Gazebo:
```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export ROS_LOCALHOST_ONLY=1
export LIBGL_ALWAYS_SOFTWARE=1  # if GPU issues
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2 - Teleop:
```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
python3 game_teleop.py
```

Terminal 3 - Video server:
```bash
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
python3 video_server.py --port 8888
```

Open http://<YOUR_IP>:8888 in browser.

## Files

- `game_teleop.py` - Pygame WASD controller
- `video_server.py` - MJPEG video server with latency injection
- `docker-compose.yml` - One-command setup
- `Dockerfile` - Container build

## Latency API

- `GET /` - Web UI with video and controls
- `GET /video` - MJPEG stream
- `GET /latency/<ms>` - Set artificial latency (e.g., `/latency/500`)

## License

MIT
