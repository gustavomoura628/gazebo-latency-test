# Gazebo Latency Test

Teleoperate a simulated robot with adjustable artificial latency to demonstrate the effects of network delay on robot control.

## What it does

- TurtleBot3 simulation in Gazebo
- WASD keyboard control (pygame)
- Video stream viewable in any browser (including VR headsets)
- Adjustable artificial latency: 0ms to 10s

## Quick Start (Docker)

**Requirements:** Docker and docker-compose

```bash
git clone https://github.com/gustavomoura628/gazebo-latency-test.git
cd gazebo-latency-test
./docker-run.sh
```

That's it! First run will build the image (~5 min), subsequent runs start instantly.

Then:
1. Open http://<YOUR_IP>:8888 in browser (or Quest 3 browser)
2. Click on the pygame window to control robot with WASD
3. Adjust latency with slider or buttons

Press Ctrl+C to stop everything.

## Native Install (Alternative)

If you prefer running without Docker:

### Requirements

- Ubuntu 22.04
- ROS 2 Humble
- tmux

### Install dependencies

```bash
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-teleop tmux
pip install pygame opencv-python numpy
```

### Run

```bash
./run.sh
```

## Files

- `docker-run.sh` - One-command Docker launcher
- `docker-entrypoint.sh` - Docker container entrypoint script
- `Dockerfile` - Container definition
- `docker-compose.yml` - Container configuration
- `run.sh` - Native launcher (starts Gazebo, teleop, video server)
- `game_teleop.py` - Pygame WASD controller
- `video_server.py` - MJPEG video server with latency injection
- `models/` - Patched TurtleBot3 model with physics fixes
- `worlds/` - Custom Gazebo world with robot included

## HTTP API

- `GET /` - Web UI with video and controls
- `GET /video` - MJPEG stream
- `GET /latency/<ms>` - Set artificial latency (e.g., `/latency/500`)
- `GET /predict/<0|1>` - Toggle predictive display (compensates for video latency)
- `GET /depth/<0|1>` - Toggle per-pixel depth mode (Phase 2b, requires torch + transformers + pillow)
- `GET /hfov/<deg>` - Set camera horizontal FOV in degrees
- `GET /scene_depth/<m>` - Set assumed scene depth (flat-plane proxy / depth-mode calibration)

## Predictive Display

Three modes:

1. **Off (default)** - serves the delayed frame as-is. Demonstrates raw latency effect.
2. **Flat-plane (Phase 2a)** - per-frame homography warp using the operator's command stream
   integrated since the frame was captured. Assumes the scene sits at a single configurable
   depth. No extra dependencies; runs at full MJPEG framerate. Eliminates move-and-wait for
   yaw, gives a reasonable expansion-from-center for forward motion.
3. **Per-pixel depth (Phase 2b)** - Depth Anything V2 estimates per-pixel depth, then
   reprojects with a GPU z-buffer for true parallax. Native run only (Docker image doesn't
   include torch). Install requirements:
   ```bash
   pip install torch transformers pillow
   ```
   First use downloads the Depth Anything V2 Small model (~100 MB). GPU strongly
   recommended (CPU inference is ~2 s/frame; CUDA is ~30 ms/frame on RTX-class GPUs).

## Troubleshooting

### No GUI windows appearing
Make sure X11 forwarding is working:
```bash
xhost +local:docker
```

### Gazebo crashes or shows glitched window
The container uses software rendering by default. If you have GPU issues, this should handle it automatically.

### Port 8888 already in use
```bash
fuser -k 8888/tcp
```

## License

MIT
