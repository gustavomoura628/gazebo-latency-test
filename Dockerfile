# Gazebo Latency Test - Docker Image
# ROS 2 Humble + TurtleBot3 + Gazebo Classic

FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Re-setup ROS repository with fresh key (base image key may be expired)
RUN rm -f /etc/apt/sources.list.d/ros2*.list && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

# Install TurtleBot3 packages, GUI dependencies, and Python tools
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-teleop \
    python3-pip \
    tmux \
    mesa-utils \
    xvfb \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    pygame \
    opencv-python-headless \
    numpy

# Set up workspace
WORKDIR /app

# Copy application files
COPY game_teleop.py video_server.py run.sh docker-entrypoint.sh ./
COPY models/ ./models/
COPY worlds/ ./worlds/

# Make scripts executable
RUN chmod +x run.sh docker-entrypoint.sh

# Environment variables
ENV TURTLEBOT3_MODEL=waffle_pi
ENV ROS_LOCALHOST_ONLY=1
ENV ROS_DOMAIN_ID=0
ENV GAZEBO_MODEL_PATH=/app/models:/opt/ros/humble/share/turtlebot3_gazebo/models:/usr/share/gazebo-11/models

# Pre-warm Gazebo: run once during build to compile shaders and cache models
# This makes runtime startup MUCH faster
RUN echo "Pre-warming Gazebo (compiling shaders, caching models)..." && \
    . /opt/ros/humble/setup.sh && \
    export LIBGL_ALWAYS_SOFTWARE=1 && \
    Xvfb :99 -screen 0 1024x768x24 & \
    export DISPLAY=:99 && \
    sleep 2 && \
    timeout 60 gzserver /app/worlds/turtlebot3_world.world --verbose 2>&1 | head -50 || true && \
    echo "Pre-warm complete!"

# Expose video server port
EXPOSE 8888

# Default command - use docker-entrypoint for better Docker compatibility
CMD ["./docker-entrypoint.sh"]
