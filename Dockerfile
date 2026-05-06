# Gazebo Latency Test - Docker Image
# ROS 2 Humble + TurtleBot3 + Gazebo Classic

FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

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
ENV LIBGL_ALWAYS_SOFTWARE=1

# Expose video server port
EXPOSE 8888

# Default command - use docker-entrypoint for better Docker compatibility
CMD ["./docker-entrypoint.sh"]
