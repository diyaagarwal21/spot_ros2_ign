# Use Space ROS base image
FROM osrf/space-ros:latest

# Install dependencies (example: Python, additional ROS 2 packages, etc.)
RUN apt-get update && apt-get install -y \
    ros-humble-robot-state-publisher \
    ros-humble-gazebo-ros-pkgs \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /app

# Copy project files into the container
COPY . /app

# Run the demo (replace this with your actual entry command)
CMD ["ros2", "launch", "champ_bringup", "champ_bringup.launch.py"]
