FROM ros:jazzy-ros-core-noble

# Set shell for sourcing
SHELL ["/bin/bash", "-c"]

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    libyaml-cpp-dev \
    libpcap-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /hesai_ws

# Copy the source code
COPY . src/

# Build the workspace
RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --parallel-workers $(nproc) --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN apt-get update && apt-get install -y ros-jazzy-rviz2

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command to run the driver
CMD ["ros2", "launch", "hesai_ros_driver", "start.py"]
