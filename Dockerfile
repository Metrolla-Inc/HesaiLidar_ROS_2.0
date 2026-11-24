# Base: the same public image as before (ros:jazzy-ros-core-noble), now pinned by the digest of its
# multi-arch index as found on Docker Hub on 2026-11-21 (tag last updated 2026-11-20 06:18 UTC).
# A build from the bare tag on that day used exactly this image. To move it, change the digest
# here and in AGENTS.md together.
ARG BASE_IMAGE=ros:jazzy-ros-core-noble@sha256:d0b8e70ceecc98a1d475c6b9ca96de9c90bf4e91e7163bf4c7543b794ff7d1f2
# hadolint ignore=DL3006
FROM ${BASE_IMAGE}

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

# The environment contract (opt-in with METROLLA_DRIVER_CONTRACT=1, AGENTS.md). /entrypoint.sh
# execs contract-entrypoint.sh only when that variable is set; without it the image runs the
# legacy entrypoint as before. selfcheck fails the build if a model's configuration does not
# render or names a calibration file (angle correction, firetimes) that is not in this image.
COPY docker/hesai_env.py docker/contract-entrypoint.sh /opt/metrolla/hesai/
RUN chmod 0755 /opt/metrolla/hesai/contract-entrypoint.sh \
    && chmod -R a+rX /opt/metrolla/hesai \
    && python3 -I /opt/metrolla/hesai/hesai_env.py selfcheck

# sensor-tool (docker/sensor_tool.py): reads and sets the sensor's own address, destination and
# clock source. The host runs it as its own container, `--entrypoint sensor-tool`, with this
# sensor's driver stopped; the driver never runs it. Standard library Python, no ROS.
COPY docker/sensor_tool.py /usr/local/bin/sensor-tool
RUN chmod 0755 /usr/local/bin/sensor-tool \
    && python3 -I -c "import ast; ast.parse(open('/usr/local/bin/sensor-tool').read())"

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command to run the driver
CMD ["ros2", "launch", "hesai_ros_driver", "start.py"]
