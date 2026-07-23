#!/bin/bash
set -e

# Source ROS2 and the built workspace, then hand off to the container CMD.
#
# Runtime configuration (which config file to load, per-lidar overrides such as
# device IP / UDP port / topics / frame_id) is handled by the launch file
# (launch/start.py) via environment variables and inline launch arguments.
# See DOCKER_README.md for the full HESAI_* variable convention. This script no
# longer edits config files -- the old sed-based templating clobbered every
# lidar in a multi-lidar config and is replaced by per-lidar launch rendering.

source /opt/ros/jazzy/setup.bash
if [ -f /hesai_ws/install/setup.bash ]; then
    source /hesai_ws/install/setup.bash
fi

exec "$@"
