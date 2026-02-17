#!/bin/bash
set -e

# Default values
LIDAR_MODEL=${LIDAR_MODEL:-"PandarXT-16"}
DEVICE_IP=${SENSOR_IP:-${DEVICE_IP:-"192.168.1.201"}}
UDP_PORT=${SENSOR_PORT:-${UDP_PORT:-"2368"}}
HOST_IP=${HOST_IP:-"192.168.1.100"}
CONFIG_PATH=${CONFIG_PATH:-"/hesai_ws/install/hesai_ros_driver/share/hesai_ros_driver/config/config.yaml"}
SOURCE_CONFIG_DIR="/hesai_ws/install/hesai_ros_driver/share/hesai_ros_driver/config"

# Create config from dedicated file if not provided as a volume
if [ ! -f "$CONFIG_PATH" ] || [ "$OVERWRITE_CONFIG" = "true" ]; then
    echo "Using dedicated config for model: $LIDAR_MODEL"
    mkdir -p $(dirname $CONFIG_PATH)
    
    # Select the source config file
    if [ -f "$SOURCE_CONFIG_DIR/${LIDAR_MODEL}.yaml" ]; then
        SELECTED_CONFIG="$SOURCE_CONFIG_DIR/${LIDAR_MODEL}.yaml"
    else
        echo "Warning: Dedicated config for $LIDAR_MODEL not found. Falling back to PandarXT-16."
        SELECTED_CONFIG="$SOURCE_CONFIG_DIR/PandarXT-16.yaml"
    fi

    # Copy and update config values
    cp "$SELECTED_CONFIG" "$CONFIG_PATH"
    sed -i "s/device_ip_address: .*/device_ip_address: $DEVICE_IP/g" "$CONFIG_PATH"
    sed -i "s/udp_port: .*/udp_port: $UDP_PORT/g" "$CONFIG_PATH"
    sed -i "s/host_ip_address: .*/host_ip_address: \"$HOST_IP\"/g" "$CONFIG_PATH"
fi

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
if [ -f /hesai_ws/install/setup.bash ]; then
    source /hesai_ws/install/setup.bash
fi

exec "$@"
