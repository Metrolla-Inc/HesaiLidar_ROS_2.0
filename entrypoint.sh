#!/bin/bash
# Metrolla driver contract, opt-in (AGENTS.md). Only when METROLLA_DRIVER_CONTRACT is set and not
# "0" does the image switch to the environment contract; otherwise everything below this block is
# the v1.0.5 entrypoint, byte for byte (tests/legacy pins it).
if [ -n "${METROLLA_DRIVER_CONTRACT:-}" ] && [ "${METROLLA_DRIVER_CONTRACT}" != "0" ]; then
    exec /opt/metrolla/hesai/contract-entrypoint.sh "$@"
fi
# --- end of the Metrolla contract block; the v1.0.5 entrypoint follows ---
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

    # Apply topic name overrides if set
    [ -n "$POINT_CLOUD_TOPIC" ] && sed -i "s|ros_send_point_cloud_topic: .*|ros_send_point_cloud_topic: $POINT_CLOUD_TOPIC|g" "$CONFIG_PATH"
    [ -n "$PACKET_TOPIC" ] && sed -i "s|ros_send_packet_topic: .*|ros_send_packet_topic: $PACKET_TOPIC|g" "$CONFIG_PATH"
    [ -n "$PACKET_TOPIC" ] && sed -i "s|ros_recv_packet_topic: .*|ros_recv_packet_topic: $PACKET_TOPIC|g" "$CONFIG_PATH"
    [ -n "$IMU_TOPIC" ] && sed -i "s|ros_send_imu_topic: .*|ros_send_imu_topic: $IMU_TOPIC|g" "$CONFIG_PATH"
    [ -n "$PACKET_LOSS_TOPIC" ] && sed -i "s|ros_send_packet_loss_topic: .*|ros_send_packet_loss_topic: $PACKET_LOSS_TOPIC|g" "$CONFIG_PATH"
    [ -n "$FRAME_ID" ] && sed -i "s|ros_frame_id: .*|ros_frame_id: $FRAME_ID|g" "$CONFIG_PATH"
fi

# Source ROS2 and workspace
source /opt/ros/jazzy/setup.bash
if [ -f /hesai_ws/install/setup.bash ]; then
    source /hesai_ws/install/setup.bash
fi

# Add library path for shared objects
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/hesai_ws/install/hesai_ros_driver/lib

exec "$@"
