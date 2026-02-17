### Docker Deployment for Hesai Lidar ROS2 Jazzy

This document explains how to deploy and configure the Hesai Lidar ROS2 driver using Docker and Balena.

#### Multi-Architecture Support
The Docker image is built for both `linux/amd64` and `linux/arm64` (e.g., Jetson, Raspberry Pi), making it suitable for a wide range of deployment environments.

#### Environment Variables
The containerized driver is configured via environment variables. These variables override the default values in the configuration files at runtime.

| Variable | Description | Default |
| :--- | :--- | :--- |
| `LIDAR_MODEL` | The model of the Hesai Lidar (e.g., `JT128`, `OT128`) | `PandarXT-16` |
| `SENSOR_IP` | IP address of the Lidar sensor | `192.168.1.201` |
| `SENSOR_PORT` | UDP port for point cloud data | `2368` |
| `HOST_IP` | IP address of the host machine | `192.168.1.100` |
| `CONFIG_PATH` | Override the path to the internal config file | `/hesai_ws/install/hesai_ros_driver/share/hesai_ros_driver/config/config.yaml` |

#### Lidar Model to Configuration Mapping
The `LIDAR_MODEL` environment variable selects a pre-defined configuration file from the `config/` directory. For models with multiple revisions or compatible versions, refer to the mapping below.

| Deployment `LIDAR_MODEL` Value | Base Lidar Model / Compatible Model | Config File |
| :--- | :--- | :--- |
| `AT128E2X` | **AT128P**, AT128E2X | `AT128E2X.yaml` |
| `AT128E3X` | AT128E3X | `AT128E3X.yaml` |
| `OT128` | OT128 | `OT128.yaml` |
| `JT128` | JT128 | `JT128.yaml` |
| `JT16` | JT16 (Serial) | `JT16.yaml` |
| `PandarXT-16` | PandarXT-16 | `PandarXT-16.yaml` |
| `PandarXT-32` | PandarXT-32 | `PandarXT-32.yaml` |
| `XT32M2X` | XT32M2X | `XT32M2X.yaml` |
| `Pandar40P` | Pandar40P | `Pandar40P.yaml` |
| `Pandar64` | Pandar64 | `Pandar64.yaml` |
| `Pandar128E3X` | Pandar128E3X | `Pandar128E3X.yaml` |
| `Pandar90E3X` | Pandar90E3X | `Pandar90E3X.yaml` |
| `QT128C2X` | QT128C2X | `QT128C2X.yaml` |
| `PandarQT` | PandarQT | `PandarQT.yaml` |
| `ATX` | ATX | `ATX.yaml` |
| `FT120` | FT120 | `FT120.yaml` |
| `ET25` | ET25 | `ET25.yaml` |

#### Balena Compose Example
To deploy on a Balena-managed device, use the following service configuration:

```yaml
services:
  met_lidar_driver:
    image: ghcr.io/metrolla-inc/hesai_driver:latest
    network_mode: host
    restart: always
    ipc: host
    environment:
      - LIDAR_MODEL=AT128E2X  # For AT128P models
      - SENSOR_IP=192.168.1.201
      - HOST_IP=192.168.1.100
      - ROS_LOCALHOST_ONLY=1
      - USE_MACADDRESS_NS=0
```

#### Custom Configuration
If you need to use a totally custom configuration file, you can map it as a volume and point the `CONFIG_PATH` variable to it:

```yaml
    volumes:
      - ./my_custom_config.yaml:/hesai_ws/custom_config.yaml
    environment:
      - CONFIG_PATH=/hesai_ws/custom_config.yaml
```
