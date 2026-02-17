# Changelog

All notable changes to the Hesai Lidar ROS2 Driver project will be documented in this file.

## [1.5.1] - 2024-03-21

### Added
- **Multi-Arch Docker Build**: Added GitHub Actions workflow for building and pushing Docker images for both `linux/amd64` and `linux/arm64`.
- **Dynamic Configuration**: Added `entrypoint.sh` to dynamically configure the driver using environment variables (`SENSOR_IP`, `SENSOR_PORT`, `HOST_IP`, `LIDAR_MODEL`).
- **New Lidar Model Support**: Added dedicated configuration files and correction mapping for:
  - `JT128`
  - `XT32M2X`
  - `Pandar90E3X`
  - `QT128C2X`
  - `ATX`
  - `FT120`
  - `ET25`
  - `PandarQT`
  - `AT128E3X`
- **Docker Documentation**: Created `DOCKER_README.md` with detailed model mapping and deployment instructions.

### Changed
- **Config Baseline**: Synchronized `config/config.yaml` with `OT128.yaml` as the new baseline for Ethernet-based sensors.
- **Network Defaults**: Standardized `ptc_port` (9347) and `host_ip_address` (192.168.1.100) across all configuration files.
- **Correction Paths**: Updated all dedicated configuration files to use absolute paths pointing to the SDK's correction files within the Docker environment.
- **Dockerfile Improvements**: Added `libboost-all-dev` and `libyaml-cpp-dev` dependencies to the Docker image for reliable builds.

### Removed
- **Multicast Support**: Removed `multicast_ip_address` from all configuration files to focus on stable unicast communication.
- **Placeholders**: Removed custom placeholders from YAML files to ensure they remain valid and compatible with the standard ROS driver.
