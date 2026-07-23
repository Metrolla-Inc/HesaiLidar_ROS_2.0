### Docker Deployment for Hesai Lidar ROS2 Jazzy

This document explains how to deploy and configure the Hesai Lidar ROS2 driver using Docker and Balena.

#### Multi-Architecture Support
The Docker image is built for both `linux/amd64` and `linux/arm64` (e.g., Jetson, Raspberry Pi), making it suitable for a wide range of deployment environments.

#### Configuration model
The driver runs as a single node that loads **one** config file whose top-level `lidar:` list can hold one or more sensors (dual-lidar is just two entries in that list — see `config/config.yaml`). Configuration happens entirely at launch time inside `launch/start.py`: it selects which config file to load and applies per-lidar overrides, then hands the rendered config to the node. There is **no `sed` templating** — the launch file edits only the fields you ask it to, so multi-lidar configs are never clobbered.

Every setting below works the same way whether passed as a container environment variable (`docker run -e ...`, Balena compose) or as an inline `ros2 launch` argument.

#### Config selection
The base config file is chosen by the first of these that is set:

| Variable | Inline arg | Description | Default |
| :--- | :--- | :--- | :--- |
| `HESAI_CONFIG_PATH` | `config_path:=` | Absolute path to a full custom config file (e.g. volume-mounted). Used as-is. | *(unset)* |
| `HESAI_CONFIG_FILE` | `config_file:=` | Basename resolved inside the packaged `config/` dir (e.g. `config.yaml`, `config1.yaml`). | `config.yaml` |
| `LIDAR_MODEL` | — | Selects `<MODEL>.yaml` as the base when neither of the above is set (see model table below). | *(unset)* |

Other top-level knobs: `HESAI_NAMESPACE` / `namespace:=` (default `hesai_ros_driver`), `HESAI_LOG_LEVEL` / `log_level:=` (default `info`), `HESAI_RVIZ` / `rviz:=true` to launch rviz2, and `HESAI_RENDERED_CONFIG` to choose where the rendered config is written (default a temp file).

#### Per-lidar overrides
Override any individual field of lidar index `i` (0-based position in the `lidar:` list). Inline args take precedence over env vars.

| Field | Env var (`i` = lidar index) | Inline arg |
| :--- | :--- | :--- |
| Device IP | `HESAI_LIDAR{i}_DEVICE_IP` | `lidar{i}_device_ip:=` |
| UDP port | `HESAI_LIDAR{i}_UDP_PORT` | `lidar{i}_udp_port:=` |
| PTC port | `HESAI_LIDAR{i}_PTC_PORT` | `lidar{i}_ptc_port:=` |
| Host IP | `HESAI_LIDAR{i}_HOST_IP` | `lidar{i}_host_ip:=` |
| Correction file | `HESAI_LIDAR{i}_CORRECTION_FILE` | `lidar{i}_correction_file:=` |
| Firetimes file | `HESAI_LIDAR{i}_FIRETIMES_FILE` | `lidar{i}_firetimes_file:=` |
| Source type | `HESAI_LIDAR{i}_SOURCE_TYPE` | `lidar{i}_source_type:=` |
| Frame id | `HESAI_LIDAR{i}_FRAME_ID` | `lidar{i}_frame_id:=` |
| Point cloud topic | `HESAI_LIDAR{i}_POINT_TOPIC` | `lidar{i}_point_topic:=` |
| Packet topic | `HESAI_LIDAR{i}_PACKET_TOPIC` | `lidar{i}_packet_topic:=` |
| IMU topic | `HESAI_LIDAR{i}_IMU_TOPIC` | `lidar{i}_imu_topic:=` |
| Packet-loss topic | `HESAI_LIDAR{i}_PACKET_LOSS_TOPIC` | `lidar{i}_packet_loss_topic:=` |
| Publish point cloud | `HESAI_LIDAR{i}_SEND_POINT_CLOUD_ROS` | `lidar{i}_send_point_cloud_ros:=` |
| Publish packets | `HESAI_LIDAR{i}_SEND_PACKET_ROS` | `lidar{i}_send_packet_ros:=` |
| Publish IMU | `HESAI_LIDAR{i}_SEND_IMU_ROS` | `lidar{i}_send_imu_ros:=` |

Dual-lidar example (override both sensors' IPs and the second UDP port):

```bash
ros2 launch hesai_ros_driver start.py \
  config_file:=config.yaml \
  lidar0_device_ip:=192.168.1.201 \
  lidar1_device_ip:=192.168.1.202 lidar1_udp_port:=2369
# equivalently, as env vars:
HESAI_CONFIG_FILE=config.yaml \
HESAI_LIDAR0_DEVICE_IP=192.168.1.201 \
HESAI_LIDAR1_DEVICE_IP=192.168.1.202 HESAI_LIDAR1_UDP_PORT=2369 \
  ros2 launch hesai_ros_driver start.py
```

Run `ros2 launch hesai_ros_driver start.py --show-args` to list the declared arguments. Indices `>= 2` are supported through the environment variables even though only `lidar0_*`/`lidar1_*` appear in `--show-args`.

**Backward-compatible aliases** (applied to lidar 0): `SENSOR_IP`/`DEVICE_IP` → device IP, `SENSOR_PORT`/`UDP_PORT` → UDP port, `HOST_IP` → host IP. The old `CONFIG_PATH` variable is superseded by `HESAI_CONFIG_PATH`.

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
      # Dual-lidar: config.yaml already lists two sensors; override their IPs/ports here.
      - HESAI_CONFIG_FILE=config.yaml
      - HESAI_LIDAR0_DEVICE_IP=192.168.1.201
      - HESAI_LIDAR0_UDP_PORT=2368
      - HESAI_LIDAR1_DEVICE_IP=192.168.1.202
      - HESAI_LIDAR1_UDP_PORT=2369
      - ROS_LOCALHOST_ONLY=1
```

For a single sensor, point at a one-entry config (e.g. `HESAI_CONFIG_FILE=config2.yaml`) or select by model with `LIDAR_MODEL=AT128E2X` and override `HESAI_LIDAR0_DEVICE_IP`. The legacy `SENSOR_IP` / `HOST_IP` names still work as lidar-0 aliases.

#### Custom Configuration
If you need to use a totally custom configuration file, map it as a volume and point `HESAI_CONFIG_PATH` to it. Per-lidar overrides still apply on top of it:

```yaml
    volumes:
      - ./my_custom_config.yaml:/hesai_ws/custom_config.yaml
    environment:
      - HESAI_CONFIG_PATH=/hesai_ws/custom_config.yaml
```
