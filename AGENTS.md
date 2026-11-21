# AGENTS.md — HesaiLidar_ROS_2.0 (Metrolla fork)

## What this repo is

The **Hesai lidar ROS 2 driver** (vendor `HesaiTechnology/HesaiLidar_ROS_2.0`, base commit
`0f5a1be`, SDK submodule `bae81f2`, "V2.0.10"), forked for Metrolla, and the source of the fleet
image **`ghcr.io/metrolla-inc/hesai_driver:<tag>`** (balena-fleets `DRIVER_VERSION`, today
`v1.0.5` = `4b55ba0`). `Metrolla-Inc/HesaiDriver` is a different repo that nothing builds from
(its `docs/VENDOR_DIFF.md` compares the two with the vendor).

| Path | Whose | What |
|---|---|---|
| `src/`, `node/`, `msg/`, `launch/`, `CMakeLists.txt`, `package.xml` | vendor + Metrolla | driver; Metrolla changed `start.py`, `source_driver_ros2.hpp` (topic/frame parameters) and the CMake install |
| `config/<model>.yaml` | Metrolla | one configuration per sensor model; `config.yaml` is what the legacy node reads |
| `Dockerfile`, `entrypoint.sh` | Metrolla | the image; `entrypoint.sh` is the v1.0.5 entrypoint plus the opt-in block |
| `docker/` | Metrolla | the environment contract (`hesai_env.py`, `contract-entrypoint.sh`) and the smoke test |
| `tests/`, `pytest.ini`, `.github/workflows/` | Metrolla | the three CI tiers (`TESTING.md`) and the image release |

## Two modes, one image

The image runs in one of two modes, chosen per container by **`METROLLA_DRIVER_CONTRACT`**:

| `METROLLA_DRIVER_CONTRACT` | Mode | Who |
|---|---|---|
| unset, empty or `0` | **legacy**: exactly the v1.0.5 behaviour | every existing fleet (they do not set it) |
| `1` | **contract**: the environment contract below | metrolla-edge (its generator sets it), new deployments |
| anything else (`true`, `2`, ...) | refused: exit 1, one line naming the variable and `1` | — |

`entrypoint.sh` starts with a 3-line block that execs `/opt/metrolla/hesai/contract-entrypoint.sh`
when the switch is set and not `0`; everything after the block is the v1.0.5 entrypoint byte for
byte (`tests/legacy` checks this against the tag's git blob). The default command, entrypoint path
and user (root) of the image are unchanged.

### Legacy mode (unchanged, documented)

`LIDAR_MODEL`, `SENSOR_IP`/`DEVICE_IP`, `SENSOR_PORT`/`UDP_PORT`, `HOST_IP`, `CONFIG_PATH`,
`OVERWRITE_CONFIG`, `POINT_CLOUD_TOPIC`, `PACKET_TOPIC`, `IMU_TOPIC`, `PACKET_LOSS_TOPIC`,
`FRAME_ID` as in `DOCKER_README.md`. **What it really does** (found for C9, kept as it is):

- With `OVERWRITE_CONFIG=true` (or no `config.yaml` in the install space, which never happens: the
  CMake install ships one) it copies `config/<LIDAR_MODEL>.yaml` (unknown or empty model:
  `PandarXT-16.yaml`, with a warning on stdout) to
  `/hesai_ws/install/hesai_ros_driver/share/hesai_ros_driver/config/config.yaml` and rewrites it
  with `sed`.
- **The node never reads that file.** `hesai_ros_driver_node` reads the ROS parameter
  `config_path`, else `PROJECT_PATH + "/config/config.yaml"`, and `PROJECT_PATH` is compiled in as
  the package *source* directory (`CMakeLists.txt`: `-DPROJECT_PATH="${PROJECT_SOURCE_DIR}"`),
  `/hesai_ws/src` in the image. `launch/start.py` passes no `config_path`. So the legacy node always
  runs `/hesai_ws/src/config/config.yaml` (the repo's `config/config.yaml`: sensor
  `192.168.1.201`, UDP 2368, PTC 9347, frame `hesai_lidar`, `/lidar_points`, `send_packet_ros:
  false`, calibration from the sensor over PTC, **no firetime file**), whatever `LIDAR_MODEL`,
  `SENSOR_IP`, `SENSOR_PORT`, `HOST_IP`, `FRAME_ID` or `POINT_CLOUD_TOPIC` say. Only the
  `start.py` launch arguments (`frame_id:=` ...) reach the node. Verified by reading the code, by
  the string `/hesai_ws/src` compiled into a local build, and by `tests/legacy`.
- The fleets work because their values equal `config.yaml`'s (mirasys-dev: `192.168.1.201`,
  2368, `hesai_lidar`, `/lidar_points`). A JT128 unit set to `LIDAR_MODEL=JT128` also runs
  `config.yaml`, not `JT128.yaml`.
- Why legacy is not fixed: making the node read the rendered file would change every fleet unit
  at once (for example `PandarXT-32.yaml` has `send_packet_ros: true`, and firetime correction
  would switch on), and refusing unknown models would crash-loop units that run today. Fleets get
  the fix by opting in, one at a time.

### Contract mode (`METROLLA_DRIVER_CONTRACT=1`)

The same contract as the LSLiDAR and Quanergy images (`Metrolla-Inc/lslidar_ros2_driver`,
`Metrolla-Inc/quanergy_client_ros`) and the Seyond image (`Metrolla-Inc/seyond_ros_driver`).
`docker/hesai_env.py` checks the environment, renders `config/<model>.yaml` with the contract's
values to **`$HESAI_RUN_DIR/config.yaml`** (`/tmp/hesai/config.yaml`), and execs
`hesai_ros_driver_node --ros-args [-r __ns:=/<SENSOR_ID>] -p config_path:=<that file>`: **the file
it renders is the file the node reads**, and the node is PID 1 (SIGTERM reaches it).
Subcommands: none or `run`; `check` / `check --json` (print what would be written and executed,
start nothing); the image's default command `ros2 launch hesai_ros_driver start.py` means `run`.
Anything else is refused.

**Any value the image does not understand stops it**: exit code 1, one line per problem on stderr,
naming the variable and what it accepts, for example

```
hesai-driver: refusing to start: LIDAR_MODEL='PandarXT32-v2' is not a Hesai model this image has a configuration for; accepted (any case, optional 'hesai-' prefix, '-' and '_' ignored): at128e2x, at128e3x, ...
```

| Variable | Required | Accepted | Default | Becomes |
|---|---|---|---|---|
| `METROLLA_DRIVER_CONTRACT` | yes | `1` | — | contract mode |
| `LIDAR_MODEL` | yes | a model below, any case, optional `hesai-` prefix, `-`/`_` ignored | — | `config/<model>.yaml` as the base |
| `SENSOR_IP` | yes | the sensor's IPv4 address, dotted decimal, unicast | — | `device_ip_address` (PTC target) |
| `SENSOR_ID` | no | `^[a-z][a-z0-9_]{0,30}$` without `__` | none | node namespace `/<id>`; relative topics follow it |
| `DATA_PORT` | no | 1024-65535 | 2368 | `udp_port` |
| `DEVICE_PORT` | no | 1024-65535 | 9347 | `ptc_port` (the sensor's PTC TCP port) |
| `HOST_IP` | no | IPv4, optionally `/1`-`/32`; not the sensor; with a prefix the sensor must be inside | model file (`192.168.1.100`) | a check, and `host_ip_address` (used by the SDK for SOME/IP only, which is off) |
| `FRAME_ID` | no | 1-64 of letters, digits, `_`, not starting with a digit | `hesai_lidar` | `ros_frame_id` |
| `POINTCLOUD_TOPIC` | no | a ROS topic name, absolute or relative | `lidar_points` (relative: `/lidar_points` without `SENSOR_ID`) | `ros_send_point_cloud_topic` |
| `TIME_MODE` | no | `host`, `sensor`, `gps`, `ptp`, `ntp` (any case) | `sensor` | `use_timestamp_type`: 1 for `host` (receive time), else 0 (the sensor's time; the sensor must be synchronised that way, the image does not program it) |
| `PUBLISH_SCAN` | no | false values only | — | the driver has no LaserScan |
| `HESAI_RUN_DIR` | no | an absolute path | `/tmp/hesai` | where `config.yaml` is written |
| `DUAL_LIDAR` | no | unset, empty or false | — | one container per sensor |

- **Empty values are refused** (`DATA_PORT=`), except `DUAL_LIDAR=`.
- **Legacy variables are refused** in contract mode with the contract's name: `SENSOR_PORT` and
  `UDP_PORT` (use `DATA_PORT`), `DEVICE_IP` (`SENSOR_IP`), `POINT_CLOUD_TOPIC`
  (`POINTCLOUD_TOPIC`), `OVERWRITE_CONFIG`, `CONFIG_PATH`, `PACKET_TOPIC`, `IMU_TOPIC`,
  `PACKET_LOSS_TOPIC` (remove).
- **Typos are refused** with "did you mean": any other variable starting with `HESAI_`, `LIDAR_`,
  `SENSOR_`, `DATA_`, `DEVICE_`, `HOST_`, `TIME_`, `FRAME_`, `POINTCLOUD_`, `POINT_`, `PUBLISH_`,
  `PACKET_`, `IMU_`, `UDP_`, `PTC_`, `CONFIG_`, `OVERWRITE_` or `DUAL_`. `METROLLA_*`, `ROS_*`,
  `RMW_*`, `FASTRTPS_*`, `TZ` pass through.
- **Calibration files must exist.** The per-model configs name files under
  `/hesai_ws/src/driver/HesaiLidar_SDK_2.0/`, but `COPY . src/` puts the SDK at
  `/hesai_ws/src/src/driver/HesaiLidar_SDK_2.0/` (the copy-path doubt of `VENDOR_DIFF.md` 5.4.1:
  confirmed by the Dockerfile; in legacy mode it does not matter because the node reads
  `config.yaml`, which names no file). The contract rewrites that prefix and **refuses to start**
  if a named angle-correction or firetime file is missing or unreadable; the image build runs
  `hesai_env.py selfcheck`, which fails the build if any model's files are missing. So in contract
  mode **firetime correction is on** for the models whose config names a firetime file (it is off in
  legacy mode): points differ from legacy by up to centimetres (22.8 mm in C13's synthetic XT32
  run). This is the vendor's intended behaviour, not yet compared on a real sensor.
- The rendered file keeps every other value of the model file (PTC on, `send_packet_ros`,
  `send_imu_ros`, ...). Packet, IMU and packet-loss topics are made relative (`lidar_packets`,
  `lidar_imu`, `lidar_packets_loss`) so they follow `SENSOR_ID`; without it they are the same
  absolute names as before.
- Models: `at128e2x` (alias `at128p`), `at128e3x`, `atx`, `et25`, `ft120`, `jt128`, `ot128`,
  `pandar128e3x`, `pandar40p`, `pandar64`, `pandar90e3x`, `pandarqt`, `pandarxt-16` (alias `xt16`),
  `pandarxt-32` (alias `xt32`), `qt128c2x`, `xt32m2x`. Refused: `at128` (ambiguous: AT128E2X or
  AT128E3X), `jt16` (serial RS-485 in this image; run it in legacy mode), the two `dual_*`
  configurations (one container per sensor).
- **Runs as any uid** (97000 in the product): everything is world-readable, it writes only
  `$HESAI_RUN_DIR` and `ROS_HOME`/`ROS_LOG_DIR` (default `/tmp/ros`, `/tmp/ros/log`).
  **Read-only root** works with a writable `/tmp` (`read_only: true`, `tmpfs: [/tmp]`); without one
  it refuses with "cannot write". Host network (UDP from the sensor, PTC to it), host IPC (Fast DDS
  shared memory), `cap_drop: [ALL]`, `no-new-privileges`. No health check is built into the image
  (adding one would change legacy containers); the product compose can use
  `grep -qsx hesai_ros_drive /proc/[0-9]*/comm` (liveness only).
- With PTC on (every model config), the v2.0.10 SDK **waits without a timeout** for the PTC
  connection before it decodes anything: a sensor that does not answer PTC gives a running node
  with no cloud (as in legacy mode).

A metrolla-edge driver template would pass:

```yaml
    user: {{ ids.dds }}
    read_only: true
    tmpfs: [/tmp]
    network_mode: host
    ipc: host
    cap_drop: [ALL]
    security_opt: ["no-new-privileges:true"]
    environment:
      METROLLA_DRIVER_CONTRACT: 1
      SENSOR_ID: {{ sensor.id }}
      LIDAR_MODEL: {{ sensor.model }}      # hesai-xt32, hesai-jt128; hesai-at128 must become at128e2x or at128e3x
      SENSOR_IP: {{ sensor.ip }}
      DATA_PORT: {{ sensor.data_port }}
      POINTCLOUD_TOPIC: /lidar_points       # or leave the default: /<sensor.id>/lidar_points
      TIME_MODE: ptp
      ROS_LOCALHOST_ONLY: 1
```

## Image and release

- Base: `ros:jazzy-ros-core-noble` as before, pinned by its multi-arch index digest
  `sha256:d0b8e70ceecc98a1d475c6b9ca96de9c90bf4e91e7163bf4c7543b794ff7d1f2` (Docker Hub,
  2026-11-21; tag updated 2026-11-20 06:18 UTC). A build from the bare tag that day used exactly
  this image. v1.0.5 was built from whatever the tag was then; that cannot be reproduced.
- Build steps unchanged otherwise (colcon Release, rviz2, `/entrypoint.sh`, default command); the
  contract adds `/opt/metrolla/hesai/` and the build-time `selfcheck`.
- **Release:** `.github/workflows/docker-build.yml` pushes
  `ghcr.io/metrolla-inc/hesai_driver:<tag>` (amd64 + arm64) **only for a pushed `v*` git tag**,
  refuses to overwrite a tag that already exists in the registry, and records the digest (job
  summary and the `image-digest` artifact). Pull requests build both platforms without logging in
  or pushing. Branch pushes no longer push anything (before: `ROS2/main`, `ROS2/dev` and `v*`
  branches pushed `:ros2-main`, `:ros2-dev`, `:v...`).

## Rules for agents working here

- Legacy mode must not change: `tests/legacy` compares every fleet environment with goldens made
  from the v1.0.5 entrypoint. Add a fleet environment there when a fleet's compose changes.
- A change of the contract changes a golden: regenerate on purpose (`REGEN=1`), read the diff,
  update the table above in the same commit.
- Vendor files: minimal changes, one commit each, with the reason. This contract needed none.
- Every bug fix adds a regression test. Never push an image except through a tag.
