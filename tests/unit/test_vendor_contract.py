"""Unit tier: the contract against the source it relies on, read as text (no ROS, no build).

If a vendor update or a Dockerfile change moves any of these, the contract would silently stop
working; these tests turn that into a failure.
"""

from __future__ import annotations

import re
from pathlib import Path

import hesai_env as h

REPO = Path(__file__).resolve().parents[2]
NODE_CC = (REPO / "node" / "hesai_ros_driver_node.cc").read_text()
CMAKE = (REPO / "CMakeLists.txt").read_text()
YAML_READER = (REPO / "src" / "manager" / "source_drive_common.hpp").read_text()
START_PY = (REPO / "launch" / "start.py").read_text()
DOCKERFILE = (REPO / "Dockerfile").read_text()
GITMODULES = (REPO / ".gitmodules").read_text()


def yaml_reads(node: str) -> set[str]:
    """Keys read with YamlRead<...>(<node>, "key", ...) in source_drive_common.hpp."""
    return set(re.findall(r'YamlRead<[^>]+>\(\s*' + re.escape(node) + r'\s*,\s*"([a-z_]+)"', YAML_READER))


def test_every_key_the_contract_sets_is_read_by_the_driver():
    udp = yaml_reads('driver_config["lidar_udp_type"]')
    assert {"device_ip_address", "udp_port", "ptc_port", "host_ip_address", "correction_file_path",
            "firetimes_path"} <= udp
    assert {"source_type", "use_timestamp_type"} <= yaml_reads("driver_config")
    assert {"ros_frame_id", "ros_send_point_cloud_topic", "ros_send_packet_topic", "ros_recv_packet_topic",
            "ros_send_imu_topic", "ros_send_packet_loss_topic", "send_point_cloud_ros"} <= yaml_reads('config["ros"]')


def test_the_driver_reads_a_bounded_ptc_connect_timeout():
    """With use_ptc_connected true the node waited for the sensor's PTC port for ever: the SDK's
    InputParam.ptc_connect_timeout (seconds; below 0 for ever) was never read from config.yaml, so a
    sensor that streams but does not answer PTC gave a running driver and no clouds (found by
    metrolla-edge's C16 replay). It is read now, beside use_ptc_connected, with the SDK's default."""
    assert "ptc_connect_timeout" in yaml_reads('driver_config["lidar_udp_type"]')
    assert re.search(r'YamlRead<float>\(\s*driver_config\["lidar_udp_type"\],\s*"ptc_connect_timeout",\s*'
                     r'driver_param\.input_param\.ptc_connect_timeout,\s*-1\)', YAML_READER)


def test_the_udp_block_is_read_only_for_source_type_1():
    assert re.search(r"if \(source_type == 1\) \{\s*YamlRead<std::string>\(driver_config\[\"lidar_udp_type\"\], \"device_ip_address\"",
                     YAML_READER)


def test_use_timestamp_type_0_is_the_sensors_time():
    types = (REPO / "src/driver/HesaiLidar_SDK_2.0/libhesai/lidar_types.h")
    if types.exists():  # submodule
        assert "use_timestamp_type;  // 0: point cloud time, other: local time" in types.read_text()
    assert h.TIME_MODES["host"] == 1 and all(v == 0 for k, v in h.TIME_MODES.items() if k != "host")


# ---- the mechanism behind "the Hesai driver ignores its rendered config" (past bug) ----

def test_node_default_config_is_the_compiled_in_source_directory():
    """Without config_path the node reads PROJECT_PATH/config/config.yaml, PROJECT_PATH being the
    package source directory at build time (/hesai_ws/src in the image)."""
    assert 'add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")' in CMAKE
    assert "config_path = (std::string)PROJECT_PATH;" in NODE_CC
    assert 'config_path += "/config/config.yaml";' in NODE_CC
    # RUN_IN_ROS_WORKSPACE (look the package up instead) is defined once, in the ROS 1 block.
    assert CMAKE.count("-DRUN_IN_ROS_WORKSPACE") == 1
    start = CMAKE.index('if($ENV{ROS_VERSION} MATCHES "1")')
    assert start < CMAKE.index("-DRUN_IN_ROS_WORKSPACE") < CMAKE.index("endif", start)


def test_node_takes_config_path_from_a_ros_parameter():
    assert re.search(r'declare_parameter<std::string>\("config_path", ""\)', NODE_CC)
    assert "config_path = path;" in NODE_CC


def test_the_legacy_launch_file_passes_no_config_path():
    """So the legacy entrypoint's rendered file (install space) is never read by the node."""
    assert "config_path" not in START_PY


def test_the_contract_hands_the_rendered_file_to_the_node():
    s = h.parse({"METROLLA_DRIVER_CONTRACT": "1", "LIDAR_MODEL": "xt32", "SENSOR_IP": "192.168.1.201"})
    argv = h.node_argv(s)
    assert argv[argv.index("-p") + 1] == f"config_path:={s.config_path}"
    assert s.config_path.startswith(s.run_dir)


def test_dockerfile_layout_matches_the_contracts_paths():
    assert "WORKDIR /hesai_ws" in DOCKERFILE
    assert "COPY . src/" in DOCKERFILE  # so the package source (PROJECT_PATH) is /hesai_ws/src
    assert "path = src/driver/HesaiLidar_SDK_2.0" in GITMODULES  # -> /hesai_ws/src/src/driver/...
    assert "COPY docker/hesai_env.py docker/contract-entrypoint.sh /opt/metrolla/hesai/" in DOCKERFILE
    assert "hesai_env.py selfcheck" in DOCKERFILE
    assert 'ENTRYPOINT ["/entrypoint.sh"]' in DOCKERFILE
    assert 'CMD ["ros2", "launch", "hesai_ros_driver", "start.py"]' in DOCKERFILE
    assert h.LEGACY_DEFAULT_CMD == ["ros2", "launch", "hesai_ros_driver", "start.py"]


def test_the_model_configs_name_an_sdk_path_that_is_not_in_the_image():
    """VENDOR_DIFF.md 5.4.1 (the copy-path doubt): the per-model configs point at
    /hesai_ws/src/driver/HesaiLidar_SDK_2.0, but COPY . src/ puts the SDK one level deeper. The
    contract rewrites the prefix; legacy keeps the configs as they are."""
    text = (REPO / "config" / "PandarXT-32.yaml").read_text()
    assert h.CONFIG_SDK_PREFIX + "correction/firetime_correction/" in text
    assert not h.CONFIG_SDK_PREFIX.startswith("/hesai_ws/src/src/")


def test_the_cmake_install_ships_config():
    ros2 = CMAKE[CMAKE.index("#Ros2#"):]
    assert re.search(r"install\(DIRECTORY\s+launch\s+rviz\s+config\s+DESTINATION share/\$\{PROJECT_NAME\}\)", ros2)
