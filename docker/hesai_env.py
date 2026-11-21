#!/usr/bin/env python3
"""Environment contract of the Metrolla Hesai driver image (opt-in: METROLLA_DRIVER_CONTRACT=1).

Reads the container's environment, refuses to start on any value it does not understand, renders
the driver's YAML configuration from the image's per-model file (config/<model>.yaml) and
(subcommand ``run``) execs ``hesai_ros_driver_node`` with ``config_path`` pointing at the rendered
file, so the file this script writes is the file the node reads, and signals reach the node.

Why ``config_path``: the node reads ``<PROJECT_PATH>/config/config.yaml`` unless the ROS parameter
``config_path`` is set (node/hesai_ros_driver_node.cc). PROJECT_PATH is the package *source*
directory, compiled in by CMakeLists.txt (``-DPROJECT_PATH="${PROJECT_SOURCE_DIR}"``), which is
``/hesai_ws/src`` in this image. The legacy entrypoint writes its rendered file to the install
space instead, so the legacy node never reads it (AGENTS.md, "Legacy mode").

Standard library only: it runs in the image before the driver and in the unit and regression tests
outside the image, where there is no ROS.

Subcommands:
  run            validate, write the configuration, exec the driver (the default)
  check [--json] validate and print what ``run`` would write and exec; nothing is started
  selfcheck      for the image build: every model renders, every calibration file a rendered
                 configuration names exists, the driver binary exists

Exit codes: 0 ok; 1 refused (a value this image does not understand, nowhere to write, a missing
calibration file, or an unknown command).
"""

from __future__ import annotations

import difflib
import ipaddress
import json
import os
import re
import sys
from dataclasses import dataclass, field
from typing import Mapping

PROG = "hesai-driver"
EXIT_OK = 0
EXIT_REFUSED = 1
EXIT_USAGE = 1

# Paths inside the image (Dockerfile: WORKDIR /hesai_ws, COPY . src/, colcon build).
WORKSPACE = "/hesai_ws"
PACKAGE_SOURCE = WORKSPACE + "/src"  # PROJECT_PATH compiled into the node
DRIVER_NODE = WORKSPACE + "/install/hesai_ros_driver/lib/hesai_ros_driver/hesai_ros_driver_node"
MODEL_CONFIG_DIR = WORKSPACE + "/install/hesai_ros_driver/share/hesai_ros_driver/config"
# The vendor SDK is a submodule at src/driver/HesaiLidar_SDK_2.0 of this repo, so after
# "COPY . src/" it is at /hesai_ws/src/src/driver/HesaiLidar_SDK_2.0. The per-model configs name
# /hesai_ws/src/driver/HesaiLidar_SDK_2.0/..., which does not exist in the image; this contract
# rewrites that prefix to the real one and checks the files exist.
SDK_DIR = PACKAGE_SOURCE + "/src/driver/HesaiLidar_SDK_2.0"
CONFIG_SDK_PREFIX = "/hesai_ws/src/driver/HesaiLidar_SDK_2.0/"
NODE_NAME = "hesai_ros_driver_node"
DEFAULT_RUN_DIR = "/tmp/hesai"
DEFAULT_ROS_HOME = "/tmp/ros"

DEFAULT_DATA_PORT = 2368  # vendor default udp_port
DEFAULT_DEVICE_PORT = 9347  # vendor default ptc_port (PTC: TCP on the sensor)
MIN_PORT = 1024
MAX_PORT = 65535
DEFAULT_FRAME_ID = "hesai_lidar"  # the per-model configs' ros_frame_id
# Relative on purpose: without SENSOR_ID it resolves to /lidar_points, the per-model configs'
# value; with SENSOR_ID it follows the node's namespace, so two sensors never share a topic.
DEFAULT_TOPIC = "lidar_points"

TRUE_WORDS = ("1", "true", "yes", "on")
FALSE_WORDS = ("0", "false", "no", "off")
# use_timestamp_type (src/manager/source_drive_common.hpp): 0 the point cloud's own (sensor)
# time, 1 the host's receive time. The vendor default and every per-model config use 0.
TIME_MODES = {"host": 1, "sensor": 0, "gps": 0, "ptp": 0, "ntp": 0}
DEFAULT_TIME_MODE = "sensor"

CONTRACT_SWITCH = "METROLLA_DRIVER_CONTRACT"
CONTRACT_VERSIONS = ("1",)


# --------------------------------------------------------------------------------------------
# Models: the per-model configuration files of config/ (one sensor each, UDP source).
# --------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Model:
    key: str  # normalised name: lowercase letters and digits only
    config: str  # config/<config>.yaml
    label: str


_MODEL_FILES = ("AT128E2X", "AT128E3X", "ATX", "ET25", "FT120", "JT128", "OT128", "Pandar128E3X",
                "Pandar40P", "Pandar64", "Pandar90E3X", "PandarQT", "PandarXT-16", "PandarXT-32",
                "QT128C2X", "XT32M2X")


def normalise_model(value: str) -> str:
    """Lowercase, without an optional 'hesai-' prefix, '-' and '_' removed. Anything else (a space,
    a dot) is kept, so it cannot match a model and the name is refused."""
    key = value.lower()
    if key.startswith("hesai-") or key.startswith("hesai_"):
        key = key[len("hesai-"):]
    return key.replace("-", "").replace("_", "")


MODELS: dict[str, Model] = {normalise_model(n): Model(normalise_model(n), n, n) for n in _MODEL_FILES}
# Other names for the same file: metrolla-edge's sensor model names (hesai-xt32 ...) and the
# product names DOCKER_README.md maps ("AT128P" uses AT128E2X.yaml).
MODEL_ALIASES = {"xt32": "pandarxt32", "xt16": "pandarxt16", "at128p": "at128e2x"}
REFUSED_MODELS = {
    "at128": "is ambiguous: use at128e2x (AT128P, AT128E2X) or at128e3x",
    "jt16": "is a serial (RS-485) sensor in this image (config/JT16.yaml source_type 4); this "
            "contract drives UDP sensors by SENSOR_IP only. Run it without "
            f"{CONTRACT_SWITCH} (legacy mode)",
    "dualjt128": "is a two-sensor configuration; run one container per sensor",
    "dualat128e2xpandarxt32": "is a two-sensor configuration; run one container per sensor",
    "config": "is not a model; name the sensor model",
}


def model_list() -> str:
    names = [m.label.lower() for m in MODELS.values()]
    return ", ".join(names) + " (also: " + ", ".join(f"{a} = {MODELS[k].label.lower()}"
                                                     for a, k in MODEL_ALIASES.items()) + ")"


def lookup_model(raw: str) -> Model | str:
    key = normalise_model(raw)
    key = MODEL_ALIASES.get(key, key)
    if key in REFUSED_MODELS:
        return REFUSED_MODELS[key]
    if key in MODELS:
        return MODELS[key]
    return ("is not a Hesai model this image has a configuration for; accepted (any case, optional "
            f"'hesai-' prefix, '-' and '_' ignored): {model_list()}")


# --------------------------------------------------------------------------------------------
# Environment variables.
# --------------------------------------------------------------------------------------------

REQUIRED = ("LIDAR_MODEL", "SENSOR_IP")
OPTIONAL = ("SENSOR_ID", "DATA_PORT", "DEVICE_PORT", "HOST_IP", "FRAME_ID", "POINTCLOUD_TOPIC",
            "TIME_MODE", "PUBLISH_SCAN", "HESAI_RUN_DIR")
COMPAT = ("DUAL_LIDAR",)  # only false: one container per sensor
KNOWN = REQUIRED + OPTIONAL + COMPAT
# The legacy entrypoint's variables. In contract mode they are refused with the contract's name,
# so a compose that switched the contract on but kept a legacy variable fails loudly.
LEGACY = {
    "SENSOR_PORT": "DATA_PORT", "UDP_PORT": "DATA_PORT", "DEVICE_IP": "SENSOR_IP",
    "POINT_CLOUD_TOPIC": "POINTCLOUD_TOPIC", "OVERWRITE_CONFIG": None, "CONFIG_PATH": None,
    "PACKET_TOPIC": None, "IMU_TOPIC": None, "PACKET_LOSS_TOPIC": None,
}
ALIASES = {"LIDAR_IP": "SENSOR_IP", "LIDAR_TYPE": "LIDAR_MODEL", "SENSOR_MODEL": "LIDAR_MODEL",
           "FRAME_TOPIC": "POINTCLOUD_TOPIC", "TIME_SERVICE": "TIME_MODE", "HOST_ADDRESS": "HOST_IP",
           "PTC_PORT": "DEVICE_PORT", "SENSOR_HOST_IP": "HOST_IP", "SENSOR_FRAME_ID": "FRAME_ID",
           "SENSOR_POINTS_TOPIC": "POINTCLOUD_TOPIC"}
STRICT_PREFIXES = ("HESAI_", "LIDAR_", "SENSOR_", "DATA_", "DEVICE_", "HOST_", "TIME_", "FRAME_",
                   "POINTCLOUD_", "POINT_", "PUBLISH_", "PACKET_", "IMU_", "UDP_", "PTC_",
                   "CONFIG_", "OVERWRITE_", "DUAL_")

ACCEPTED = {
    "LIDAR_MODEL": "a Hesai model name such as pandarxt-32 (or xt32), jt128 or at128e2x (the full "
                   "list is in the message when the name is unknown)",
    "SENSOR_IP": "the sensor's own IPv4 address, for example 192.168.1.201",
    "SENSOR_ID": "a lowercase letter then up to 30 lowercase letters, digits or single '_'",
    "DATA_PORT": f"a UDP port {MIN_PORT}-{MAX_PORT} (default {DEFAULT_DATA_PORT})",
    "DEVICE_PORT": f"the sensor's PTC (TCP) port {MIN_PORT}-{MAX_PORT} (default {DEFAULT_DEVICE_PORT})",
    "HOST_IP": "the host's IPv4 address on the sensor network, optionally with /1-/32",
    "FRAME_ID": f"letters, digits and '_', not starting with a digit (default {DEFAULT_FRAME_ID})",
    "POINTCLOUD_TOPIC": f"a ROS topic name (default {DEFAULT_TOPIC})",
    "TIME_MODE": f"host, sensor, gps, ptp or ntp (default {DEFAULT_TIME_MODE})",
    "PUBLISH_SCAN": "0, false, no or off (the Hesai driver publishes no LaserScan)",
    "HESAI_RUN_DIR": f"an absolute directory path (default {DEFAULT_RUN_DIR})",
}

_PORT_RE = re.compile(r"[1-9][0-9]{0,4}")
_OCTET = r"(?:25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9]?[0-9])"
_IPV4_RE = re.compile(rf"{_OCTET}\.{_OCTET}\.{_OCTET}\.{_OCTET}")
_PREFIX_RE = re.compile(r"[1-9]|[12][0-9]|3[0-2]")
_SENSOR_ID_RE = re.compile(r"[a-z][a-z0-9_]{0,30}")  # metrolla-edge deployment schema, sensors[].id
_FRAME_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]{0,63}")
_TOPIC_RE = re.compile(r"/?[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*")
_RUN_DIR_RE = re.compile(r"/[A-Za-z0-9_./-]*")


@dataclass
class Settings:
    model: Model
    sensor_ip: str
    sensor_id: str | None
    data_port: int
    device_port: int
    host_ip: str | None  # as given, maybe with /prefix
    frame_id: str
    topic: str
    time_mode: str
    run_dir: str
    sources: dict[str, str] = field(default_factory=dict)

    @property
    def namespace(self) -> str:
        return "/" + self.sensor_id if self.sensor_id else "/"

    @property
    def node_fqn(self) -> str:
        return (self.namespace + "/" + NODE_NAME) if self.sensor_id else "/" + NODE_NAME

    @property
    def config_path(self) -> str:
        return self.run_dir.rstrip("/") + "/config.yaml"


class Refused(Exception):
    def __init__(self, problems: list[str]):
        super().__init__("\n".join(problems))
        self.problems = problems


class BrokenImage(Exception):
    """The image's own files do not have the shape this script expects (a build problem)."""


def shown(value: str) -> str:
    text = value if len(value) <= 64 else value[:61] + "..."
    if text.isprintable() and text.isascii():
        return "'" + text + "'"
    return ascii(text)


def _ipv4(name: str, value: str, problems: list[str], what: str) -> ipaddress.IPv4Address | None:
    if not _IPV4_RE.fullmatch(value):
        problems.append(f"{name}={shown(value)} is not an IPv4 address; accepted: {what} in dotted "
                        "decimal without leading zeros, for example 192.168.1.201")
        return None
    address = ipaddress.IPv4Address(value)
    if address.is_unspecified or address.is_multicast or address.is_reserved or \
            address == ipaddress.IPv4Address("255.255.255.255"):
        problems.append(f"{name}={shown(value)} is not a unicast address; accepted: {what}, not "
                        "0.0.0.0, multicast (224-239.x.x.x), 240.x.x.x and above or broadcast")
        return None
    return address


def _port(name: str, value: str, problems: list[str], default_text: str) -> int | None:
    if not _PORT_RE.fullmatch(value) or not MIN_PORT <= int(value) <= MAX_PORT:
        problems.append(f"{name}={shown(value)} is not a usable port; accepted: a whole number "
                        f"{MIN_PORT}-{MAX_PORT} without leading zeros ({default_text})")
        return None
    return int(value)


def _bool(name: str, value: str, problems: list[str], default_text: str) -> bool | None:
    word = value.lower()
    if word in TRUE_WORDS:
        return True
    if word in FALSE_WORDS:
        return False
    problems.append(f"{name}={shown(value)} is not a boolean; accepted: {', '.join(TRUE_WORDS)} or "
                    f"{', '.join(FALSE_WORDS)}, in any case ({default_text})")
    return None


def parse(env: Mapping[str, str]) -> Settings:
    """Validate the environment. Raises Refused with one line per problem."""
    problems: list[str] = []
    sources: dict[str, str] = {}

    def get(name: str) -> str | None:
        value = env.get(name)
        sources[name] = "default" if value is None else "env"
        return value

    switch = env.get(CONTRACT_SWITCH)
    if switch not in CONTRACT_VERSIONS:
        problems.append(f"{CONTRACT_SWITCH}={shown(switch or '')} is not a contract version this "
                        f"image knows; accepted: {', '.join(CONTRACT_VERSIONS)} (or unset or 0 for "
                        "the legacy entrypoint)")

    for name in REQUIRED + OPTIONAL:  # an empty DUAL_LIDAR means false, as in the Seyond image
        if name in env and env[name] == "":
            problems.append(f"{name} is set but empty; accepted: {ACCEPTED[name]}"
                            + ("" if name in REQUIRED else ", or unset it for the default"))
    present = {k: v for k, v in env.items() if k in KNOWN and v != ""}

    # LIDAR_MODEL
    model = None
    if "LIDAR_MODEL" not in env:
        problems.append("LIDAR_MODEL is not set; accepted (any case, optional 'hesai-' prefix, '-' "
                        f"and '_' ignored): {model_list()}")
    elif "LIDAR_MODEL" in present:
        found = lookup_model(present["LIDAR_MODEL"])
        if isinstance(found, str):
            problems.append(f"LIDAR_MODEL={shown(present['LIDAR_MODEL'])} {found}")
        else:
            model = found
    get("LIDAR_MODEL")

    # SENSOR_IP
    sensor = None
    if "SENSOR_IP" not in env:
        problems.append("SENSOR_IP is not set; accepted: the sensor's own IPv4 address, for example "
                        "192.168.1.201 (the driver connects to it for calibration over PTC)")
    elif "SENSOR_IP" in present:
        sensor = _ipv4("SENSOR_IP", present["SENSOR_IP"], problems, "the sensor's own IPv4 address")
    get("SENSOR_IP")

    # SENSOR_ID
    sensor_id = None
    value = get("SENSOR_ID")
    if value:
        if not _SENSOR_ID_RE.fullmatch(value) or "__" in value:
            problems.append(f"SENSOR_ID={shown(value)} is not a sensor id; accepted: 1-31 characters, "
                            "a lowercase letter then lowercase letters, digits or single '_' (it "
                            "becomes the node's ROS namespace)")
        else:
            sensor_id = value

    # DATA_PORT / DEVICE_PORT
    data_port = DEFAULT_DATA_PORT
    value = get("DATA_PORT")
    if value:
        port = _port("DATA_PORT", value, problems, f"the UDP port the sensor sends to, default "
                     f"{DEFAULT_DATA_PORT}")
        data_port = port if port is not None else data_port
    device_port = DEFAULT_DEVICE_PORT
    value = get("DEVICE_PORT")
    if value:
        port = _port("DEVICE_PORT", value, problems, f"the sensor's PTC (TCP) port, default "
                     f"{DEFAULT_DEVICE_PORT}")
        device_port = port if port is not None else device_port

    # HOST_IP
    host_ip = None
    value = get("HOST_IP")
    if value:
        address, _, prefix = value.partition("/")
        if "/" in value and not _PREFIX_RE.fullmatch(prefix):
            problems.append(f"HOST_IP={shown(value)} has no usable prefix length; accepted: the "
                            "host's IPv4 address on the sensor network, optionally with /1-/32, for "
                            "example 192.168.1.100/24")
        else:
            host = _ipv4("HOST_IP", address, problems,
                         "the host's IPv4 address on the sensor network (optionally with /1-/32)")
            if host is not None:
                host_ip = value
                if sensor is not None and host == sensor:
                    problems.append(f"HOST_IP={shown(value)} is the sensor's own address; accepted: "
                                    "the host's address on the sensor network")
                    host_ip = None
                elif sensor is not None and prefix:
                    network = ipaddress.IPv4Interface(value).network
                    if sensor not in network:
                        problems.append(f"HOST_IP={shown(value)}: the sensor at {sensor} is not in "
                                        f"the host's subnet {network}; accepted: a host address in "
                                        "the sensor's subnet")
                        host_ip = None

    # FRAME_ID
    frame_id = DEFAULT_FRAME_ID
    value = get("FRAME_ID")
    if value:
        if not _FRAME_RE.fullmatch(value):
            problems.append(f"FRAME_ID={shown(value)} is not a frame id; accepted: 1-64 characters, "
                            "letters, digits and '_', not starting with a digit (default "
                            f"{DEFAULT_FRAME_ID})")
        else:
            frame_id = value

    # POINTCLOUD_TOPIC
    topic = DEFAULT_TOPIC
    value = get("POINTCLOUD_TOPIC")
    if value:
        if not _TOPIC_RE.fullmatch(value) or "__" in value or len(value) > 200:
            problems.append(f"POINTCLOUD_TOPIC={shown(value)} is not a ROS topic name; accepted: "
                            "'/'-separated tokens of letters, digits and single '_', each starting "
                            "with a letter, absolute (/x) or relative to the SENSOR_ID namespace "
                            f"(default {DEFAULT_TOPIC})")
        else:
            topic = value

    # TIME_MODE
    time_mode = DEFAULT_TIME_MODE
    value = get("TIME_MODE")
    if value:
        if value.lower() not in TIME_MODES:
            problems.append(f"TIME_MODE={shown(value)} is not a time mode; accepted, any case: host "
                            "(stamp with the host's receive time), sensor (the default), gps, ptp "
                            "or ntp (stamp with the sensor's own time; the sensor must be "
                            "synchronised that way, this image does not program it)")
        else:
            time_mode = value.lower()

    # PUBLISH_SCAN
    value = get("PUBLISH_SCAN")
    if value:
        flag = _bool("PUBLISH_SCAN", value, problems, "only a false value is accepted here")
        if flag:
            problems.append(f"PUBLISH_SCAN={shown(value)} is not supported by this image; accepted: "
                            "unset or false. The Hesai driver publishes PointCloud2 only")

    # HESAI_RUN_DIR
    run_dir = DEFAULT_RUN_DIR
    value = get("HESAI_RUN_DIR")
    if value:
        if not _RUN_DIR_RE.fullmatch(value) or "/../" in value + "/":
            problems.append(f"HESAI_RUN_DIR={shown(value)} is not an absolute directory path; "
                            "accepted: an absolute path of letters, digits, '_', '.', '-' and '/' "
                            f"(default {DEFAULT_RUN_DIR})")
        else:
            run_dir = value

    # DUAL_LIDAR
    value = env.get("DUAL_LIDAR")
    if value:
        flag = _bool("DUAL_LIDAR", value, problems, "only a false value is accepted here")
        if flag:
            problems.append(f"DUAL_LIDAR={shown(value)} is not supported with {CONTRACT_SWITCH}; "
                            "accepted: unset or false. Run one container per Hesai sensor")

    # Unknown variables under this image's prefixes, and the legacy entrypoint's variables.
    for name in sorted(env):
        if name in KNOWN:
            continue
        if name in LEGACY:
            instead = LEGACY[name]
            what = (f"use {instead}" if instead else
                    "the contract renders the configuration itself; remove it")
            problems.append(f"{name} is a legacy variable of this image and is not read with "
                            f"{CONTRACT_SWITCH}=1; {what}")
            continue
        if not name.startswith(STRICT_PREFIXES):
            continue
        close = [ALIASES[name]] if name in ALIASES else difflib.get_close_matches(name, KNOWN, n=1,
                                                                                    cutoff=0.6)
        hint = f" (did you mean {close[0]}?)" if close else ""
        problems.append(f"{name} is not a variable this image knows{hint}; accepted: "
                        f"{', '.join(KNOWN)}")

    if problems:
        raise Refused(problems)
    assert model is not None and sensor is not None
    return Settings(model=model, sensor_ip=str(sensor), sensor_id=sensor_id, data_port=data_port,
                    device_port=device_port, host_ip=host_ip, frame_id=frame_id, topic=topic,
                    time_mode=time_mode, run_dir=run_dir, sources=sources)


# --------------------------------------------------------------------------------------------
# Rendering: the per-model file, line by line, with the contract's keys replaced. Each replaced
# key must be found exactly once at its path, or the image is broken (selfcheck fails the build).
# --------------------------------------------------------------------------------------------

UDP = ("lidar", "driver", "lidar_udp_type")
DRIVER = ("lidar", "driver")
ROS = ("lidar", "ros")
PATH_KEYS = ("correction_file_path", "firetimes_path")
SIDE_TOPICS = ("ros_send_packet_topic", "ros_recv_packet_topic", "ros_send_imu_topic",
               "ros_send_packet_loss_topic")

_LINE_RE = re.compile(r"^(?P<indent>[ ]*)(?P<dash>- )?(?P<key>[A-Za-z_][A-Za-z0-9_]*):(?P<rest>.*)$")


def yaml_scalar(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, str):
        return json.dumps(value)  # a JSON string is a YAML double-quoted scalar
    raise TypeError(f"cannot render {value!r}")


def _scalar_text(rest: str) -> str:
    """The value part of 'key: value  # comment', without quotes."""
    text = rest.strip()
    if text.startswith('"'):
        end = text.find('"', 1)
        return text[1:end]
    return text.split(" #", 1)[0].strip()


def _walk(lines: list[str]):
    """Yield (index, path, indent_text, key, rest) for each 'key: ...' line."""
    stack: list[tuple[int, str]] = []
    items_at_top = 0
    for i, line in enumerate(lines):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        m = _LINE_RE.match(line)
        if not m:
            raise BrokenImage(f"line {i + 1} is not 'key: value': {line!r}")
        column = len(m["indent"]) + (2 if m["dash"] else 0)
        if m["dash"]:
            items_at_top += 1
            if items_at_top > 1:
                raise BrokenImage("more than one sensor in the file")
        while stack and stack[-1][0] >= column:
            stack.pop()
        path = tuple(k for _, k in stack) + (m["key"],)
        yield i, path, line[:column], m["key"], m["rest"]
        if not m["rest"].strip() or m["rest"].strip().startswith("#"):
            stack.append((column, m["key"]))


def read_model_config(model: Model, config_dir: str | None = None) -> str:
    path = os.path.join(config_dir or MODEL_CONFIG_DIR, model.config + ".yaml")
    with open(path, encoding="utf-8") as handle:
        return handle.read()


def sdk_path(value: str, sdk_dir: str | None = None) -> str:
    if value.startswith(CONFIG_SDK_PREFIX):
        return (sdk_dir or SDK_DIR).rstrip("/") + "/" + value[len(CONFIG_SDK_PREFIX):]
    return value


def _relative(topic: str) -> str:
    return topic.lstrip("/")


def render_config(s: Settings, source: str, sdk_dir: str | None = None) -> tuple[str, list[str]]:
    """Return the rendered YAML and the calibration files it names (to be checked)."""
    lines = source.split("\n")
    host = s.host_ip.partition("/")[0] if s.host_ip else None
    exact: dict[tuple[str, ...], object] = {
        DRIVER + ("source_type",): 1,
        UDP + ("device_ip_address",): s.sensor_ip,
        UDP + ("udp_port",): s.data_port,
        UDP + ("ptc_port",): s.device_port,
        DRIVER + ("use_timestamp_type",): TIME_MODES[s.time_mode],
        ROS + ("ros_frame_id",): s.frame_id,
        ROS + ("ros_send_point_cloud_topic",): s.topic,
        ROS + ("send_point_cloud_ros",): True,
    }
    if host:
        exact[UDP + ("host_ip_address",)] = host
    seen: dict[tuple[str, ...], int] = {}
    files: list[str] = []
    for i, path, indent, key, rest in list(_walk(lines)):
        if path in exact:
            seen[path] = seen.get(path, 0) + 1
            if path == DRIVER + ("source_type",):
                if _scalar_text(rest) != "1":
                    raise BrokenImage(f"{s.model.config}.yaml: source_type is {_scalar_text(rest)}, "
                                      "not 1 (UDP)")
                continue  # checked, kept as written
            if path == ROS + ("send_point_cloud_ros",):
                if _scalar_text(rest) != "true":
                    raise BrokenImage(f"{s.model.config}.yaml: send_point_cloud_ros is not true")
                continue
            lines[i] = f"{indent}{key}: {yaml_scalar(exact[path])}"
        elif key in PATH_KEYS:
            value = _scalar_text(rest)
            if value:
                real = sdk_path(value, sdk_dir)
                files.append(real)
                lines[i] = f"{indent}{key}: {yaml_scalar(real)}"
        elif path[:2] == ROS and key in SIDE_TOPICS:
            value = _scalar_text(rest)
            lines[i] = f"{indent}{key}: {yaml_scalar(_relative(value))}"
    missing = [".".join(p) for p in exact if seen.get(p, 0) != 1]
    if missing:
        raise BrokenImage(f"{s.model.config}.yaml: expected each of these exactly once: "
                          f"{', '.join(missing)}")
    header = [
        f"# Rendered from the environment by the {PROG} image ({CONTRACT_SWITCH}=1); do not edit.",
        f"# LIDAR_MODEL {s.model.label}: config/{s.model.config}.yaml with the contract's keys set.",
        f"# TIME_MODE {s.time_mode}: " + ("points stamped with the host's receive time "
                                          "(use_timestamp_type 1)" if s.time_mode == "host" else
                                          "points stamped with the sensor's own time "
                                          "(use_timestamp_type 0); the sensor must be synchronised "
                                          "that way, this image does not program it"),
    ]
    if s.host_ip:
        header.append(f"# HOST_IP {s.host_ip}: checked against SENSOR_IP; host_ip_address is used "
                      "by the driver only for SOME/IP, which is off")
    return "\n".join(header) + "\n" + "\n".join(lines), files


def render(s: Settings, config_dir: str | None = None, sdk_dir: str | None = None
           ) -> tuple[str, list[str]]:
    return render_config(s, read_model_config(s.model, config_dir), sdk_dir)


def node_argv(s: Settings) -> list[str]:
    argv = [DRIVER_NODE, "--ros-args"]
    if s.sensor_id:
        argv += ["-r", f"__ns:={s.namespace}"]
    argv += ["-p", f"config_path:={s.config_path}"]
    return argv


def missing_files(files: list[str]) -> list[str]:
    return [f for f in files if not (os.path.isfile(f) and os.access(f, os.R_OK))]


def describe(s: Settings, files: list[str]) -> dict:
    return {
        "model": {"label": s.model.label, "config": f"config/{s.model.config}.yaml"},
        "sensor_ip": s.sensor_ip,
        "data_port": s.data_port,
        "device_port": s.device_port,
        "host_ip": s.host_ip,
        "frame_id": s.frame_id,
        "pointcloud_topic": s.topic,
        "time_mode": s.time_mode,
        "use_timestamp_type": TIME_MODES[s.time_mode],
        "node": s.node_fqn,
        "config_path": s.config_path,
        "calibration_files": files,
        "argv": node_argv(s),
        "sources": dict(sorted(s.sources.items())),
    }


# --------------------------------------------------------------------------------------------
# Command line.
# --------------------------------------------------------------------------------------------


def refuse(problems: list[str], as_json: bool = False) -> int:
    if as_json:
        print(json.dumps({"refused": problems}, indent=2))
    for problem in problems:
        print(f"{PROG}: refusing to start: {problem}", file=sys.stderr)
    return EXIT_REFUSED


def broken(message: str) -> int:
    print(f"{PROG}: broken image: {message}", file=sys.stderr)
    return EXIT_REFUSED


def write_config(s: Settings, text: str) -> None:
    os.makedirs(s.run_dir, mode=0o700, exist_ok=True)
    tmp = s.config_path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        handle.write(text)
    os.replace(tmp, s.config_path)


def exec_env(env: Mapping[str, str]) -> dict[str, str]:
    out = dict(env)
    out.setdefault("ROS_HOME", DEFAULT_ROS_HOME)
    out.setdefault("ROS_LOG_DIR", out["ROS_HOME"].rstrip("/") + "/log")
    return out


def _missing_problem(missing: list[str], s: Settings) -> list[str]:
    return [f"the calibration file '{f}' named by config/{s.model.config}.yaml is not in "
            "this image; accepted: an image whose SDK has it (a build problem: rebuild with "
            "the SDK submodule checked out)" for f in missing]


def cmd_run(env: Mapping[str, str]) -> int:
    try:
        s = parse(env)
        text, files = render(s)
    except Refused as exc:
        return refuse(exc.problems)
    except (BrokenImage, OSError) as exc:
        return broken(str(exc))
    missing = missing_files(files)
    if missing:
        return refuse(_missing_problem(missing, s))
    child_env = exec_env(env)
    try:
        write_config(s, text)
        os.makedirs(child_env["ROS_LOG_DIR"], exist_ok=True)
    except OSError as exc:
        return refuse([f"cannot write {exc.filename or s.run_dir} ({exc.strerror}) as uid "
                       f"{os.getuid()}; accepted: a writable /tmp, for example docker run "
                       "--tmpfs /tmp or compose 'tmpfs: [/tmp]' with a read-only root filesystem "
                       "(or point HESAI_RUN_DIR and ROS_LOG_DIR at a writable volume)"])
    print(f"{PROG}: Hesai {s.model.label} at {s.sensor_ip}, data port {s.data_port}, PTC port "
          f"{s.device_port}, node {s.node_fqn}, topic {s.topic}, time {s.time_mode}, "
          f"configuration {s.config_path}", file=sys.stderr, flush=True)
    argv = node_argv(s)
    try:
        os.execve(argv[0], argv, child_env)
    except OSError as exc:
        print(f"{PROG}: cannot start the driver {argv[0]}: {exc.strerror} (a broken image)",
              file=sys.stderr)
    return EXIT_REFUSED


def cmd_check(env: Mapping[str, str], as_json: bool) -> int:
    try:
        s = parse(env)
        text, files = render(s)
    except Refused as exc:
        return refuse(exc.problems, as_json)
    except (BrokenImage, OSError) as exc:
        return broken(str(exc))
    missing = missing_files(files)
    if missing:
        return refuse(_missing_problem(missing, s), as_json)
    if as_json:
        out = describe(s, files)
        out["config_yaml"] = text
        print(json.dumps(out, indent=2))
    else:
        sys.stdout.write(text)
        print("# exec: " + " ".join(node_argv(s)))
    return EXIT_OK


def cmd_selfcheck(config_dir: str | None = None, sdk_dir: str | None = None,
                  node: str | None = None) -> int:
    """Build-time check: every model renders and every file it names is in the image."""
    problems = []
    for model in MODELS.values():
        env = {CONTRACT_SWITCH: "1", "LIDAR_MODEL": model.label, "SENSOR_IP": "192.168.1.201"}
        try:
            s = parse(env)
            _, files = render(s, config_dir, sdk_dir)
        except (Refused, BrokenImage, OSError) as exc:
            problems.append(f"{model.label}: {exc}")
            continue
        for f in missing_files(files):
            problems.append(f"{model.label}: calibration file missing: {f}")
        for f in files:
            # The contract runs as any uid (97000 in the product): every file must be world-readable.
            if os.path.isfile(f) and not os.stat(f).st_mode & 0o004:
                problems.append(f"{model.label}: calibration file not world-readable: {f}")
        print(f"{PROG}: selfcheck {model.label}: renders, {len(files)} calibration file(s) present")
    node = node or DRIVER_NODE
    if not os.access(node, os.X_OK):
        problems.append(f"driver binary missing or not executable: {node}")
    for p in problems:
        print(f"{PROG}: selfcheck FAILED: {p}", file=sys.stderr)
    return EXIT_REFUSED if problems else EXIT_OK


# The image's default CMD (legacy). With the contract switched on it means "run".
LEGACY_DEFAULT_CMD = ["ros2", "launch", "hesai_ros_driver", "start.py"]
USAGE = f"usage: {PROG} [run | check [--json] | selfcheck]  (with {CONTRACT_SWITCH}=1; see AGENTS.md)"


def main(argv: list[str], env: Mapping[str, str]) -> int:
    if not argv or argv == ["run"] or argv == LEGACY_DEFAULT_CMD:
        return cmd_run(env)
    if argv[0] == "check" and argv[1:] in ([], ["--json"]):
        return cmd_check(env, argv[1:] == ["--json"])
    if argv == ["selfcheck"]:
        return cmd_selfcheck()
    if argv[0] in ("-h", "--help", "help"):
        print(USAGE)
        return EXIT_OK
    print(f"{PROG}: refusing to start: unknown command {shown(' '.join(argv))} with "
          f"{CONTRACT_SWITCH}=1; accepted: run (the default), check, check --json. The contract "
          "renders the configuration from the environment; to run something else, unset "
          f"{CONTRACT_SWITCH} or override the entrypoint", file=sys.stderr)
    return EXIT_USAGE


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:], os.environ))
