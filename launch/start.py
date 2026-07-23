"""Parameterized launch for the Hesai ROS2 driver.

The driver runs as a single node that reads one config file whose top-level
``lidar:`` list may contain one or more sensors (see NodeManager::Init). This
launch file lets you pick which config file to load and override individual
per-lidar fields at launch time, driven by either environment variables or
inline ``ros2 launch key:=value`` arguments -- so the driver can be configured
"like a container" with no YAML editing and no C++ changes.

Config selection (first match wins):
  * ``config_path:=<abs>``  / env ``HESAI_CONFIG_PATH``  -- a full custom config
    file (e.g. volume-mounted). Used as the base as-is.
  * ``config_file:=<name>`` / env ``HESAI_CONFIG_FILE``  -- a basename resolved
    inside the package's installed ``config/`` dir. Default ``config.yaml``.
  * env ``LIDAR_MODEL``  -- selects ``<MODEL>.yaml`` as the base when neither of
    the above is set (backward compatible with the old entrypoint.sh).

Per-lidar overrides (``i`` = 0-based index into the ``lidar:`` list):
  * inline: ``lidar0_device_ip:=192.168.1.201 lidar1_udp_port:=2369``
  * env:    ``HESAI_LIDAR0_DEVICE_IP=192.168.1.201 HESAI_LIDAR1_UDP_PORT=2369``
  Inline args take precedence over env vars. Indices >= 2 are supported through
  the environment scan even though only lidar0_*/lidar1_* are declared for
  ``--show-args`` documentation.

Backward-compatible aliases (applied to lidar0): ``SENSOR_IP``/``DEVICE_IP`` ->
device_ip, ``SENSOR_PORT``/``UDP_PORT`` -> udp_port, ``HOST_IP`` -> host_ip.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = 'hesai_ros_driver'

# override field suffix -> (path within one lidar list entry, caster)
FIELD_MAP = {
    'device_ip':            (['driver', 'lidar_udp_type', 'device_ip_address'], str),
    'udp_port':             (['driver', 'lidar_udp_type', 'udp_port'], int),
    'ptc_port':             (['driver', 'lidar_udp_type', 'ptc_port'], int),
    'host_ip':              (['driver', 'lidar_udp_type', 'host_ip_address'], str),
    'correction_file':      (['driver', 'lidar_udp_type', 'correction_file_path'], str),
    'firetimes_file':       (['driver', 'lidar_udp_type', 'firetimes_path'], str),
    'source_type':          (['driver', 'source_type'], int),
    'frame_id':             (['ros', 'ros_frame_id'], str),
    'point_topic':          (['ros', 'ros_send_point_cloud_topic'], str),
    'packet_topic':         (['ros', 'ros_send_packet_topic'], str),
    'imu_topic':            (['ros', 'ros_send_imu_topic'], str),
    'packet_loss_topic':    (['ros', 'ros_send_packet_loss_topic'], str),
    'send_point_cloud_ros': (['ros', 'send_point_cloud_ros'], lambda v: str(v).lower() in ('1', 'true', 'yes', 'on')),
    'send_packet_ros':      (['ros', 'send_packet_ros'], lambda v: str(v).lower() in ('1', 'true', 'yes', 'on')),
    'send_imu_ros':         (['ros', 'send_imu_ros'], lambda v: str(v).lower() in ('1', 'true', 'yes', 'on')),
}

# legacy single-lidar env names -> lidar0 field suffix
LEGACY_ALIASES = {
    'device_ip': ('SENSOR_IP', 'DEVICE_IP'),
    'udp_port': ('SENSOR_PORT', 'UDP_PORT'),
    'host_ip': ('HOST_IP',),
}


def _resolve_base_config(context, config_dir):
    """Pick the base config file path from args/env (first match wins)."""
    explicit_path = LaunchConfiguration('config_path').perform(context).strip()
    if explicit_path:
        return explicit_path

    config_file = LaunchConfiguration('config_file').perform(context).strip()
    if not config_file:
        model = os.environ.get('LIDAR_MODEL', '').strip()
        candidate = os.path.join(config_dir, '{}.yaml'.format(model)) if model else ''
        if model and os.path.isfile(candidate):
            return candidate
        config_file = 'config.yaml'

    return os.path.join(config_dir, config_file)


def _override_value(context, index, suffix):
    """Resolve one override: inline launch arg first, then env, then legacy alias."""
    arg = context.launch_configurations.get('lidar{}_{}'.format(index, suffix))
    if arg is not None and arg != '':
        return arg

    env_val = os.environ.get('HESAI_LIDAR{}_{}'.format(index, suffix.upper()), '')
    if env_val != '':
        return env_val

    if index == 0:
        for legacy in LEGACY_ALIASES.get(suffix, ()):  # lidar0 legacy aliases
            val = os.environ.get(legacy, '')
            if val != '':
                return val
    return None


def _apply(entry, path, value):
    node = entry
    for key in path[:-1]:
        node = node.setdefault(key, {})
    node[path[-1]] = value


def _launch_setup(context, *args, **kwargs):
    config_dir = os.path.join(get_package_share_directory(PACKAGE), 'config')
    base_config = _resolve_base_config(context, config_dir)

    with open(base_config, 'r') as fh:
        config = yaml.safe_load(fh)

    lidars = config.get('lidar', []) if isinstance(config, dict) else []
    changed = False
    for i, entry in enumerate(lidars):
        for suffix, (path, caster) in FIELD_MAP.items():
            raw = _override_value(context, i, suffix)
            if raw is None:
                continue
            _apply(entry, path, caster(raw))
            changed = True
            print('[hesai launch] lidar{} {} = {}'.format(i, '.'.join(path), raw))

    if changed:
        rendered = os.environ.get(
            'HESAI_RENDERED_CONFIG',
            os.path.join(tempfile.gettempdir(), 'hesai_rendered_config.yaml'),
        )
        with open(rendered, 'w') as fh:
            yaml.safe_dump(config, fh, sort_keys=False, default_flow_style=False)
        config_path = rendered
        print('[hesai launch] rendered config -> {}'.format(rendered))
    else:
        config_path = base_config
    print('[hesai launch] base config: {}'.format(base_config))

    namespace = LaunchConfiguration('namespace').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)

    driver_node = Node(
        namespace=namespace,
        package=PACKAGE,
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': config_path}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    rviz_config = os.path.join(get_package_share_directory(PACKAGE), 'rviz', 'rviz2.rviz')
    rviz_node = Node(
        namespace='rviz2',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [driver_node, rviz_node]


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'config_file', default_value=EnvironmentVariable('HESAI_CONFIG_FILE', default_value=''),
            description='Base config basename resolved in the package config/ dir (default config.yaml).'),
        DeclareLaunchArgument(
            'config_path', default_value=EnvironmentVariable('HESAI_CONFIG_PATH', default_value=''),
            description='Absolute path to a full custom config file. Overrides config_file.'),
        DeclareLaunchArgument(
            'namespace', default_value=EnvironmentVariable('HESAI_NAMESPACE', default_value='hesai_ros_driver'),
            description='ROS namespace for the driver node.'),
        DeclareLaunchArgument(
            'log_level', default_value=EnvironmentVariable('HESAI_LOG_LEVEL', default_value='info'),
            description='ROS logger level (debug|info|warn|error|fatal).'),
        DeclareLaunchArgument(
            'rviz', default_value=EnvironmentVariable('HESAI_RVIZ', default_value='false'),
            description='Launch rviz2 with the packaged config.'),
    ]

    # Declared for --show-args and clean CLI use; indices >=2 work via env scan.
    for i in (0, 1):
        for suffix in FIELD_MAP:
            args.append(DeclareLaunchArgument(
                'lidar{}_{}'.format(i, suffix), default_value='',
                description='Override {} for lidar {} (empty = keep config value).'.format(suffix, i)))

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
