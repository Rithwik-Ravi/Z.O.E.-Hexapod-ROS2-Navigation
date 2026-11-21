#!/usr/bin/env python3
"""
sim_compat.py — Cross-distro (ROS 2 Humble / Jazzy) compatibility and path helpers.

Provides:
  - Automatic detection of ROS_DISTRO ('humble' vs 'jazzy') so launch files
    and worlds run seamlessly on both Ignition Gazebo 6 (Fortress / Humble) and
    Gazebo Harmonic (Jazzy) without duplicating URDF/SDF assets.
  - Dynamic resolution of workspace `maps/` and `.brain_secrets` paths with
    zero hardcoded user home directories.
"""

import os
import site
from typing import Dict, List, Tuple

COMPAT_TMP_DIR = '/tmp/zoe_gz_compat'

HUMBLE_BT_PLUGINS = [
    'nav2_compute_path_to_pose_action_bt_node',
    'nav2_compute_path_through_poses_action_bt_node',
    'nav2_smooth_path_action_bt_node',
    'nav2_follow_path_action_bt_node',
    'nav2_spin_action_bt_node',
    'nav2_wait_action_bt_node',
    'nav2_assisted_teleop_action_bt_node',
    'nav2_back_up_action_bt_node',
    'nav2_drive_on_heading_bt_node',
    'nav2_clear_costmap_service_bt_node',
    'nav2_is_stuck_condition_bt_node',
    'nav2_goal_reached_condition_bt_node',
    'nav2_goal_updated_condition_bt_node',
    'nav2_globally_updated_goal_condition_bt_node',
    'nav2_is_path_valid_condition_bt_node',
    'nav2_initial_pose_received_condition_bt_node',
    'nav2_reinitialize_global_localization_service_bt_node',
    'nav2_rate_controller_bt_node',
    'nav2_distance_controller_bt_node',
    'nav2_speed_controller_bt_node',
    'nav2_truncate_path_action_bt_node',
    'nav2_truncate_path_local_action_bt_node',
    'nav2_goal_updater_node_bt_node',
    'nav2_recovery_node_bt_node',
    'nav2_pipeline_sequence_bt_node',
    'nav2_round_robin_node_bt_node',
    'nav2_transform_available_condition_bt_node',
    'nav2_time_expired_condition_bt_node',
    'nav2_path_expiring_timer_condition',
    'nav2_distance_traveled_condition_bt_node',
    'nav2_single_trigger_bt_node',
    'nav2_goal_updated_controller_bt_node',
    'nav2_is_battery_low_condition_bt_node',
    'nav2_navigate_through_poses_action_bt_node',
    'nav2_navigate_to_pose_action_bt_node',
    'nav2_remove_passed_goals_action_bt_node',
    'nav2_planner_selector_bt_node',
    'nav2_controller_selector_bt_node',
    'nav2_goal_checker_selector_bt_node',
    'nav2_controller_cancel_bt_node',
    'nav2_path_longer_on_approach_bt_node',
    'nav2_wait_cancel_bt_node',
    'nav2_spin_cancel_bt_node',
    'nav2_back_up_cancel_bt_node',
    'nav2_assisted_teleop_cancel_bt_node',
    'nav2_drive_on_heading_cancel_bt_node',
]


def get_ros_distro() -> str:
    """Return active ROS_DISTRO in lowercase (defaults to 'jazzy')."""
    return os.environ.get('ROS_DISTRO', 'jazzy').strip().lower()


def is_humble() -> bool:
    """Return True when running under ROS 2 Humble (Ignition Gazebo Fortress)."""
    return get_ros_distro() == 'humble'


def get_dynamic_pythonpath() -> str:
    """Build a PYTHONPATH including user site-packages without hardcoded user paths."""
    paths = [site.getusersitepackages()]
    existing = os.environ.get('PYTHONPATH', '')
    if existing:
        paths.append(existing)
    return ':'.join(p for p in paths if p)


def get_maps_dir() -> str:
    """
    Resolve the directory used for saving and loading SLAM OccupancyGrid maps.

    Search order:
      1. `ZOE_MAPS_DIR` environment variable (if set).
      2. Source workspace `<ws>/src/rover/maps` (detected from cwd or module path).
      3. Installed package share `<share>/rover/maps` or `~/zoe_maps` fallback.
    """
    env_dir = os.environ.get('ZOE_MAPS_DIR', '').strip()
    if env_dir:
        return env_dir

    # Walk up from cwd and this file to locate src/rover/maps in symlink-install setups
    probes = [os.getcwd(), os.path.dirname(os.path.abspath(__file__))]
    for start in probes:
        cur = os.path.abspath(start)
        for _ in range(6):
            candidate = os.path.join(cur, 'src', 'rover', 'maps')
            if os.path.isdir(candidate):
                return candidate
            direct_maps = os.path.join(cur, 'maps')
            if os.path.isfile(os.path.join(cur, 'package.xml')) and os.path.isdir(direct_maps):
                return direct_maps
            parent = os.path.dirname(cur)
            if parent == cur:
                break
            cur = parent

    try:
        from ament_index_python.packages import get_package_share_directory
        share_maps = os.path.join(get_package_share_directory('rover'), 'maps')
        if os.path.isdir(share_maps):
            return share_maps
    except Exception:
        pass

    return os.path.expanduser('~/zoe_maps')


def load_brain_secrets(logger=None) -> Dict[str, str]:
    """
    Load KEY=VALUE pairs from `.brain_secrets` without hardcoded home paths.

    Searches cwd, parent directories up to workspace root, and `~/.brain_secrets`.
    """
    candidates = [
        os.path.join(os.getcwd(), '.brain_secrets'),
        os.path.expanduser('~/.brain_secrets'),
    ]
    cur = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        candidates.append(os.path.join(cur, '.brain_secrets'))
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent

    seen = set()
    for path in candidates:
        norm = os.path.abspath(path)
        if norm in seen:
            continue
        seen.add(norm)
        if not os.path.isfile(norm):
            continue
        secrets: Dict[str, str] = {}
        try:
            with open(norm, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        k, v = line.split('=', 1)
                        secrets[k.strip()] = v.strip().strip('"\'')
            if secrets:
                if logger is not None:
                    logger.info(f'Loaded secrets from {norm}')
                return secrets
        except Exception as exc:
            if logger is not None:
                logger.warn(f'Could not read {norm}: {exc}')
    return {}


def get_bridge_args(include_sensors: bool = True) -> List[str]:
    """Return `ros_gz_bridge` topic remappings matching Jazzy (`gz.msgs`) or Humble (`ignition.msgs`)."""
    ns = 'ignition.msgs' if is_humble() else 'gz.msgs'
    args = [
        f'/clock@rosgraph_msgs/msg/Clock[{ns}.Clock',
        f'/scan@sensor_msgs/msg/LaserScan[{ns}.LaserScan',
        f'/odom@nav_msgs/msg/Odometry[{ns}.Odometry',
        f'/cmd_vel@geometry_msgs/msg/Twist]{ns}.Twist',
        f'/tf@tf2_msgs/msg/TFMessage[{ns}.Pose_V',
    ]
    if include_sensors:
        args.extend([
            f'/imu/data@sensor_msgs/msg/Imu[{ns}.IMU',
            f'/camera/camera/image_raw@sensor_msgs/msg/Image[{ns}.Image',
        ])
    return args


def _patch_gazebo_xml_for_humble(xml_text: str, is_world: bool = False) -> str:
    """Convert Gazebo Harmonic (`gz-sim`) plugin tags to Ignition Gazebo 6 (`ignition-gazebo`)."""
    replacements = [
        ('gz-sim-physics-system', 'ignition-gazebo-physics-system'),
        ('gz::sim::systems::Physics', 'ignition::gazebo::systems::Physics'),
        ('gz-sim-user-commands-system', 'ignition-gazebo-user-commands-system'),
        ('gz::sim::systems::UserCommands', 'ignition::gazebo::systems::UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'ignition-gazebo-scene-broadcaster-system'),
        ('gz::sim::systems::SceneBroadcaster', 'ignition::gazebo::systems::SceneBroadcaster'),
        ('gz-sim-diff-drive-system', 'ignition-gazebo-diff-drive-system'),
        ('gz::sim::systems::DiffDrive', 'ignition::gazebo::systems::DiffDrive'),
        ('gz-sim-sensors-system', 'ignition-gazebo-sensors-system'),
        ('gz::sim::systems::Sensors', 'ignition::gazebo::systems::Sensors'),
    ]
    out = xml_text
    for src, dst in replacements:
        out = out.replace(src, dst)

    if is_world and 'ignition::gazebo::systems::Imu' not in out:
        imu_plugin = (
            '    <plugin filename="ignition-gazebo-imu-system" '
            'name="ignition::gazebo::systems::Imu"/>\n'
        )
        out = out.replace('</world>', imu_plugin + '  </world>')

    if not is_world:
        # Ensure URDF camera and IMU sensors publish to standard ROS-bridged topics
        out = out.replace('<sensor name="camera" type="depth">', '<sensor name="camera" type="camera">\n    <topic>/camera/camera/image_raw</topic>')
        out = out.replace('<sensor name="imu_sensor" type="imu">', '<sensor name="imu_sensor" type="imu">\n    <topic>/imu/data</topic>')
    return out


def prepare_gazebo_assets(urdf_path: str, world_path: str) -> Tuple[str, str]:
    """
    Return `(urdf_path, world_path)` ready for the active Gazebo generation.

    On ROS 2 Jazzy (Gazebo Harmonic), returns the original paths.
    On ROS 2 Humble (Ignition Fortress 6), writes translated XML files to `/tmp/zoe_gz_compat/`.
    """
    if not is_humble():
        return urdf_path, world_path

    os.makedirs(COMPAT_TMP_DIR, exist_ok=True)
    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_xml = _patch_gazebo_xml_for_humble(f.read(), is_world=False)
    with open(world_path, 'r', encoding='utf-8') as f:
        world_xml = _patch_gazebo_xml_for_humble(f.read(), is_world=True)

    compat_urdf = os.path.join(COMPAT_TMP_DIR, 'hexa_humble.urdf')
    compat_world = os.path.join(COMPAT_TMP_DIR, 'sim_world_humble.sdf')
    with open(compat_urdf, 'w', encoding='utf-8') as f:
        f.write(urdf_xml)
    with open(compat_world, 'w', encoding='utf-8') as f:
        f.write(world_xml)
    return compat_urdf, compat_world


def prepare_nav2_params(params_path: str) -> str:
    """
    Return a Nav2 YAML path compatible with the active ROS_DISTRO.

    On ROS 2 Humble, injects `bt_navigator.ros__parameters.plugin_lib_names`
    (required by Humble's `nav2_bt_navigator` when Jazzy's `navigators` key is used)
    into a temporary YAML copy in `/tmp/zoe_gz_compat/`.
    """
    if not is_humble():
        return params_path

    import yaml

    os.makedirs(COMPAT_TMP_DIR, exist_ok=True)
    with open(params_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}

    bt_params = data.get('bt_navigator', {}).get('ros__parameters', {})
    if isinstance(bt_params, dict):
        bt_params.pop('navigators', None)
        bt_params.pop('navigate_to_pose', None)
        bt_params.pop('navigate_through_poses', None)
        if 'plugin_lib_names' not in bt_params:
            bt_params['plugin_lib_names'] = list(HUMBLE_BT_PLUGINS)

    # Humble Nav2 uses slash-style plugin IDs for planner_server and behavior_server
    planner_cfg = data.get('planner_server', {}).get('ros__parameters', {}).get('GridBased', {})
    if isinstance(planner_cfg, dict) and 'plugin' in planner_cfg:
        planner_cfg['plugin'] = planner_cfg['plugin'].replace('::', '/')

    behavior_params = data.get('behavior_server', {}).get('ros__parameters', {})
    if isinstance(behavior_params, dict):
        for bname in ('spin', 'backup', 'drive_on_heading', 'wait', 'assisted_teleop'):
            bcfg = behavior_params.get(bname)
            if isinstance(bcfg, dict) and 'plugin' in bcfg:
                bcfg['plugin'] = bcfg['plugin'].replace('::', '/')

    out_path = os.path.join(COMPAT_TMP_DIR, f'humble_{os.path.basename(params_path)}')
    with open(out_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return out_path
