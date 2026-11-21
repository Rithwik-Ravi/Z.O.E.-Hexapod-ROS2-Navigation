"""
sim_navigation.launch.py — Dual ROS 2 Humble / Jazzy Gazebo SLAM & Nav2 Launch.

Features:
  1. Sets GZ_SIM_RESOURCE_PATH & IGN_GAZEBO_RESOURCE_PATH for STL mesh loading
  2. Starts Gazebo Sim (Harmonic on Jazzy, Ignition Fortress 6 on Humble)
  3. Spawns `hexa.urdf` robot model and launches `robot_state_publisher`
  4. Bridges /clock, /scan, /odom, /cmd_vel, /tf, /imu/data, /camera/camera/image_raw
  5. Launches `hxp_cmd` hexapod IK locomotion controller
  6. Starts SLAM Toolbox online async mapping
  7. Autostarts Nav2 lifecycle manager at t=15s
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

from rover.sim_compat import get_bridge_args, prepare_gazebo_assets, prepare_nav2_params


def generate_launch_description():
    """Build full simulation SLAM + Nav2 launch description."""
    pkg_dir = get_package_share_directory('rover')
    models_dir = os.path.dirname(pkg_dir)
    nav2_params_raw = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    urdf_raw = os.path.join(pkg_dir, 'urdf', 'hexa.urdf')
    world_raw = os.path.join(pkg_dir, 'worlds', 'sim_world.sdf')

    urdf_path, world_path = prepare_gazebo_assets(urdf_raw, world_raw)
    nav2_params = prepare_nav2_params(nav2_params_raw)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file = LaunchConfiguration('params_file', default=nav2_params)
    open_rviz = LaunchConfiguration('open_rviz', default='true')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')],
    )
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[models_dir, ':', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')],
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock for Gazebo navigation',
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=nav2_params,
        description='Nav2 params YAML file',
    )
    declare_open_rviz = DeclareLaunchArgument(
        'open_rviz', default_value='true',
        description='Launch RViz2 for visualization',
    )

    with open(urdf_path, 'r', encoding='utf-8') as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'rover', '-file', urdf_path, '-x', '0.0', '-y', '0.0', '-z', '0.25'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=get_bridge_args(include_sensors=True),
    )

    hxp_ik_node = Node(
        package='rover',
        executable='hxp_cmd',
        name='ik_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    nav2_nodes = GroupAction(actions=[
        SetParameter('use_sim_time', use_sim_time),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
        ),
    ])

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml'),
        }.items(),
    )

    configure_nav2 = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                    'bond_timeout': 4.0,
                    'use_sim_time': use_sim_time,
                }],
            )
        ],
    )

    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav2.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_use_sim_time,
        declare_params_file,
        declare_open_rviz,
        gz_sim,
        spawn_robot,
        robot_state_publisher,
        bridge,
        hxp_ik_node,
        mapping,
        nav2_nodes,
        configure_nav2,
        rviz_node,
    ])
