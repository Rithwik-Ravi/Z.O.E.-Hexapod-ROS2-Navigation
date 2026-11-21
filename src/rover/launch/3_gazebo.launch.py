"""
3_gazebo.launch.py — Dual ROS 2 Humble / Jazzy Gazebo + RViz2 Launch for Z.O.E. Hexapod.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover.sim_compat import get_bridge_args, prepare_gazebo_assets


def generate_launch_description():
    """Launch Gazebo simulation, robot_state_publisher, bridge, and RViz2."""
    pkg_dir = get_package_share_directory('rover')
    models_dir = os.path.dirname(pkg_dir)
    urdf_raw = os.path.join(pkg_dir, 'urdf', 'hexa.urdf')
    world_raw = os.path.join(pkg_dir, 'worlds', 'sim_world.sdf')
    rviz_config = os.path.join(pkg_dir, 'config', 'view_robot.rviz')

    urdf_path, world_path = prepare_gazebo_assets(urdf_raw, world_raw)

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, ':', os.environ.get('GZ_SIM_RESOURCE_PATH', '')],
    )
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[models_dir, ':', os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')],
    )

    open_rviz = LaunchConfiguration('open_rviz', default='true')
    declare_open_rviz = DeclareLaunchArgument(
        'open_rviz', default_value='true', description='Launch RViz2'
    )

    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content, 'use_sim_time': True}],
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
        parameters=[{'use_sim_time': True}],
        arguments=get_bridge_args(include_sensors=True),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        declare_open_rviz,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge,
        rviz_node,
    ])
