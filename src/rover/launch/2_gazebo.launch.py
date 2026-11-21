"""
2_gazebo.launch.py — Dual ROS 2 Humble / Jazzy Gazebo Launch for Z.O.E. Hexapod.

Launches:
  1. GZ_SIM_RESOURCE_PATH / IGN_GAZEBO_RESOURCE_PATH for STL mesh resolution
  2. Robot State Publisher with URDF description
  3. Gazebo Sim (Harmonic on Jazzy, Ignition Fortress 6 on Humble) with sim_world.sdf
  4. Robot spawner via ros_gz_sim create
  5. ros_gz_bridge for /clock, /scan, /odom, /cmd_vel, /tf, /imu/data, /camera/camera/image_raw
  6. RViz2 with view_robot.rviz
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
    """Build launch description for Gazebo simulation and ROS-Gazebo bridge."""
    pkg_dir = get_package_share_directory('rover')
    models_dir = os.path.dirname(pkg_dir)
    urdf_raw = os.path.join(pkg_dir, 'urdf', 'hexa.urdf')
    world_raw = os.path.join(pkg_dir, 'worlds', 'sim_world.sdf')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'view_robot.rviz')

    urdf_path, world_path = prepare_gazebo_assets(urdf_raw, world_raw)

    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = f'{models_dir}:{existing_gz_path}' if existing_gz_path else models_dir
    gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=new_gz_path)

    existing_ign_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    new_ign_path = f'{models_dir}:{existing_ign_path}' if existing_ign_path else models_dir
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=new_ign_path)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    open_rviz = LaunchConfiguration('open_rviz', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock from Gazebo',
    )
    declare_open_rviz = DeclareLaunchArgument(
        'open_rviz', default_value='true',
        description='Open RViz2 for visualization',
    )

    with open(urdf_path, 'r', encoding='utf-8') as f:
        urdf_content = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_content, 'use_sim_time': use_sim_time}],
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(open_rviz),
    )

    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        declare_use_sim_time,
        declare_open_rviz,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge,
        rviz_node,
    ])
