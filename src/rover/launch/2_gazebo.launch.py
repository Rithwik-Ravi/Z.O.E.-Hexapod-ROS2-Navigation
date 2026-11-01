"""
2_gazebo.launch.py — Gazebo Sim (Harmonic) Launch for Hexapod Robot in ROS 2 Jazzy

Replaces obsolete Gazebo Classic (gazebo_ros) with Gazebo Sim (ros_gz_sim & ros_gz_bridge).
Launches:
  1. Sets GZ_SIM_RESOURCE_PATH for STL mesh resolution
  2. Robot State Publisher with URDF description
  3. Gazebo Sim with sim_world.sdf environment
  4. Robot spawner via ros_gz_sim create
  5. ros_gz_bridge for /clock, /scan, /odom, /cmd_vel, /tf
  6. RViz2 with view_robot.rviz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover')
    models_dir = os.path.dirname(pkg_dir)
    urdf_path = os.path.join(pkg_dir, 'urdf', 'hexa.urdf')
    default_world = os.path.join(pkg_dir, 'worlds', 'sim_world.sdf')
    rviz_config_path = os.path.join(pkg_dir, 'config', 'view_robot.rviz')

    # Environment: Set GZ_SIM_RESOURCE_PATH so model://rover/meshes/*.stl and package:// resolve
    existing_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_resource_path = (
        f"{models_dir}:{existing_resource_path}"
        if existing_resource_path
        else models_dir
    )
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_resource_path
    )

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_path = LaunchConfiguration('world', default=default_world)
    open_rviz = LaunchConfiguration('open_rviz', default='true')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock from Gazebo'
    )
    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Full path to Gazebo SDF world file'
    )
    declare_open_rviz = DeclareLaunchArgument(
        'open_rviz', default_value='true',
        description='Open RViz2 for visualization'
    )

    # 1. Robot Description (URDF) & Robot State Publisher
    with open(urdf_path, 'r') as f:
        urdf_content = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'use_sim_time': use_sim_time
        }]
    )

    # 2. Gazebo Sim Launch (Harmonic)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items()
    )

    # 3. Spawn Robot in Gazebo Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'rover',
            '-file', urdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.25'
        ]
    )

    # 4. ROS-Gazebo Bridge (Clock, Scan, Odom, CmdVel, TF)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ]
    )

    # 5. RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(open_rviz)
    )

    return LaunchDescription([
        gz_resource_path,
        declare_use_sim_time,
        declare_world,
        declare_open_rviz,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        bridge,
        rviz_node,
    ])