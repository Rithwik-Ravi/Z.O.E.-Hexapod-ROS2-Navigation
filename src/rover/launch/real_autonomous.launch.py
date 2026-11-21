#!/usr/bin/env python3
"""
real_autonomous.launch.py — Full Autonomous Brain Stack on Real Hardware

Combines the complete real_navigation.launch.py stack (RPLidar A1, ESP32,
SLAM Toolbox, Nav2) with the AI brain nodes:
  - sensor_bridge_node   (t=25s — after SLAM + Nav2 fully warm up on real HW)
  - tool_dispatcher_node (t=26s)
  - nvidia_brain_node    (t=27s — after services are ready)
  - voice_bridge_node    (t=28s, optional)

API key is auto-loaded from <workspace_root>/.brain_secrets — no manual
export needed on the robot.

Usage (on RPi5 / Jetson — just source and launch):
  cd ~/gz_hex
  source install/setup.bash
  ros2 launch rover real_autonomous.launch.py

Optional overrides:
  ros2 launch rover real_autonomous.launch.py \
      lidar_port:=/dev/ttyUSB0 \
      esp_port:=/dev/ttyUSB1 \
      model:=meta/llama-3.1-70b-instruct \
      enable_voice:=true \
      open_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover.sim_compat import get_dynamic_pythonpath, load_brain_secrets


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover')
    brain_config = os.path.join(pkg_dir, 'config', 'brain_config.yaml')

    # ── Load secrets at launch-parse time ─────────────────────────────────────
    secrets = load_brain_secrets()
    nvidia_api_key = secrets.get('NVIDIA_API_KEY', os.environ.get('NVIDIA_API_KEY', ''))

    # ── Launch Arguments ───────────────────────────────────────────────────────
    declare_model = DeclareLaunchArgument(
        'model', default_value='meta/llama-3.2-11b-vision-instruct',
        description='NVIDIA NIM model name')

    declare_enable_voice = DeclareLaunchArgument(
        'enable_voice', default_value='false',
        description='Enable microphone voice listener node (speech-to-text)')

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port', default_value='',
        description='Override lidar serial port (empty = auto-detect)')

    declare_esp_port = DeclareLaunchArgument(
        'esp_port', default_value='',
        description='Override ESP32 serial port (empty = auto-detect)')

    declare_open_rviz = DeclareLaunchArgument(
        'open_rviz', default_value='true',
        description='Launch RViz2 for visualization')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_real_params.yaml'),
        description='Nav2 parameters file')

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'slam_toolbox_real_params.yaml'),
        description='SLAM Toolbox parameters file')

    # ── LaunchConfigurations ───────────────────────────────────────────────────
    model        = LaunchConfiguration('model')
    enable_voice = LaunchConfiguration('enable_voice')
    lidar_port   = LaunchConfiguration('lidar_port')
    esp_port     = LaunchConfiguration('esp_port')
    open_rviz    = LaunchConfiguration('open_rviz')
    params_file  = LaunchConfiguration('params_file')
    slam_params  = LaunchConfiguration('slam_params_file')

    # ── Include the full real hardware navigation stack ────────────────────────
    real_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'real_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time':     'false',
            'open_rviz':        open_rviz,
            'params_file':      params_file,
            'slam_params_file': slam_params,
            'lidar_port':       lidar_port,
            'esp_port':         esp_port,
        }.items(),
    )

    # ── Python-path so openai package resolves inside the ROS2 launch env ─────
    pythonpath_val = get_dynamic_pythonpath()

    # ── Environment injected into all brain nodes ──────────────────────────────
    brain_env = {
        'PYTHONPATH': pythonpath_val,
        'NVIDIA_API_KEY': nvidia_api_key,     # auto-loaded from .brain_secrets
    }

    # ── Sensor Bridge Node ─────────────────────────────────────────────────────
    sensor_bridge = Node(
        package='rover',
        executable='sensor_bridge',
        name='sensor_bridge_node',
        output='screen',
        parameters=[brain_config, {'use_sim_time': False}],
    )

    # ── Tool Dispatcher Node ───────────────────────────────────────────────────
    tool_dispatcher = Node(
        package='rover',
        executable='tool_dispatcher',
        name='tool_dispatcher_node',
        output='screen',
        parameters=[brain_config, {'use_sim_time': False}],
    )

    # ── NVIDIA Brain Node ──────────────────────────────────────────────────────
    brain_node = Node(
        package='rover',
        executable='nvidia_brain',
        name='nvidia_brain_node',
        output='screen',
        parameters=[
            brain_config,
            {
                'use_sim_time': False,
                'model': model,
            }
        ],
        additional_env=brain_env,
    )

    # ── Voice Bridge Node (optional) ───────────────────────────────────────────
    voice_node = Node(
        package='rover',
        executable='voice_bridge',
        name='voice_bridge_node',
        output='screen',
        condition=IfCondition(enable_voice),
        additional_env=brain_env,
    )

    return LaunchDescription([
        # Args
        declare_model,
        declare_enable_voice,
        declare_lidar_port,
        declare_esp_port,
        declare_open_rviz,
        declare_params_file,
        declare_slam_params_file,

        # Real hardware nav stack (Lidar, ESP32, SLAM, Nav2, RViz)
        real_nav,

        # Brain stack — staggered to let SLAM + Nav2 warm up first
        # Nav2 lifecycle manager starts at t=8s inside real_navigation;
        # wait 17 more seconds for all bonds to settle before brain connects.
        TimerAction(period=25.0, actions=[sensor_bridge]),
        TimerAction(period=26.0, actions=[tool_dispatcher]),
        TimerAction(period=27.0, actions=[brain_node]),
        TimerAction(period=28.0, actions=[voice_node]),
    ])
