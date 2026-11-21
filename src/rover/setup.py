from glob import glob
import os
from setuptools import setup

package_name = 'rover'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rithwik-Ravi',
    maintainer_email='mailrittyplay@gmail.com',
    description='Z.O.E. 18-DOF Hexapod ROS 2 navigation, SLAM, Gazebo simulation, and RL bridge',
    license='BSD-3-Clause',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            # ── Teleop & Calibration ───────────────────────────────────────
            'teleop = rover.teleop:main',
            'cal_servo = rover.test_esp:main',
            # ── Hexapod Locomotion & Hardware Bridges ──────────────────────
            'hxp_cmd = rover.hxp_command:main',
            'hxp_nav_ctrl = rover.hexapod_nav_controller:main',
            'hxp_ik = rover.hexapod_ik_controller:main',
            'rviz_ik = rover.rviz_hexa_ik:main',
            'tripod_gait = rover.tripod_gait:main',
            'imu_stabilizer = rover.imu_stabalizer:main',
            'esp_bdg = rover.esp:main',
            'odom_from_tf = rover.odom_from_tf:main',
            'rl_policy_bridge = rover.rl_policy_bridge:main',
            # ── Manipulation & Vision Utilities ────────────────────────────
            'armc = rover.arm_control:main',
            'arm_ik = rover.arm_ik_commander:main',
            'object_detector = rover.object_detector:main',
            'box_tracker = rover.box_tracker:main',
            # ── Autonomous AI Brain Stack ("Zoe") ──────────────────────────
            'sensor_bridge = rover.sensor_bridge_node:main',
            'tool_dispatcher = rover.tool_dispatcher_node:main',
            'nvidia_brain = rover.nvidia_brain_node:main',
            'voice_bridge = rover.voice_bridge_node:main',
        ],
    },
)
