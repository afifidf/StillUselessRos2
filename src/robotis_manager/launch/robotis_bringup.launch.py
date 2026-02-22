#!/usr/bin/env python3
"""
robotis_bringup.launch.py

Launch pertama yang harus dijalankan di robot.
Menjalankan op3_manager (core ROBOTIS controller) dan kamera.

Cara pakai:
    ros2 launch robotis_manager robotis_bringup.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    camera_param_path = PathJoinSubstitution([
        FindPackageShare('robotis_manager'),
        'config',
        'camera_param.yaml'
    ])

    return LaunchDescription([

        # =============================================
        # OP3 Manager - Core ROBOTIS Controller
        # Load semua motion modules (walking, action, head_control, dll)
        # =============================================
        Node(
            package='op3_manager',
            executable='op3_manager',
            name='op3_manager',
            output='screen',
            parameters=[{
                'gazebo': False,
            }],
        ),

        # =============================================
        # USB Camera Node
        # Publish topic: /usb_cam_node/image_raw
        # =============================================
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[camera_param_path],
            remappings=[
                ('image_raw', 'usb_cam_node/image_raw'),
            ],
        ),

    ])
