#!/usr/bin/env python3

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

        # USB Camera Node
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
