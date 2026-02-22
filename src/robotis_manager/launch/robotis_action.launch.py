#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Vision Image Handler Node
        Node(
            package='robotis_manager',
            executable='main_node_handler',
            name='vision_image_handler',
            output='screen',
        ),

        # Motion Driver Node
        Node(
            package='robotis_manager',
            executable='main_motion_driver',
            name='motion_motion_driver',
            output='screen',
        ),

        # Head Tracking Node
        Node(
            package='robotis_manager',
            executable='main_head_tracking',
            name='motion_head_tracking',
            output='screen',
        ),

        # Task Control Node
        Node(
            package='robotis_manager',
            executable='main_task_control',
            name='motion_task',
            output='screen',
        ),

    ])
