#!/usr/bin/env python3
"""
robotis_action_editor.launch.py

Launch untuk mengedit gerakan/action robot menggunakan op3_action_editor.
HARUS dijalankan dengan robot terhubung via USB.

Cara pakai:
    ros2 launch robotis_manager robotis_action_editor.launch.py

Atau pakai cara lama:
    ros2 run op3_action_editor executor.py

Page number action yang tersedia (motion_4095.bin):
    1   = standup
    9   = walkready
    81  = f_get_up (getup dari depan)
    82  = b_get_up (getup dari belakang)
    83  = r_kick (tendang kanan)
    84  = l_kick (tendang kiri)
    122 = f_get_up (versi baru)
    123 = b_get_up (versi baru)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # =============================================
        # OP3 Manager - Core ROBOTIS Controller
        # Harus jalan dulu sebelum action editor
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
        # OP3 Action Editor
        # UI terminal untuk edit gerakan robot
        # =============================================
        Node(
            package='op3_action_editor',
            executable='op3_action_editor',
            name='op3_action_editor',
            output='screen',
            prefix='xterm -e',  # buka di terminal baru
        ),

    ])
