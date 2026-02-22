#!/usr/bin/env python3
"""
robotis_main.launch.py

Launch UTAMA - membuat robot bisa:
- Berjalan otomatis
- Mendeteksi bola
- Menendang bola
- Bangun sendiri dari jatuh
- Head tracking ke bola

Cara pakai:
    # Terminal 1 - jalankan bringup dulu
    ros2 launch robotis_manager robotis_bringup.launch.py

    # Terminal 2 - jalankan program utama
    ros2 launch robotis_manager robotis_main.launch.py

Topik penting:
    /usb_cam_node/image_raw     : input kamera
    /robotis/ball_position      : posisi bola (dari vision)
    /robotis/walking/command    : perintah jalan
    /robotis/action/page_num    : nomor action (kick, getup, dll)
    /robotis/head_control/...   : kontrol kepala
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # =============================================
        # Vision Node
        # Subscribe kamera, deteksi bola, publish posisi bola
        # Topic output: /robotis/ball_position
        # =============================================
        Node(
            package='robotis_manager',
            executable='main_node_handler',
            name='vision_image_handler',
            output='screen',
        ),

        # =============================================
        # Motion Driver Node
        # Terima posisi bola, kirim perintah walking & action
        # Topic input : /robotis/ball_position
        # Topic output: /robotis/walking/command, /robotis/action/page_num
        # =============================================
        Node(
            package='robotis_manager',
            executable='main_motion_driver',
            name='motion_driver',
            output='screen',
        ),

        # =============================================
        # Head Tracking Node
        # Gerakkan kepala robot mengikuti bola
        # Topic input : /robotis/ball_position
        # Topic output: /robotis/head_control/set_joint_states
        # =============================================
        Node(
            package='robotis_manager',
            executable='main_head_tracking',
            name='head_tracking',
            output='screen',
        ),

        # =============================================
        # Task Control Node
        # State machine utama robot:
        # FIND_BALL -> APPROACH -> KICK -> GETUP
        # =============================================
        Node(
            package='robotis_manager',
            executable='main_task_control',
            name='task_control',
            output='screen',
        ),

    ])
