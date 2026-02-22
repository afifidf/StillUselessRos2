#!/usr/bin/env python3
"""
robotis_calibrate.launch.py

Launch untuk kalibrasi warna bola dan lapangan.
Tidak butuh robot hardware - bisa dijalankan di laptop.

Cara pakai:
    ros2 launch robotis_manager robotis_calibrate.launch.py

Window yang akan muncul:
    - frame_ball  : kamera utama + titik bola terdeteksi
    - field_mask  : mask lapangan hijau
    - ball_mask   : mask bola orange
    - hsv         : frame HSV
    - ball_frame  : area dalam lapangan saja
    - Calibrate   : trackbar kalibrasi (geser untuk ubah nilai HSV)

Hasil kalibrasi otomatis tersimpan ke:
    install/robotis_manager/share/robotis_manager/data/
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

        # =============================================
        # Vision Node
        # Subscribe kamera, tampilkan frame + mask overlay
        # =============================================
        Node(
            package='robotis_manager',
            executable='main_node_handler',
            name='vision_image_handler',
            output='screen',
        ),

        # =============================================
        # Calibrate Node
        # Trackbar UI untuk kalibrasi HSV lapangan dan bola
        # =============================================
        Node(
            package='robotis_manager',
            executable='main_calibrate',
            name='vision_calibrate',
            output='screen',
        ),

    ])
