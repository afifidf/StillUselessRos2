from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotis_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),

        # Config YAML files (camera params, dll)
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),

        # Config text & json files
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('robotis_manager', '*.txt'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('robotis_manager', '*.json'))),

        # Data files (HSV calibration .txt files)
        (os.path.join('share', package_name, 'data'),
         glob(os.path.join('robotis_manager', 'data', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='afifi',
    maintainer_email='adekmail31@gmail.com',
    description='Robotis OP3 Manager - ROS2 Port',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'main_node_handler  = robotis_manager.main_node_handler:main',
            'main_task_control  = robotis_manager.main_task_control:main',
            'main_motion_driver = robotis_manager.main_motion_driver:main',
            'main_head_tracking = robotis_manager.main_head_tracking:main',
            'vision_main        = robotis_manager.vision_main:main',
            'main_calibrate     = robotis_manager.main_calibrate:main',
            'action_bin_tool    = robotis_manager.action_bin_tool:main',
        ],
    },
)
