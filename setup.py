
import os
from glob import glob
from setuptools import setup

package_name = 'areal_landing_px4_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Garlow',
    maintainer_email='adamgarlow@icloud.com',
    description='Implements communication between the ROS2 companion computer' \
        ' and PX4 FMU',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_control_command_srv = areal_landing_px4_communication.service_px4_control_commands:main',  
            'px4_pos_set_move_act = areal_landing_px4_communication.action_px4_position_setpoint_move:main',
            'px4_mocap_pubsub = areal_landing_px4_communication.px4_vicon_pubsub:main'
        ],
    },
)
