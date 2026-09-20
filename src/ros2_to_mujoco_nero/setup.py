import os
from glob import glob

from setuptools import setup

package_name = 'ros2_to_mujoco_nero'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'mujoco_node = ros2_to_mujoco_nero.mujoco_node:main',
            'main_node = ros2_to_mujoco_nero.main_node:main',
            'inverse_kinematics_node = ros2_to_mujoco_nero.inverse_kinematics_node:main',
            'inverse_kinematics_body_jacobian_node = ros2_to_mujoco_nero.inverse_kinematics_body_jacobian_node:main',
        ],
    },
)