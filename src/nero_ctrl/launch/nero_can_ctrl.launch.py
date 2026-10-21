from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    agx_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("agx_arm_ctrl"),
                "launch",
                "start_single_agx_arm.launch.py",
            )
        ),
        launch_arguments={
            "can_port": "can0",
            "arm_type": "nero",
            "effector_type": "agx_gripper",
            "tcp_offset": "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            "control_enabled": "true",
            "speed_percent": "50",
        }.items(),
    )

    return LaunchDescription([
        agx_arm_launch,

        Node(
            package="nero_ctrl",
            executable="main_node",
            name="main_node",
            output="screen",
        ),

        Node(
            package="nero_ctrl",
            executable="can_module",
            name="can_module",
            output="screen",
        ),
    ])
