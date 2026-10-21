from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import os


def generate_launch_description():
    can_activate = ExecuteProcess(
        cmd=[
        "bash",
        "/home/ubuntu/Projects/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh",
        "can0",
        "1000000",
        ],
        output="screen",
    )

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
            "speed_percent": "10",
        }.items(),
    )

    main_node = Node(
            package="nero_ctrl",
            executable="main_node",
            name="main_node",
            output="screen",
    )

    can_module = Node(
            package="nero_ctrl",
            executable="can_module",
            name="can_module",
            output="screen",
    )

    return LaunchDescription([
        can_activate,
        RegisterEventHandler(
            OnProcessExit(
                target_action=can_activate,
                on_exit=[agx_arm_launch, main_node, can_module],
            )
        ),
    ])
