from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_to_mujoco_nero',
            executable='main_node',
            name='main_node',
            output='screen',
        ),
        Node(
            package='ros2_to_mujoco_nero',
            executable='mujoco_node',
            name='mujoco_node',
            output='screen',
        ),
        Node(
            package='ros2_to_mujoco_nero',
            executable='inverse_kinematics_node',
            name='inverse_kinematics_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
