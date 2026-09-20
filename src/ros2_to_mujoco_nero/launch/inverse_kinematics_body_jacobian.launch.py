from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_to_mujoco_nero',
            executable='inverse_kinematics_body_jacobian_node',
            name='inverse_kinematics_body_jacobian_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
