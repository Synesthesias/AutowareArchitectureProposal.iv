from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pose_initializer',
            executable='pose_initializer_node',
            name='pose_initializer',
            output='screen',
            parameters=[{}],
        )
    ])
