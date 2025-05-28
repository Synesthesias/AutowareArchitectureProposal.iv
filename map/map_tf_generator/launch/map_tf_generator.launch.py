from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='map_tf_generator',
            executable='map_tf_generator_node',
            name='map_tf_generator',
            parameters=[{
                'x': 0.0,
                'y': 0.0,
                'z': 0.0,
                'yaw': 0.0,
                'parent_frame_id': 'map',
                'child_frame_id': 'base_link'
            }]
        )
    ])
