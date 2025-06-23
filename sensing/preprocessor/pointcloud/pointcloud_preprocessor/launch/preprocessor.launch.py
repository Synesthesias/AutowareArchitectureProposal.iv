from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_preprocessor',
            executable='distance_based_compare_map_filter_node_exec',
            name='compare_map_filter',
            output='screen',
            parameters=[{'distance_threshold': 1.0}]
        ),
        Node(
            package='pointcloud_preprocessor',
            executable='concatenate_data_node_exec',
            name='concatenate_data',
            output='screen'
        )
    ])
