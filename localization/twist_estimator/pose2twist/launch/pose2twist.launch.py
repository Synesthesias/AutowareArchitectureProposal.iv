from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_pose_topic',
            default_value='/localization/pose_estimator/pose'
        ),
        DeclareLaunchArgument(
            'output_twist_topic',
            default_value='/estimate_twist'
        ),
        Node(
            package='pose2twist',
            executable='pose2twist',
            name='pose2twist',
            output='log',
            remappings=[
                ('pose', LaunchConfiguration('input_pose_topic')),
                ('twist', LaunchConfiguration('output_twist_topic')),
            ]
        )
    ])
