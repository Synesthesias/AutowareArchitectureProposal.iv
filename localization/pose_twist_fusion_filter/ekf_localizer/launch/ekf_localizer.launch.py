from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('ekf_localizer')

    param_file = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution([
            pkg_share,
            'config',
            'ekf_localizer.param.yaml',
        ]),
        description='Path to the EKF parameter file.',
    )

    input_initial_pose = DeclareLaunchArgument(
        'input_initial_pose_topic',
        default_value='initialpose',
        description='Initial pose topic.',
    )
    input_pose = DeclareLaunchArgument(
        'input_pose_topic',
        default_value='in_pose',
        description='Input pose topic.',
    )
    input_pose_with_covariance = DeclareLaunchArgument(
        'input_pose_with_covariance_topic',
        default_value='in_pose_with_covariance',
        description='Input pose with covariance topic.',
    )
    input_twist = DeclareLaunchArgument(
        'input_twist_topic',
        default_value='in_twist',
        description='Input twist topic.',
    )
    input_twist_with_covariance = DeclareLaunchArgument(
        'input_twist_with_covariance_topic',
        default_value='in_twist_with_covariance',
        description='Input twist with covariance topic.',
    )

    output_pose = DeclareLaunchArgument(
        'output_pose_topic',
        default_value='ekf_pose',
        description='Output pose topic.',
    )
    output_pose_with_covariance = DeclareLaunchArgument(
        'output_pose_with_covariance_topic',
        default_value='ekf_pose_with_covariance',
        description='Output pose with covariance topic.',
    )
    output_pose_without_yawbias = DeclareLaunchArgument(
        'output_pose_without_yawbias_topic',
        default_value='ekf_pose_without_yawbias',
        description='Output pose without yaw bias topic.',
    )
    output_pose_with_covariance_without_yawbias = DeclareLaunchArgument(
        'output_pose_with_covariance_without_yawbias_topic',
        default_value='ekf_pose_with_covariance_without_yawbias',
        description='Output pose with covariance without yaw bias topic.',
    )
    output_twist = DeclareLaunchArgument(
        'output_twist_topic',
        default_value='ekf_twist',
        description='Output twist topic.',
    )
    output_twist_with_covariance = DeclareLaunchArgument(
        'output_twist_with_covariance_topic',
        default_value='ekf_twist_with_covariance',
        description='Output twist with covariance topic.',
    )

    node = Node(
        package='ekf_localizer',
        executable='ekf_localizer',
        name='ekf_localizer',
        output='screen',
        parameters=[LaunchConfiguration('param_file')],
        remappings=[
            ('initialpose', LaunchConfiguration('input_initial_pose_topic')),
            ('in_pose', LaunchConfiguration('input_pose_topic')),
            ('in_pose_with_covariance', LaunchConfiguration('input_pose_with_covariance_topic')),
            ('in_twist', LaunchConfiguration('input_twist_topic')),
            ('in_twist_with_covariance', LaunchConfiguration('input_twist_with_covariance_topic')),
            ('ekf_pose', LaunchConfiguration('output_pose_topic')),
            (
                'ekf_pose_with_covariance',
                LaunchConfiguration('output_pose_with_covariance_topic'),
            ),
            (
                'ekf_pose_without_yawbias',
                LaunchConfiguration('output_pose_without_yawbias_topic'),
            ),
            (
                'ekf_pose_with_covariance_without_yawbias',
                LaunchConfiguration('output_pose_with_covariance_without_yawbias_topic'),
            ),
            ('ekf_twist', LaunchConfiguration('output_twist_topic')),
            (
                'ekf_twist_with_covariance',
                LaunchConfiguration('output_twist_with_covariance_topic'),
            ),
        ],
    )

    return LaunchDescription([
        param_file,
        input_initial_pose,
        input_pose,
        input_pose_with_covariance,
        input_twist,
        input_twist_with_covariance,
        output_pose,
        output_pose_with_covariance,
        output_pose_without_yawbias,
        output_pose_with_covariance_without_yawbias,
        output_twist,
        output_twist_with_covariance,
        node,
    ])
