from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Arguments
    args = [
        DeclareLaunchArgument('show_debug_info', default_value='false'),
        DeclareLaunchArgument('enable_yaw_bias_estimation', default_value='True'),
        DeclareLaunchArgument('predict_frequency', default_value='50.0'),
        DeclareLaunchArgument('tf_rate', default_value='10.0'),
        DeclareLaunchArgument('extend_state_step', default_value='50'),
        DeclareLaunchArgument('input_initial_pose_name', default_value='initialpose'),

        DeclareLaunchArgument('use_pose_with_covariance', default_value='false'),
        DeclareLaunchArgument('input_pose_name', default_value='in_pose'),
        DeclareLaunchArgument('input_pose_with_cov_name', default_value='in_pose_with_covariance'),
        DeclareLaunchArgument('pose_additional_delay', default_value='0.0'),
        DeclareLaunchArgument('pose_measure_uncertainty_time', default_value='0.01'),
        DeclareLaunchArgument('pose_rate', default_value='10.0'),
        DeclareLaunchArgument('pose_gate_dist', default_value='10000.0'),
        DeclareLaunchArgument('pose_stddev_x', default_value='0.05'),
        DeclareLaunchArgument('pose_stddev_y', default_value='0.05'),
        DeclareLaunchArgument('pose_stddev_yaw', default_value='0.025'),

        DeclareLaunchArgument('use_twist_with_covariance', default_value='false'),
        DeclareLaunchArgument('input_twist_name', default_value='in_twist'),
        DeclareLaunchArgument('input_twist_with_cov_name', default_value='in_twist_with_covariance'),
        DeclareLaunchArgument('twist_additional_delay', default_value='0.0'),
        DeclareLaunchArgument('twist_rate', default_value='10.0'),
        DeclareLaunchArgument('twist_gate_dist', default_value='10000.0'),
        DeclareLaunchArgument('twist_stddev_vx', default_value='0.2'),
        DeclareLaunchArgument('twist_stddev_wz', default_value='0.03'),

        DeclareLaunchArgument('proc_stddev_yaw_c', default_value='0.005'),
        DeclareLaunchArgument('proc_stddev_yaw_bias_c', default_value='0.001'),
        DeclareLaunchArgument('proc_stddev_vx_c', default_value='5.0'),
        DeclareLaunchArgument('proc_stddev_wz_c', default_value='1.0'),

        DeclareLaunchArgument('output_pose_name', default_value='ekf_pose'),
        DeclareLaunchArgument('output_pose_with_covariance_name', default_value='ekf_pose_with_covariance'),
        DeclareLaunchArgument('output_pose_without_yawbias_name', default_value='ekf_pose_without_yawbias'),
        DeclareLaunchArgument('output_pose_with_covariance_without_yawbias_name', default_value='ekf_pose_with_covariance_without_yawbias'),
        DeclareLaunchArgument('output_twist_name', default_value='ekf_twist'),
        DeclareLaunchArgument('output_twist_with_covariance_name', default_value='ekf_twist_with_covariance')
    ]

    # Node Definition
    node = Node(
        package='ekf_localizer',
        executable='ekf_localizer_node',
        name='ekf_localizer',
        output='screen',
        remappings=[
            # Pose remaps
            ('in_pose', LaunchConfiguration('input_pose_name')),
            ('in_pose_with_covariance', 'input_pose_with_cov_UNUSED'),
            ('in_twist', LaunchConfiguration('input_twist_name')),
            ('in_twist_with_covariance', 'input_twist_with_covariance_UNUSED'),
            ('initialpose', LaunchConfiguration('input_initial_pose_name')),

            ('ekf_pose', LaunchConfiguration('output_pose_name')),
            ('ekf_pose_with_covariance', LaunchConfiguration('output_pose_with_covariance_name')),
            ('ekf_pose_without_yawbias', LaunchConfiguration('output_pose_without_yawbias_name')),
            ('ekf_pose_with_covariance_without_yawbias', LaunchConfiguration('output_pose_with_covariance_without_yawbias_name')),
            ('ekf_twist', LaunchConfiguration('output_twist_name')),
            ('ekf_twist_with_covariance', LaunchConfiguration('output_twist_with_covariance_name'))
        ],
        parameters=[{
            'pose_frame_id': 'map',
            'show_debug_info': LaunchConfiguration('show_debug_info'),
            'enable_yaw_bias_estimation': LaunchConfiguration('enable_yaw_bias_estimation'),

            'predict_frequency': LaunchConfiguration('predict_frequency'),
            'tf_rate': LaunchConfiguration('tf_rate'),
            'extend_state_step': LaunchConfiguration('extend_state_step'),

            'use_pose_with_covariance': LaunchConfiguration('use_pose_with_covariance'),
            'pose_additional_delay': LaunchConfiguration('pose_additional_delay'),
            'pose_measure_uncertainty_time': LaunchConfiguration('pose_measure_uncertainty_time'),
            'pose_rate': LaunchConfiguration('pose_rate'),
            'pose_gate_dist': LaunchConfiguration('pose_gate_dist'),
            'pose_stddev_x': LaunchConfiguration('pose_stddev_x'),
            'pose_stddev_y': LaunchConfiguration('pose_stddev_y'),
            'pose_stddev_yaw': LaunchConfiguration('pose_stddev_yaw'),

            'use_twist_with_covariance': LaunchConfiguration('use_twist_with_covariance'),
            'twist_additional_delay': LaunchConfiguration('twist_additional_delay'),
            'twist_rate': LaunchConfiguration('twist_rate'),
            'twist_gate_dist': LaunchConfiguration('twist_gate_dist'),
            'twist_stddev_vx': LaunchConfiguration('twist_stddev_vx'),
            'twist_stddev_wz': LaunchConfiguration('twist_stddev_wz'),

            'proc_stddev_yaw_c': LaunchConfiguration('proc_stddev_yaw_c'),
            'proc_stddev_yaw_bias_c': LaunchConfiguration('proc_stddev_yaw_bias_c'),
            'proc_stddev_vx_c': LaunchConfiguration('proc_stddev_vx_c'),
            'proc_stddev_wz_c': LaunchConfiguration('proc_stddev_wz_c')
        }]
    )

    # Remapping conditions (if / unless の対応)
    conditional_remaps = [
        Node(
            package='ekf_localizer',
            executable='ekf_localizer_node',
            name='ekf_localizer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_pose_with_covariance')),
            remappings=[
                ('in_pose', 'input_pose_UNUSED'),
                ('in_pose_with_covariance', LaunchConfiguration('input_pose_with_cov_name')),
            ]
        ),
        Node(
            package='ekf_localizer',
            executable='ekf_localizer_node',
            name='ekf_localizer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_twist_with_covariance')),
            remappings=[
                ('in_twist', 'input_twist_UNUSED'),
                ('in_twist_with_covariance', LaunchConfiguration('input_twist_with_cov_name')),
            ]
        )
    ]

    return LaunchDescription(args + [node] + conditional_remaps)