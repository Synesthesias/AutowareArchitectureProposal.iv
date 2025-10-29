from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    input_topics_arg = DeclareLaunchArgument(
        "input_topics",
        default_value="['/points_raw']",
        description="List of pointcloud topics to concatenate.",
    )
    output_frame_arg = DeclareLaunchArgument(
        "output_frame",
        default_value="base_link",
        description="Target frame for concatenated clouds.",
    )
    timeout_sec_arg = DeclareLaunchArgument(
        "timeout_sec",
        default_value="0.1",
        description="Timeout before publishing partial concatenated cloud [s].",
    )
    max_queue_size_arg = DeclareLaunchArgument(
        "max_queue_size",
        default_value="10",
        description="Maximum buffered clouds per subscription.",
    )
    use_twist_compensation_arg = DeclareLaunchArgument(
        "use_twist_compensation",
        default_value="true",
        description="Whether to compensate motion using twist data.",
    )
    twist_topic_arg = DeclareLaunchArgument(
        "twist_topic",
        default_value="/vehicle/status/twist",
        description="Topic providing twist stamped messages for compensation.",
    )
    max_twist_dt_arg = DeclareLaunchArgument(
        "max_twist_dt",
        default_value="0.1",
        description="Maximum delta between twist samples used for compensation [s].",
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic",
        default_value="/points_raw/concatenated",
        description="Output topic for the concatenated point cloud.",
    )

    concatenate_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::ConcatenateDataNode",
        name="concatenate_data",
        parameters=[
            {
                "input_topics": ParameterValue(LaunchConfiguration("input_topics"), value_type=list),
                "output_frame": LaunchConfiguration("output_frame"),
                "timeout_sec": LaunchConfiguration("timeout_sec"),
                "max_queue_size": LaunchConfiguration("max_queue_size"),
                "use_twist_compensation": LaunchConfiguration("use_twist_compensation"),
                "twist_topic": LaunchConfiguration("twist_topic"),
                "max_twist_dt": LaunchConfiguration("max_twist_dt"),
            }
        ],
        remappings=[
            ("output", LaunchConfiguration("output_topic")),
        ],
    )

    container = ComposableNodeContainer(
        name="pointcloud_preprocessor_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[concatenate_component],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            input_topics_arg,
            output_frame_arg,
            timeout_sec_arg,
            max_queue_size_arg,
            use_twist_compensation_arg,
            twist_topic_arg,
            max_twist_dt_arg,
            output_topic_arg,
            container,
        ]
    )
