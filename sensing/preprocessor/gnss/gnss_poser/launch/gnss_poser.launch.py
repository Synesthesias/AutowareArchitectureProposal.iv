from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="$(arg base_frame)"),
        DeclareLaunchArgument("buff_epoch", default_value="10"),
        DeclareLaunchArgument("coordinate_system", default_value="1"),
        DeclareLaunchArgument("gnss_base_frame", default_value="$(arg gnss_base_frame)"),
        DeclareLaunchArgument("gnss_frame", default_value="$(arg gnss_frame)"),
        DeclareLaunchArgument("input_topic_fix", default_value="input_topic_fix"),
        DeclareLaunchArgument("input_topic_navpvt", default_value="input_topic_navpvt"),
        DeclareLaunchArgument("map_frame", default_value="$(arg map_frame)"),
        DeclareLaunchArgument("output_topic_gnss_fixed", default_value="output_topic_gnss_fixed"),
        DeclareLaunchArgument("output_topic_gnss_pose", default_value="output_topic_gnss_pose"),
        DeclareLaunchArgument("output_topic_gnss_pose_cov", default_value="output_topic_gnss_pose_cov"),
        DeclareLaunchArgument("plane_zone", default_value="9"),

        Node(
            package="gnss_poser",
            executable="gnss_poser_node",
            name="gnss_poser",
            output="screen",
            parameters=[
                {"base_frame": LaunchConfiguration("base_frame")},
                {"gnss_base_frame": LaunchConfiguration("gnss_base_frame")},
                {"gnss_frame": LaunchConfiguration("gnss_frame")},
                {"map_frame": LaunchConfiguration("map_frame")},
                {"coordinate_system": LaunchConfiguration("coordinate_system")},
                {"buff_epoch": LaunchConfiguration("buff_epoch")},
                {"plane_zone": LaunchConfiguration("plane_zone")},
            ],
            remappings=[
                ("fix", LaunchConfiguration("input_topic_fix")),
                ("navpvt", LaunchConfiguration("input_topic_navpvt")),
                ("gnss_pose", LaunchConfiguration("output_topic_gnss_pose")),
                ("gnss_pose_cov", LaunchConfiguration("output_topic_gnss_pose_cov")),
                ("gnss_fixed", LaunchConfiguration("output_topic_gnss_fixed")),
            ],
        ),
    ])
