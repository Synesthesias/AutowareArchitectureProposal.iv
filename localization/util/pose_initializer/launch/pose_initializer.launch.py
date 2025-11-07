from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
  parameter_file = PathJoinSubstitution([
    FindPackageShare("pose_initializer"),
    "params",
    "pose_initializer.param.yaml",
  ])

  container = ComposableNodeContainer(
    name="pose_initializer_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container_mt",
    composable_node_descriptions=[
      ComposableNode(
        package="pose_initializer",
        plugin="pose_initializer::PoseInitializer",
        name="pose_initializer",
        parameters=[parameter_file],
        remappings=[
          ("initialpose", "/initialpose"),
          ("initialpose3d", "/initialpose3d"),
          ("gnss_pose_cov", "/sensing/gnss/pose_with_covariance"),
          ("pointcloud_map", "/map/pointcloud_map"),
          ("ndt_align_srv", "/localization/pose_estimator/ndt_align_srv"),
        ],
      ),
    ],
    output="screen",
    emulate_tty=True,
  )

  return LaunchDescription([container])
