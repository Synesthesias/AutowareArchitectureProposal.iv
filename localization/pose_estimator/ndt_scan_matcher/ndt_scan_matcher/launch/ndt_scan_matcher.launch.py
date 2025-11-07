from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
  input_sensor_points_topic = DeclareLaunchArgument(
    "input_sensor_points_topic", default_value="/points_raw",
    description="Sensor points topic")
  input_sensor_points_queue_size = DeclareLaunchArgument(
    "input_sensor_points_queue_size", default_value="1",
    description="Subscriber queue size")
  input_initial_pose_topic = DeclareLaunchArgument(
    "input_initial_pose_topic", default_value="/ekf_pose_with_covariance",
    description="Initial pose topic")
  input_map_points_topic = DeclareLaunchArgument(
    "input_map_points_topic", default_value="/pointcloud_map",
    description="Map points topic")

  output_pose_topic = DeclareLaunchArgument(
    "output_pose_topic", default_value="ndt_pose",
    description="Estimated pose topic")
  output_pose_with_covariance_topic = DeclareLaunchArgument(
    "output_pose_with_covariance_topic", default_value="ndt_pose_with_covariance",
    description="Estimated pose with covariance topic")
  output_diagnostics_topic = DeclareLaunchArgument(
    "output_diagnostics_topic", default_value="/diagnostics",
    description="Diagnostics topic")

  base_frame = DeclareLaunchArgument(
    "base_frame", default_value="base_link",
    description="Vehicle reference frame")
  node_name = DeclareLaunchArgument(
    "node_name", default_value="ndt_scan_matcher",
    description="Node name")
  container_name = DeclareLaunchArgument(
    "container_name", default_value="ndt_scan_matcher_container",
    description="Composable node container name")

  ndt_implement_type = DeclareLaunchArgument(
    "ndt_implement_type", default_value="2",
    description="0: PCL_GENERIC, 1: PCL_MODIFIED, 2: OMP")
  trans_epsilon = DeclareLaunchArgument(
    "trans_epsilon", default_value="0.01",
    description="Maximum difference between consecutive transformations")
  step_size = DeclareLaunchArgument(
    "step_size", default_value="0.1",
    description="Newton line search maximum step length")
  resolution = DeclareLaunchArgument(
    "resolution", default_value="2.0",
    description="NDT voxel grid resolution")
  max_iterations = DeclareLaunchArgument(
    "max_iterations", default_value="30",
    description="Maximum NDT iterations")
  converged_transform_probability = DeclareLaunchArgument(
    "converged_param_transform_probability", default_value="3.0",
    description="Minimum transformation probability to accept alignment")

  omp_search_method = DeclareLaunchArgument(
    "omp_neighborhood_search_method", default_value="0",
    description="OMP neighbor search: 0:KDTREE, 1:DIRECT26, 2:DIRECT7, 3:DIRECT1")
  omp_num_threads = DeclareLaunchArgument(
    "omp_num_threads", default_value="4",
    description="OMP thread count")

  node = ComposableNode(
    package="ndt_scan_matcher",
    plugin="NDTScanMatcherComponent",
    name=LaunchConfiguration("node_name"),
    parameters=[
      {
        "input_sensor_points_queue_size": LaunchConfiguration("input_sensor_points_queue_size"),
        "base_frame": LaunchConfiguration("base_frame"),
        "ndt_implement_type": LaunchConfiguration("ndt_implement_type"),
        "trans_epsilon": LaunchConfiguration("trans_epsilon"),
        "step_size": LaunchConfiguration("step_size"),
        "resolution": LaunchConfiguration("resolution"),
        "max_iterations": LaunchConfiguration("max_iterations"),
        "converged_param_transform_probability": LaunchConfiguration(
          "converged_param_transform_probability"),
        "omp_neighborhood_search_method": LaunchConfiguration(
          "omp_neighborhood_search_method"),
        "omp_num_threads": LaunchConfiguration("omp_num_threads"),
      }
    ],
    remappings=[
      ("points_raw", LaunchConfiguration("input_sensor_points_topic")),
      ("ekf_pose_with_covariance", LaunchConfiguration("input_initial_pose_topic")),
      ("pointcloud_map", LaunchConfiguration("input_map_points_topic")),
      ("ndt_pose", LaunchConfiguration("output_pose_topic")),
      ("ndt_pose_with_covariance", LaunchConfiguration(
        "output_pose_with_covariance_topic")),
      ("/diagnostics", LaunchConfiguration("output_diagnostics_topic")),
    ],
  )

  container = ComposableNodeContainer(
    name=LaunchConfiguration("container_name"),
    namespace="",
    package="rclcpp_components",
    executable="component_container_mt",
    composable_node_descriptions=[node],
    output="screen",
  )

  return LaunchDescription(
    [
      input_sensor_points_topic,
      input_sensor_points_queue_size,
      input_initial_pose_topic,
      input_map_points_topic,
      output_pose_topic,
      output_pose_with_covariance_topic,
      output_diagnostics_topic,
      base_frame,
      node_name,
      container_name,
      ndt_implement_type,
      trans_epsilon,
      step_size,
      resolution,
      max_iterations,
      converged_transform_probability,
      omp_search_method,
      omp_num_threads,
      container,
    ])
