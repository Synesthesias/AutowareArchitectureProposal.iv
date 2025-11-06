# Copyright 2025
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    container_name = LaunchConfiguration("container_name")
    container_executable = LaunchConfiguration("container_executable")
    input_pose_topic = LaunchConfiguration("input_pose_topic")
    output_twist_topic = LaunchConfiguration("output_twist_topic")
    twist_frame_id = LaunchConfiguration("twist_frame_id")
    params_file = LaunchConfiguration("params_file")

    composable_node = ComposableNode(
        package="pose2twist",
        plugin="pose2twist::Pose2TwistComponent",
        name="pose2twist",
        remappings=[
            ("pose", input_pose_topic),
            ("twist", output_twist_topic),
        ],
        parameters=[params_file, {"twist_frame_id": twist_frame_id}],
    )

    container = ComposableNodeContainer(
        name=container_name,
        namespace="",
        package="rclcpp_components",
        executable=container_executable,
        composable_node_descriptions=[composable_node],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "container_name",
            default_value="pose2twist_container",
            description="Name of the component container",
        ),
        DeclareLaunchArgument(
            "container_executable",
            default_value="component_container_mt",
            description="Container executable to use",
        ),
        DeclareLaunchArgument(
            "input_pose_topic",
            default_value="/localization/pose_estimator/pose",
            description="Pose topic to subscribe",
        ),
        DeclareLaunchArgument(
            "output_twist_topic",
            default_value="/estimate_twist",
            description="Twist topic to publish",
        ),
        DeclareLaunchArgument(
            "twist_frame_id",
            default_value="base_link",
            description="Frame ID for published twist",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("pose2twist"), "params", "pose2twist.param.yaml"]
            ),
            description="YAML file with additional parameters",
        ),
        container,
    ])
