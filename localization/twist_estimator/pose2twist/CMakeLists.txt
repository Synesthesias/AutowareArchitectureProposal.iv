cmake_minimum_required(VERSION 3.14)
project(pose2twist)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(pose2twist_component SHARED
  src/pose2twist_component.cpp
)

rclcpp_components_register_nodes(pose2twist_component "pose2twist::Pose2TwistComponent")

ament_auto_add_executable(pose2twist_node
  src/pose2twist_node.cpp
)
target_link_libraries(pose2twist_node pose2twist_component)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    params
)
