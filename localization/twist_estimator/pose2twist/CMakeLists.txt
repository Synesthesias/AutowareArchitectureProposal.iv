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

target_include_directories(pose2twist_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
