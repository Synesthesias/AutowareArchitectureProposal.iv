cmake_minimum_required(VERSION 3.16)
project(pose_initializer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/pose_initializer_component.cpp
)

target_include_directories(${PROJECT_NAME}_component
  PUBLIC
    include
    ${PCL_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME}_component
  PUBLIC
    ${PCL_DEFINITIONS}
)

target_link_libraries(${PROJECT_NAME}_component
  ${PCL_LIBRARIES}
)
target_compile_options(${PROJECT_NAME}_component PRIVATE -Wall -Wextra -Wpedantic)

rclcpp_components_register_nodes(${PROJECT_NAME}_component "pose_initializer::PoseInitializer")

ament_auto_add_executable(${PROJECT_NAME}_node src/pose_initializer_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_component)

ament_auto_package(
  INSTALL_TO_SHARE launch params
  USE_SCOPED_HEADER_INSTALL_DIR
)
