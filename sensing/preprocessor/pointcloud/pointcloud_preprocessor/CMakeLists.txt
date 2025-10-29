cmake_minimum_required(VERSION 3.14)
project(pointcloud_preprocessor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters search)

ament_auto_find_build_dependencies()

ament_auto_add_library(pointcloud_preprocessor_component SHARED
  src/filter_node_base.cpp
  src/concatenate_data_node_component.cpp
  src/distance_based_compare_map_filter_node_component.cpp
)

target_compile_definitions(pointcloud_preprocessor_component
  PRIVATE POINTCLOUD_PREPROCESSOR_BUILDING_LIBRARY)

target_include_directories(pointcloud_preprocessor_component
  PUBLIC
    ${PCL_INCLUDE_DIRS})

target_compile_definitions(pointcloud_preprocessor_component
  PUBLIC
    ${PCL_DEFINITIONS})

target_link_libraries(pointcloud_preprocessor_component
  Eigen3::Eigen
  ${PCL_LIBRARIES})

rclcpp_components_register_nodes(pointcloud_preprocessor_component
  "pointcloud_preprocessor::ConcatenateDataNode"
  "pointcloud_preprocessor::DistanceBasedCompareMapFilterNode")

ament_auto_add_executable(concatenate_data_node
  src/concatenate_data_node_main.cpp)
target_link_libraries(concatenate_data_node pointcloud_preprocessor_component)

ament_auto_add_executable(distance_based_compare_map_filter_node
  src/distance_based_compare_map_filter_node_main.cpp)
target_link_libraries(distance_based_compare_map_filter_node pointcloud_preprocessor_component)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
