cmake_minimum_required(VERSION 3.14)
project(ndt_scan_matcher)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

ament_auto_add_library(ndt_scan_matcher_component SHARED
  src/ndt_scan_matcher_core.cpp
)

rclcpp_components_register_nodes(ndt_scan_matcher_component "NDTScanMatcherComponent")

target_include_directories(ndt_scan_matcher_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_auto_add_executable(ndt_scan_matcher_node src/ndt_scan_matcher_node.cpp)
target_link_libraries(ndt_scan_matcher_node ndt_scan_matcher_component)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_auto_package(
  INSTALL_TO_SHARE launch
)
