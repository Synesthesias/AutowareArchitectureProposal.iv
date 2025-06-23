cmake_minimum_required(VERSION 3.10)
project(pointcloud_preprocessor)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# filter lib
ament_auto_add_library(filter SHARED
  src/filter.cpp
)

# concatenate lib + exec
ament_auto_add_library(concatenate_data_node SHARED
  src/concatenate_data_node.cpp
)

ament_auto_add_executable(concatenate_data_node_exec
  src/concatenate_data_node_main.cpp
)
target_link_libraries(concatenate_data_node_exec
  concatenate_data_node
  filter
)

# 🆕 compare map filter node
ament_auto_add_library(distance_based_compare_map_filter_node SHARED
  src/distance_based_compare_map_filter_node.cpp
)
target_link_libraries(distance_based_compare_map_filter_node
  filter
)

ament_auto_add_executable(distance_based_compare_map_filter_node_exec
  src/distance_based_compare_map_filter_node_main.cpp
)
target_link_libraries(distance_based_compare_map_filter_node_exec
  distance_based_compare_map_filter_node
  filter
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS concatenate_data_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS distance_based_compare_map_filter_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
