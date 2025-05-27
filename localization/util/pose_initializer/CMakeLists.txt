cmake_minimum_required(VERSION 3.14)
project(pose_initializer)

find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)  # Add this line
ament_auto_find_build_dependencies()

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})  # Add this line

ament_auto_add_executable(${PROJECT_NAME}_node
  src/pose_initializer_node.cpp
  src/pose_initializer_core.cpp
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
